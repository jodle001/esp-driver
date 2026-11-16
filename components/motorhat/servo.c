#include "servo.h"

#include <math.h>
#include <string.h>

static float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

void servo_joint_init(servo_joint_t* j, const servo_joint_params_t* p, int n,
                      const servo_motor_params_t mp[]) {
  memset(j, 0, sizeof(*j));
  j->p = *p;
  j->n = n < SERVO_MAX_MOTORS ? n : SERVO_MAX_MOTORS;
  for (int i = 0; i < j->n; i++) {
    j->mp[i] = mp[i];
  }
  j->state = SERVO_IDLE;
}

// Joint position the motors are at now: the average of their positions, so a
// small offset between two wrist motors is split between them
static float measured_pos(const servo_joint_t* j, const int32_t counts[]) {
  float sum = 0.0f;
  for (int i = 0; i < j->n; i++) {
    sum += (float)j->mp[i].sign * ((float)counts[i] - j->m[i].origin);
  }
  return sum / (float)j->n;
}

// IDLE -> ACTIVE: take the reference from where the motors are. A motor
// further than max_error/2 from that (moved by hand, slipped) gets a new
// origin instead of being yanked back.
static void activate(servo_joint_t* j, const int32_t counts[]) {
  float pos = measured_pos(j, counts);
  for (int i = 0; i < j->n; i++) {
    float mine = (float)j->mp[i].sign * ((float)counts[i] - j->m[i].origin);
    if (fabsf(mine - pos) > 0.5f * j->p.max_error) {
      j->m[i].origin = (float)counts[i] - (float)j->mp[i].sign * pos;
    }
    j->m[i].integral = 0.0f;
    j->m[i].err_time = 0.0f;
    j->m[i].stall_time = 0.0f;
    j->m[i].stall_ref = counts[i];
  }
  j->pos = pos;
  j->v_ref = 0.0f;
  j->settled_time = 0.0f;
  j->state = SERVO_ACTIVE;
}

void servo_joint_command(servo_joint_t* j, int dir, const int32_t counts[]) {
  dir = dir > 0 ? 1 : (dir < 0 ? -1 : 0);
  if (dir == 0) {
    if (j->state == SERVO_FAULT) {
      j->state = SERVO_IDLE;
      j->fault = SERVO_FAULT_NONE;
    }
    j->dir = 0;
    return;
  }
  if (j->state == SERVO_FAULT) {
    return;  // needs a stop first
  }
  if (j->state == SERVO_IDLE) {
    activate(j, counts);
  }
  j->dir = dir;
  j->settled_time = 0.0f;
}

void servo_joint_halt(servo_joint_t* j) {
  j->dir = 0;
  j->v_ref = 0.0f;
  if (j->state == SERVO_ACTIVE) {
    j->state = SERVO_IDLE;
  }
  for (int i = 0; i < j->n; i++) {
    j->m[i].integral = 0.0f;
    j->m[i].duty = 0.0f;
  }
}

static void fault(servo_joint_t* j, servo_fault_t why, int motor) {
  j->state = SERVO_FAULT;
  j->fault = why;
  j->fault_motor = motor;
  j->dir = 0;
  j->v_ref = 0.0f;
  for (int i = 0; i < j->n; i++) {
    j->m[i].integral = 0.0f;
    j->m[i].duty = 0.0f;
  }
}

// Reference speed and position for this step
static void update_reference(servo_joint_t* j, float dt) {
  const servo_joint_params_t* p = &j->p;
  float v_des = (float)j->dir * p->jog_speed;
  j->at_limit = false;
  if (p->limited) {
    // never faster than the speed that can still stop at the limit
    float up = p->max_pos - j->pos;
    float dn = j->pos - p->min_pos;
    float cap_up = up > 0.0f ? sqrtf(2.0f * p->accel * up) : 0.0f;
    float cap_dn = dn > 0.0f ? sqrtf(2.0f * p->accel * dn) : 0.0f;
    if (v_des > cap_up) {
      v_des = cap_up;
      j->at_limit = j->dir > 0;
    }
    if (v_des < -cap_dn) {
      v_des = -cap_dn;
      j->at_limit = j->dir < 0;
    }
  }
  float dv = p->accel * dt;
  j->v_ref += clampf(v_des - j->v_ref, -dv, dv);
  j->pos += j->v_ref * dt;
  if (p->limited) {
    if (j->pos > p->max_pos) {
      j->pos = p->max_pos;
      j->v_ref = 0.0f;
    } else if (j->pos < p->min_pos) {
      j->pos = p->min_pos;
      j->v_ref = 0.0f;
    }
  }
}

bool servo_joint_update(servo_joint_t* j, const int32_t counts[], float dt,
                        float duty[]) {
  if (j->state != SERVO_ACTIVE) {
    for (int i = 0; i < j->n; i++) {
      duty[i] = 0.0f;
    }
    return false;
  }

  update_reference(j, dt);
  const servo_joint_params_t* p = &j->p;
  bool moving = j->v_ref != 0.0f;
  bool settled = !moving && j->dir == 0;

  for (int i = 0; i < j->n; i++) {
    const servo_motor_params_t* mp = &j->mp[i];
    servo_motor_t* m = &j->m[i];
    float target = m->origin + (float)mp->sign * j->pos;
    float e = target - (float)counts[i];
    m->err = e;
    if (fabsf(e) > p->hold_window) {
      settled = false;
    }

    // A motor that only needs to hold, inside the window, is left alone so it
    // neither hums nor winds its integral up against static friction
    bool hold_still = !moving && fabsf(e) <= p->hold_window;

    float u =
        p->kff * (float)mp->sign * j->v_ref + p->kp * e + p->ki * m->integral;
    if (hold_still) {
      u = 0.0f;
    } else if (u != 0.0f) {
      u += copysignf(mp->deadband, u);
    }
    if (u > mp->max_duty || u < -mp->max_duty) {
      u = clampf(u, -mp->max_duty, mp->max_duty);
      // anti-windup: only integrate in the direction that unsaturates
      if ((u > 0.0f) != (e > 0.0f)) {
        m->integral += e * dt;
      }
    } else if (!hold_still) {
      m->integral += e * dt;
    }
    // the integral alone may never ask for more than the duty cap
    if (p->ki > 0.0f) {
      float i_max = mp->max_duty / p->ki;
      m->integral = clampf(m->integral, -i_max, i_max);
    }
    m->duty = u;
    duty[i] = u;

    // following error
    if (fabsf(e) > p->max_error) {
      m->err_time += dt;
      if (m->err_time >= p->fault_s) {
        fault(j, SERVO_FAULT_FOLLOWING, i);
        break;
      }
    } else {
      m->err_time = 0.0f;
    }

    // stall: driven near the cap but not moving
    if (fabsf(u) >= 0.9f * mp->max_duty) {
      if (fabsf((float)(counts[i] - m->stall_ref)) >= p->stall_counts) {
        m->stall_ref = counts[i];
        m->stall_time = 0.0f;
      } else {
        m->stall_time += dt;
        if (m->stall_time >= p->fault_s) {
          fault(j, SERVO_FAULT_STALL, i);
          break;
        }
      }
    } else {
      m->stall_ref = counts[i];
      m->stall_time = 0.0f;
    }
  }

  // Two motors of one joint: they must stay together, a seized motor or one
  // turning the wrong way shows here long before its own following error
  if (j->state == SERVO_ACTIVE && j->n == 2 && j->p.max_pair > 0.0f) {
    float a = (float)j->mp[0].sign * ((float)counts[0] - j->m[0].origin);
    float b = (float)j->mp[1].sign * ((float)counts[1] - j->m[1].origin);
    if (fabsf(a - b) > j->p.max_pair) {
      fault(j, SERVO_FAULT_PAIR,
            fabsf(j->m[0].err) > fabsf(j->m[1].err) ? 0 : 1);
    }
  }

  if (j->state != SERVO_ACTIVE) {
    for (int i = 0; i < j->n; i++) {
      duty[i] = 0.0f;
    }
    return false;
  }

  if (settled && p->hold_s > 0.0f) {
    j->settled_time += dt;
    if (j->settled_time >= p->hold_s) {
      servo_joint_halt(j);
      for (int i = 0; i < j->n; i++) {
        duty[i] = 0.0f;
      }
      return false;
    }
  } else {
    j->settled_time = 0.0f;
  }
  return true;
}
