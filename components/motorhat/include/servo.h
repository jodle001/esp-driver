// Closed-loop jog control for the Motor HAT axes. Plain C with no ESP-IDF
// calls, so it builds and runs on a PC as well as on the ESP32.
//
// A joint is one commanded direction (azimuth or altitude). It owns a speed
// reference that ramps to +-jog_speed on a start and back to 0 on a stop, and
// a position reference integrated from it. Each motor of the joint (one, or
// both wrist motors) follows that same position with feedforward + PI, so two
// wrist motors cannot drift apart. Positions are raw encoder counts.
#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

#define SERVO_MAX_MOTORS 2

typedef struct {
  float jog_speed;     // counts/s of the reference while a start is held
  float accel;         // counts/s^2, ramp of the reference
  float kff;           // duty per count/s of reference speed
  float kp;            // duty per count of position error
  float ki;            // duty per count*s of integrated error
  float max_error;     // counts; a motor further behind than this faults...
  float fault_s;       // ...after this long, and so does a motor that is
  float stall_counts;  // driven near max duty and moves fewer counts than this
  float hold_s;        // settled this long after a stop: brake, 0 holds forever
  float hold_window;   // counts; "settled" and "on target" band
  float max_pair;      // counts; two motors this far apart fault at once
  bool limited;        // clamp the joint position to [min_pos, max_pos]
  float min_pos;
  float max_pos;
} servo_joint_params_t;

typedef struct {
  int sign;        // +1 if this motor's count rises when the joint moves +
  float max_duty;  // 0 to 1, the duty cap (keeps a stall inside the driver)
  float deadband;  // duty the motor needs before it moves at all
} servo_motor_params_t;

typedef enum {
  SERVO_IDLE = 0,  // motors braked, nothing driven
  SERVO_ACTIVE,    // the loop drives the motors (moving or holding)
  SERVO_FAULT,     // stopped by a fault, cleared by a stop command
} servo_state_t;

typedef enum {
  SERVO_FAULT_NONE = 0,
  SERVO_FAULT_FOLLOWING,  // a motor fell more than max_error behind
  SERVO_FAULT_STALL,      // a motor driven hard did not move
  SERVO_FAULT_PAIR,       // the two motors of a joint came apart
} servo_fault_t;

typedef struct {
  float origin;       // count at joint position 0
  float integral;     // count*s
  float duty;         // last output, + raises the count
  float err;          // last position error, counts
  float err_time;     // s spent beyond max_error
  float stall_time;   // s spent driven hard without moving
  int32_t stall_ref;  // count when the stall timer started
} servo_motor_t;

typedef struct {
  servo_joint_params_t p;
  servo_motor_params_t mp[SERVO_MAX_MOTORS];
  servo_motor_t m[SERVO_MAX_MOTORS];
  int n;  // motors in this joint
  servo_state_t state;
  servo_fault_t fault;
  int fault_motor;
  int dir;             // -1, 0 or +1, the last command
  float v_ref;         // counts/s
  float pos;           // counts, the joint position reference
  float settled_time;  // s settled with a stop commanded
  bool at_limit;       // the reference is being held back by a soft limit
} servo_joint_t;

// Origins start at 0, so count 0 (the boot position) is joint position 0.
void servo_joint_init(servo_joint_t* j, const servo_joint_params_t* p, int n,
                      const servo_motor_params_t mp[]);

// dir -1, 0 or +1. A repeated start is ignored apart from keeping it running.
// A stop (0) ramps down and holds, and also clears a fault.
void servo_joint_command(servo_joint_t* j, int dir, const int32_t counts[]);

// Stop driving at once (overcurrent, emergency stop): IDLE, integrals cleared.
void servo_joint_halt(servo_joint_t* j);

// One control step. counts[] are the motors' encoder counts. duty[] gets each
// motor's duty, -max_duty to +max_duty, + meaning "raise the count".
// Returns true while the motors must be driven; false means brake them.
bool servo_joint_update(servo_joint_t* j, const int32_t counts[], float dt,
                        float duty[]);

#endif
