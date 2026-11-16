#include "motorhat_servo.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "signal_bus.h"

#define TAG "motorhat_servo"

enum { JOINT_AZIMUTH = 0, JOINT_ALTITUDE, NUM_JOINTS };
static const char* const joint_name[NUM_JOINTS] = {"azimuth", "altitude"};
static const char* const state_name[] = {"idle", "active", "fault"};
static const char* const fault_name[] = {"none", "following error", "stall",
                                         "motors apart"};

static motorhat_handle_t* s_hat = NULL;
static motorhat_servo_config_t s_cfg;
static SemaphoreHandle_t s_lock = NULL;

static servo_joint_t s_joint[NUM_JOINTS];
// The axis (motorhat_axis_t) of each motor of each joint
static motorhat_axis_t s_joint_axis[NUM_JOINTS][SERVO_MAX_MOTORS];
// Tick of the last start that drove each joint, for the max run check
static TickType_t s_started[NUM_JOINTS];
// Last counts read, for the debug log
static int32_t s_counts[NUM_JOINTS][SERVO_MAX_MOTORS];
// A start refused by a fault was logged; commander repeats starts every 100 ms
static bool s_refusal_logged[NUM_JOINTS];

// What was last written to each axis's motor, so unchanged outputs cost no I2C
static motorhat_direction_t s_dir[MOTORHAT_NUM_AXES];
static uint16_t s_pwm[MOTORHAT_NUM_AXES];
static bool s_written[MOTORHAT_NUM_AXES];

static int sign_of(int v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }

// Caller holds s_lock
static esp_err_t read_counts(int j, int32_t counts[]) {
  for (int i = 0; i < s_joint[j].n; i++) {
    int count = 0;
    esp_err_t err =
        s_hat->axis_count_cb(s_hat->axis_count_ctx, s_joint_axis[j][i], &count);
    if (err != ESP_OK) {
      return err;
    }
    counts[i] = count;
    s_counts[j][i] = count;
  }
  return ESP_OK;
}

// Drives one axis's motor at duty (+ raises its count) or brakes it.
// Caller holds s_lock
static esp_err_t write_axis(motorhat_axis_t axis, float duty, bool drive) {
  motorhat_direction_t dir = MOTORHAT_DIRECTION_BRAKE;
  uint16_t pwm = 0;
  if (drive && duty != 0.0f) {
    bool raise = duty > 0.0f;
    dir = raise == s_hat->forward_increases_count[axis]
              ? MOTORHAT_DIRECTION_FORWARD
              : MOTORHAT_DIRECTION_BACKWARD;
    float mag = fminf(fabsf(duty), 1.0f);
    pwm = (uint16_t)lroundf(mag * (float)PCA9685_PWM_MAX);
  }
  if (s_written[axis] && dir == s_dir[axis] && pwm == s_pwm[axis]) {
    return ESP_OK;
  }

  motorhat_motor_t motor = s_hat->axis_motor[axis];
  esp_err_t err = ESP_OK;
  if (!s_written[axis] || dir != s_dir[axis]) {
    // speed 0 first so the H-bridge never changes direction at speed
    err = motorhat_set_motor_speed(s_hat, motor, 0);
    if (err == ESP_OK) {
      err = motorhat_set_motor_direction(s_hat, motor, dir);
    }
  }
  if (err == ESP_OK) {
    err = motorhat_set_motor_speed(s_hat, motor, pwm);
  }
  s_written[axis] = err == ESP_OK;
  s_dir[axis] = dir;
  s_pwm[axis] = pwm;
  return err;
}

// Caller holds s_lock
static void halt_all(void) {
  for (int j = 0; j < NUM_JOINTS; j++) {
    servo_joint_halt(&s_joint[j]);
  }
  // whoever stopped the motors wrote them last, so resend on the next drive
  for (int axis = 0; axis < MOTORHAT_NUM_AXES; axis++) {
    s_written[axis] = false;
  }
}

static void control_task(void* args) {
  TickType_t period = pdMS_TO_TICKS(s_cfg.period_ms);
  if (period == 0) {
    period = 1;
  }
  const float dt = (float)(period * portTICK_PERIOD_MS) / 1000.0f;
  const TickType_t max_run = pdMS_TO_TICKS(s_hat->max_run_ms);
  servo_state_t last_state[NUM_JOINTS] = {SERVO_IDLE, SERVO_IDLE};

  TickType_t last_wake = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&last_wake, period);

    // Messages are built under the lock and logged after it, so a slow UART
    // never holds up the motors
    bool timed_out[NUM_JOINTS] = {false};
    bool changed[NUM_JOINTS] = {false};
    servo_joint_t snapshot[NUM_JOINTS];
    memset(snapshot, 0, sizeof(snapshot));
    bool write_failed = false;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (xEventGroupGetBits(g_motor_events) & (HOMING_FLAG | CURRENT_ANY)) {
      // homing drives the motors itself; an overcurrent stop is under way
      halt_all();
      xSemaphoreGive(s_lock);
      continue;
    }

    TickType_t now = xTaskGetTickCount();
    for (int j = 0; j < NUM_JOINTS && !write_failed; j++) {
      servo_joint_t* joint = &s_joint[j];
      if (joint->n == 0) {
        continue;
      }
      int32_t counts[SERVO_MAX_MOTORS] = {0};
      if (read_counts(j, counts) != ESP_OK) {
        write_failed = true;  // can't control blind: stop everything
        break;
      }
      if (joint->dir != 0 && max_run > 0 && now - s_started[j] > max_run) {
        servo_joint_command(joint, 0, counts);
        timed_out[j] = true;
      }

      float duty[SERVO_MAX_MOTORS];
      bool drive = servo_joint_update(joint, counts, dt, duty);
      for (int i = 0; i < joint->n; i++) {
        if (write_axis(s_joint_axis[j][i], duty[i], drive) != ESP_OK) {
          write_failed = true;
          break;
        }
      }
      changed[j] = joint->state != last_state[j];
      last_state[j] = joint->state;
      snapshot[j] = *joint;
    }
    if (write_failed) {
      halt_all();
      motorhat_emergency_stop(s_hat);
    }
    xSemaphoreGive(s_lock);

    if (write_failed) {
      ESP_LOGE(TAG, "Encoder read or motor write failed, all motors stopped");
      continue;
    }
    for (int j = 0; j < NUM_JOINTS; j++) {
      if (timed_out[j]) {
        ESP_LOGW(TAG, "%s ran %" PRIu32 " ms without a new start, stopping",
                 joint_name[j], s_hat->max_run_ms);
      }
      if (!changed[j]) {
        continue;
      }
      if (snapshot[j].state == SERVO_FAULT) {
        ESP_LOGE(TAG,
                 "%s FAULT: %s on axis %d (error %.0f counts), stopped. Send a "
                 "stop to clear it",
                 joint_name[j], fault_name[snapshot[j].fault],
                 s_joint_axis[j][snapshot[j].fault_motor],
                 snapshot[j].m[snapshot[j].fault_motor].err);
      } else {
        ESP_LOGI(TAG, "%s %s", joint_name[j], state_name[snapshot[j].state]);
      }
    }
  }
}

// Low priority: prints each active joint every log_ms for tuning
static void log_task(void* args) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(s_cfg.log_ms));
    servo_joint_t snapshot[NUM_JOINTS];
    int32_t counts[NUM_JOINTS][SERVO_MAX_MOTORS];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(snapshot, s_joint, sizeof(snapshot));
    memcpy(counts, s_counts, sizeof(counts));
    xSemaphoreGive(s_lock);

    for (int j = 0; j < NUM_JOINTS; j++) {
      const servo_joint_t* joint = &snapshot[j];
      if (joint->n == 0 || joint->state != SERVO_ACTIVE) {
        continue;
      }
      char line[160];
      int len = snprintf(line, sizeof(line), "%s ref %.0f v %.0f",
                         joint_name[j], joint->pos, joint->v_ref);
      for (int i = 0; i < joint->n && len > 0 && len < (int)sizeof(line); i++) {
        len += snprintf(line + len, sizeof(line) - (size_t)len,
                        " | axis %d cnt %" PRId32 " err %.0f duty %.2f",
                        s_joint_axis[j][i], counts[j][i], joint->m[i].err,
                        joint->m[i].duty);
      }
      ESP_LOGI(TAG, "%s", line);
    }
  }
}

esp_err_t motorhat_servo_init(motorhat_handle_t* handle,
                              const motorhat_servo_config_t* config) {
  if (handle == NULL || config == NULL || handle->axis_count_cb == NULL) {
    ESP_LOGE(TAG, "Closed loop needs the axis count callback");
    return ESP_ERR_INVALID_ARG;
  }
  s_hat = handle;
  s_cfg = *config;

  // Azimuth: one motor. Altitude: one, or both wrist motors
  servo_motor_params_t mp[SERVO_MAX_MOTORS];
  s_joint_axis[JOINT_AZIMUTH][0] = MOTORHAT_AXIS_AZIMUTH;
  mp[0] = config->motor[MOTORHAT_AXIS_AZIMUTH];
  servo_joint_init(&s_joint[JOINT_AZIMUTH], &config->joint[JOINT_AZIMUTH], 1,
                   mp);

  int n = 1;
  s_joint_axis[JOINT_ALTITUDE][0] = MOTORHAT_AXIS_ALTITUDE;
  mp[0] = config->motor[MOTORHAT_AXIS_ALTITUDE];
  if (handle->num_axes > MOTORHAT_AXIS_ALTITUDE_B) {
    s_joint_axis[JOINT_ALTITUDE][1] = MOTORHAT_AXIS_ALTITUDE_B;
    mp[1] = config->motor[MOTORHAT_AXIS_ALTITUDE_B];
    n = 2;
  }
  servo_joint_init(&s_joint[JOINT_ALTITUDE], &config->joint[JOINT_ALTITUDE], n,
                   mp);

  s_lock = xSemaphoreCreateMutex();
  if (s_lock == NULL) {
    return ESP_ERR_NO_MEM;
  }
  // Above the old soft limit task (9): this task now does its job
  if (xTaskCreate(control_task, "servo_task", 4096, NULL, 10, NULL) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }
  if (config->log_ms > 0 &&
      xTaskCreate(log_task, "servo_log", 3072, NULL, 2, NULL) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  for (int j = 0; j < NUM_JOINTS; j++) {
    const servo_joint_params_t* p = &config->joint[j];
    ESP_LOGI(TAG,
             "Closed loop %s: %d motor(s), jog %.0f counts/s, limits %.0f to "
             "%.0f",
             joint_name[j], s_joint[j].n, p->jog_speed, p->min_pos, p->max_pos);
  }
  return ESP_OK;
}

void motorhat_servo_start(int8_t delta_azimuth, int8_t delta_altitude) {
  if (s_lock == NULL) {
    return;
  }
  const int dirs[NUM_JOINTS] = {sign_of(delta_azimuth),
                                sign_of(delta_altitude)};
  bool refused[NUM_JOINTS] = {false};

  xSemaphoreTake(s_lock, portMAX_DELAY);
  TickType_t now = xTaskGetTickCount();
  for (int j = 0; j < NUM_JOINTS; j++) {
    if (s_joint[j].n == 0) {
      continue;
    }
    int32_t counts[SERVO_MAX_MOTORS] = {0};
    if (read_counts(j, counts) != ESP_OK) {
      continue;
    }
    servo_joint_command(&s_joint[j], dirs[j], counts);
    if (dirs[j] != 0) {
      s_started[j] = now;
    }
    if (s_joint[j].state != SERVO_FAULT) {
      s_refusal_logged[j] = false;
    } else if (dirs[j] != 0 && !s_refusal_logged[j]) {
      refused[j] = true;
      s_refusal_logged[j] = true;
    }
  }
  xSemaphoreGive(s_lock);

  for (int j = 0; j < NUM_JOINTS; j++) {
    if (refused[j]) {
      ESP_LOGW(TAG, "%s is in a fault, send a stop first", joint_name[j]);
    }
  }
}

void motorhat_servo_stop(void) {
  if (s_lock == NULL) {
    return;
  }
  xSemaphoreTake(s_lock, portMAX_DELAY);
  for (int j = 0; j < NUM_JOINTS; j++) {
    int32_t counts[SERVO_MAX_MOTORS] = {0};
    servo_joint_command(&s_joint[j], 0, counts);  // a stop ignores counts
    s_refusal_logged[j] = false;
  }
  xSemaphoreGive(s_lock);
}

void motorhat_servo_halt(void) {
  if (s_lock == NULL) {
    return;
  }
  xSemaphoreTake(s_lock, portMAX_DELAY);
  halt_all();
  xSemaphoreGive(s_lock);
}
