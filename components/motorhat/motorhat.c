#include "motorhat.h"

#include <inttypes.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "signal_bus.h"

#define TAG "motorhat"

static motorhat_handle_t* s_handle = NULL;

// Last direction commanded on each axis
static motorhat_direction_t s_axis_direction[MOTORHAT_NUM_AXES];

// Tick of the last start command that drove each axis, for the max run check
static TickType_t s_axis_started[MOTORHAT_NUM_AXES];

// Held while checking soft limits and writing axis motor state. Check and motor
// writes that follow can't be intertwined with another task
static SemaphoreHandle_t s_axis_lock = NULL;

// Defined below with the other soft limit functions, started by motorat_init
static void soft_limit_monitor_task(void* args);

static motorhat_direction_t delta_to_direction(int8_t delta) {
  if (delta > 0) return MOTORHAT_DIRECTION_FORWARD;
  if (delta < 0) return MOTORHAT_DIRECTION_BACKWARD;

  return MOTORHAT_DIRECTION_RELEASE;
}

static motorhat_direction_t reverse_direction(motorhat_direction_t direction) {
  if (direction == MOTORHAT_DIRECTION_FORWARD)
    return MOTORHAT_DIRECTION_BACKWARD;
  if (direction == MOTORHAT_DIRECTION_BACKWARD)
    return MOTORHAT_DIRECTION_FORWARD;

  return direction;
}

void motor_stop_task(void* args) {
  while (1) {
    // Block indefinitely until any stop bit is set
    xEventGroupWaitBits(g_motor_events, CURRENT_ANY,
                        pdFALSE,  // don't clear bits on exit
                        pdFALSE,  // any bit (OR)
                        portMAX_DELAY);

    // Check for homing sequence before reacting
    if (!(xEventGroupGetBits(g_motor_events) & HOMING_FLAG)) {
      motorhat_emergency_stop(s_handle);
    } else {
      // Homing sequence will handle this, yield and check again shortly
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

esp_err_t motorhat_init(motorhat_handle_t* handle,
                        const motorhat_config_t* config) {
  if (handle == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  // ALTITUDE_B is the last axis, so leaving it out just shortens the axis loops
  int num_axes =
      config->altitude_b_enabled ? MOTORHAT_NUM_AXES : MOTORHAT_AXIS_ALTITUDE_B;
  // Each axis needs its own valid port, or two axes would fight over one motor
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < num_axes; axis++) {
    motorhat_motor_t motor = config->axis_motor[axis];
    if (motor < MOTORHAT_MOTOR1 || motor >= MOTORHAT_NUM_MOTORS) {
      ESP_LOGE(TAG, "Axis %d mapped to invalid motor %d", axis, motor + 1);
      return ESP_ERR_INVALID_ARG;
    }
    if (config->axis_speed[axis] > PCA9685_PWM_MAX) {
      ESP_LOGE(TAG, "Axis %d speed %d above the maximum %d", axis,
               config->axis_speed[axis], PCA9685_PWM_MAX);
      return ESP_ERR_INVALID_ARG;
    }
    for (int other = MOTORHAT_AXIS_AZIMUTH; other < axis; other++) {
      if (config->axis_motor[other] == motor) {
        ESP_LOGE(TAG, "Axes %d and %d both mapped to motor %d", other, axis,
                 motor + 1);
        return ESP_ERR_INVALID_ARG;
      }
    }
  }

  if (config->soft_limits_enabled) {
    if (config->axis_count_cb == NULL) {
      ESP_LOGE(TAG, "Soft limits enabled but no axis count callback given");
      return ESP_ERR_INVALID_ARG;
    }

    for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < num_axes; axis++) {
      int count = 0;
      esp_err_t err =
          config->axis_count_cb(config->axis_count_ctx, axis, &count);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d count read failed: %s", axis,
                 esp_err_to_name(err));
        return err;
      }

      if (config->soft_limits[axis].min_count > 0 ||
          config->soft_limits[axis].max_count < 0) {
        ESP_LOGE(TAG, "Axis %d soft limits do not contain the boot position",
                 axis);
        return ESP_ERR_INVALID_ARG;
      }
    }
  }

  s_axis_lock = xSemaphoreCreateMutex();
  if (s_axis_lock == NULL) {
    return ESP_ERR_NO_MEM;
  }

  // FORWARD is 0 so the zero initialized array must be set explicitly
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    s_axis_direction[axis] = MOTORHAT_DIRECTION_RELEASE;
  }

  s_handle = handle;
  handle->polar_pan_speed = config->polar_pan_speed;
  handle->encoder_cb = config->encoder_cb;
  handle->encoder_ctx = config->encoder_ctx;
  handle->limit_gpio = config->limit_gpio;

  handle->soft_limits_enabled = config->soft_limits_enabled;
  handle->axis_count_cb = config->axis_count_cb;
  handle->axis_count_ctx = config->axis_count_ctx;
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    handle->soft_limits[axis] = config->soft_limits[axis];
    handle->forward_increases_count[axis] =
        config->forward_increases_count[axis];
    handle->axis_motor[axis] = config->axis_motor[axis];
    handle->axis_speed[axis] = config->axis_speed[axis];
  }
  handle->soft_limit_poll_ms = config->soft_limit_poll_ms;
  handle->max_run_ms = config->max_run_ms;
  handle->altitude_b_reversed = config->altitude_b_reversed;
  handle->num_axes = num_axes;

  esp_err_t err = pca9685_init(&handle->pca9685, &config->pca9685_config);
  if (err != ESP_OK) {
    return err;
  }

  // Started after the PCA9685 is ready; it may emergency stop at once.
  xTaskCreate(motor_stop_task, "motor_stop_task", 4096, handle, 8, NULL);

  // Watches moving axes for soft limits and the max run time
  if (handle->soft_limits_enabled || handle->max_run_ms > 0) {
    if (xTaskCreate(soft_limit_monitor_task, "soft_limit_task", 4096, NULL, 9,
                    NULL) != pdPASS) {
      return ESP_ERR_NO_MEM;
    }
  }

  return ESP_OK;
}

// Socket commands
esp_err_t motorhat_home(uint16_t delay_ms) {
  if (xEventGroupGetBits(g_motor_events) & HOMING_FLAG) {
    ESP_LOGW(TAG, "Homing in progress, cannot send commands");
    return ESP_ERR_INVALID_STATE;
  }

  // Homing drives each axis until a current or limit switch event, which the
  // soft limits would interrupt and it re-zeroes the encoders the soft limits
  // are measured from so we need to disable homing
  if (s_handle->soft_limits_enabled) {
    ESP_LOGW(TAG, "Soft limits enabled, home command rejected");
    return ESP_ERR_NOT_SUPPORTED;
  }

  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;

  ESP_LOGI(TAG, "Received home command: delay_ms=%d", delay_ms);

  // Set homing flag
  xEventGroupSetBits(g_motor_events, HOMING_FLAG);

  // Add delay
  vTaskDelay(pdMS_TO_TICKS(delay_ms));

  // Stop any movement before homing
  for (int m = MOTORHAT_MOTOR1; m < MOTORHAT_NUM_MOTORS; m++) {
    motorhat_set_motor_speed(s_handle, m, 0);
    motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_RELEASE);
  }

  // Home each axis individually until we candetermine which motor triggers
  // limit switch and current events
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < s_handle->num_axes; axis++) {
    int m = s_handle->axis_motor[axis];
    ESP_LOGI(TAG, "Homing motor %d", m);

    motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_FORWARD);
    motorhat_set_motor_speed(s_handle, m, s_handle->polar_pan_speed);

    EventBits_t bits = xEventGroupWaitBits(g_motor_events, HOME_EVENT, pdTRUE,
                                           pdFALSE, portMAX_DELAY);

    // Stop when a homing event is detected
    motorhat_set_motor_speed(s_handle, m, 0);

    if (bits & CURRENT_ANY) {
      ESP_LOGI(TAG, "Motor %d hit the forward end of its range", m);

      // Small pause before going opposite direction
      vTaskDelay(pdMS_TO_TICKS(200));

      // Switch directions and move towards limit switch
      motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_BACKWARD);
      motorhat_set_motor_speed(s_handle, m, s_handle->polar_pan_speed);

      bits = xEventGroupWaitBits(g_motor_events, HOME_EVENT, pdTRUE, pdFALSE,
                                 portMAX_DELAY);

      // Stop when a homing event is detected
      motorhat_set_motor_speed(s_handle, m, 0);

      if (bits & LIMIT_ANY) {
        ESP_LOGI(TAG, "Motor %d hit a limit switch moving backward", m);

        // Continue forward slowly until limit switch releases
        motorhat_set_motor_speed(s_handle, m, s_handle->polar_pan_speed / 3);

        // Detect limit switch release by polling
        while (!gpio_get_level(s_handle->limit_gpio)) {
          vTaskDelay(pdMS_TO_TICKS(10));
        }
        ESP_LOGI(TAG, "Motor %d finished homing", m);

        // Homing finished, stop the motor and continue sequence
        motorhat_set_motor_speed(s_handle, m, 0);
        motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_RELEASE);

      } else {
        ESP_LOGW(TAG, "Motor %d unexpected homing event detected", m);
        motorhat_emergency_stop(s_handle);
      }

    } else if (bits & LIMIT_ANY) {
      ESP_LOGI(TAG, "Motor %d hit a limit switch moving forward", m);

      // Small pause before going the opposite direction
      vTaskDelay(pdMS_TO_TICKS(200));

      // Turn around and move slowly until limit switch releases
      motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_BACKWARD);
      motorhat_set_motor_speed(s_handle, m, s_handle->polar_pan_speed / 3);

      // Detect limit switch release by polling
      while (!gpio_get_level(s_handle->limit_gpio)) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      ESP_LOGI(TAG, "Motor %d finished homing", m);

      // Homing finished, stop the motor and continue sequence
      motorhat_set_motor_speed(s_handle, m, 0);
      motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_RELEASE);

    } else {
      ESP_LOGW(TAG, "Motor %d unexpected homing event detected", m);
      motorhat_emergency_stop(s_handle);
    }
  }

  // Reset encoder values
  if (s_handle->encoder_cb) {
    s_handle->encoder_cb(s_handle->encoder_ctx);
    ESP_LOGI(TAG, "Homing successful, encoders reset");
  } else {
    ESP_LOGW(TAG, "Encoder reset callback not found, homing unsuccessful");
  }

  // Clear homing flag
  xEventGroupClearBits(g_motor_events, HOMING_FLAG);

  return ESP_OK;
}

esp_err_t motorhat_polar_pan(int16_t delta_azimuth, int16_t delta_altitude,
                             uint16_t delay_ms, uint16_t time_ms) {
  if (xEventGroupGetBits(g_motor_events) & HOMING_FLAG) {
    ESP_LOGW(TAG, "Homing in progress, cannot send commands");
    return ESP_ERR_INVALID_STATE;
  }

  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;

  ESP_LOGI(TAG,
           "Received polar pan command: delta_azimuth=%d, delta_altitude=%d, "
           "delay_ms=%d, time_ms=%d",
           delta_azimuth, delta_altitude, delay_ms, time_ms);

  // TODO: Implement polar pan logic using motorhat_set_motor_direction and
  // motorhat_set_motor_speed
  return ESP_OK;
}

// Returns true if axis may be driven in this direction.
// BRAKE and RELEASE are always allowed.
// FORWARD and BACKWARD are refused if they would move the count further past
// the soft limit the axis has already reached. Caller must hold s_axis_lock
static bool soft_limit_allows(motorhat_axis_t axis,
                              motorhat_direction_t direction) {
  if (direction != MOTORHAT_DIRECTION_FORWARD &&
      direction != MOTORHAT_DIRECTION_BACKWARD) {
    return true;
  }

  int count = 0;
  esp_err_t err =
      s_handle->axis_count_cb(s_handle->axis_count_ctx, axis, &count);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Axis %d count read failed (%s), refusing to move it", axis,
             esp_err_to_name(err));
    return false;
  }

  // if direction is forward and going forward makes encoder count go up, true
  // if direction is forward and going forward makes encoder go down, false
  // if direction is backward and going forward makes count go up, false
  // if direction is backward and going forward makes count go down, true
  // works out whether this command pushes the count up or down
  bool count_increases = (direction == MOTORHAT_DIRECTION_FORWARD) ==
                         s_handle->forward_increases_count[axis];
  const motorhat_soft_limit_t* limit = &s_handle->soft_limits[axis];

  if (count_increases && count >= limit->max_count) {
    ESP_LOGW(TAG, "Axis %d at max soft limit (count %d, max %" PRId32 ")", axis,
             count, limit->max_count);
    return false;
  }

  if (!count_increases && count <= limit->min_count) {
    ESP_LOGW(TAG, "Axis %d at min soft limit (count %d, min %" PRId32 ")", axis,
             count, limit->min_count);
    return false;
  }

  return true;
}

// Brakes one axis. If that fails, stop every motor instead so a failed write
// can never leave a motor running.
// Caller must hold s_axis_lock.
static void stop_axis(motorhat_axis_t axis) {
  esp_err_t err =
      motorhat_set_motor_speed(s_handle, s_handle->axis_motor[axis], 0);
  if (err == ESP_OK) {
    err = motorhat_set_motor_direction(s_handle, s_handle->axis_motor[axis],
                                       MOTORHAT_DIRECTION_BRAKE);
  }

  if (err == ESP_OK) {
    s_axis_direction[axis] = MOTORHAT_DIRECTION_BRAKE;
    return;
  }

  ESP_LOGE(TAG, "Axis %d stop failed (%s), stopping all motors", axis,
           esp_err_to_name(err));
  motorhat_emergency_stop(s_handle);
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    s_axis_direction[axis] = MOTORHAT_DIRECTION_RELEASE;
  }
}

// With two altitude motors the wrist only pitches while both turn, and one
// turning alone rolls it, so stopping either stops both.
// Caller must hold s_axis_lock.
static void stop_axis_and_partner(motorhat_axis_t axis) {
  stop_axis(axis);
  if (s_handle->num_axes <= MOTORHAT_AXIS_ALTITUDE_B) {
    return;
  }
  if (axis == MOTORHAT_AXIS_ALTITUDE) {
    stop_axis(MOTORHAT_AXIS_ALTITUDE_B);
  } else if (axis == MOTORHAT_AXIS_ALTITUDE_B) {
    stop_axis(MOTORHAT_AXIS_ALTITUDE);
  }
}

// Every soft_limit_poll_ms, stops any axis being driven past its soft limit
// (the same rule as polar pan start, soft_limit_allows, so a moving axis stops
// where a new start in that direction would be refused) or driven longer than
// max_run_ms without a new start command.
static void soft_limit_monitor_task(void* args) {
  // At CONFIG_FREERTOS_HZ=100 a tick is 10ms so shorter periods round down to 0
  // ticks which vTaskDelayUntil does not allow
  TickType_t period = pdMS_TO_TICKS(s_handle->soft_limit_poll_ms);
  if (period == 0) {
    period = 1;
  }
  TickType_t max_run_ticks = pdMS_TO_TICKS(s_handle->max_run_ms);

  TickType_t last_wake = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&last_wake, period);

    xSemaphoreTake(s_axis_lock, portMAX_DELAY);
    TickType_t now = xTaskGetTickCount();
    for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < s_handle->num_axes; axis++) {
      motorhat_direction_t direction = s_axis_direction[axis];
      if (direction != MOTORHAT_DIRECTION_FORWARD &&
          direction != MOTORHAT_DIRECTION_BACKWARD) {
        continue;
      }

      if (s_handle->soft_limits_enabled &&
          !soft_limit_allows(axis, direction)) {
        ESP_LOGW(TAG, "Axis %d stopped at soft limit", axis);
        stop_axis_and_partner(axis);
      } else if (max_run_ticks > 0 &&
                 now - s_axis_started[axis] > max_run_ticks) {
        ESP_LOGW(TAG, "Axis %d ran %" PRIu32 " ms without a new start, stopped",
                 axis, s_handle->max_run_ms);
        stop_axis_and_partner(axis);
      }
    }
    xSemaphoreGive(s_axis_lock);
  }
}

// Applies a polar pan start to botha axes. Axes that soft limits refuse are
// brakes and left at speed 0. Caller must hold s_axis_lock.
static esp_err_t start_axes(
    motorhat_direction_t directions[MOTORHAT_NUM_AXES]) {
  bool blocked[MOTORHAT_NUM_AXES] = {false};
  if (s_handle->soft_limits_enabled) {
    for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < s_handle->num_axes; axis++) {
      if (!soft_limit_allows(axis, directions[axis])) {
        directions[axis] = MOTORHAT_DIRECTION_BRAKE;
        blocked[axis] = true;
      }
    }
  }

  // Never drive one of two altitude motors alone, it would roll the wrist
  if (s_handle->num_axes > MOTORHAT_AXIS_ALTITUDE_B &&
      (blocked[MOTORHAT_AXIS_ALTITUDE] || blocked[MOTORHAT_AXIS_ALTITUDE_B])) {
    directions[MOTORHAT_AXIS_ALTITUDE] = MOTORHAT_DIRECTION_BRAKE;
    directions[MOTORHAT_AXIS_ALTITUDE_B] = MOTORHAT_DIRECTION_BRAKE;
    blocked[MOTORHAT_AXIS_ALTITUDE] = true;
    blocked[MOTORHAT_AXIS_ALTITUDE_B] = true;
  }

  // set both axes speed to 0 to prevent direction change while moving
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < s_handle->num_axes; axis++) {
    esp_err_t err =
        motorhat_set_motor_speed(s_handle, s_handle->axis_motor[axis], 0);
    if (err != ESP_OK) {
      return err;
    }
  }

  // Set directions
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < s_handle->num_axes; axis++) {
    esp_err_t err = motorhat_set_motor_direction(
        s_handle, s_handle->axis_motor[axis], directions[axis]);
    if (err != ESP_OK) {
      return err;
    }
    s_axis_direction[axis] = directions[axis];
  }

  // Set speeds to a default value
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < s_handle->num_axes; axis++) {
    if (blocked[axis]) {
      continue;
    }
    esp_err_t err = motorhat_set_motor_speed(
        s_handle, s_handle->axis_motor[axis], s_handle->axis_speed[axis]);
    if (err != ESP_OK) {
      return err;
    }
    s_axis_started[axis] = xTaskGetTickCount();
  }

  return ESP_OK;
}

esp_err_t motorhat_polar_pan_start(int8_t delta_azimuth,
                                   int8_t delta_altitude) {
  if (xEventGroupGetBits(g_motor_events) & HOMING_FLAG) {
    ESP_LOGW(TAG, "Homing in progress, cannot send commands");
    return ESP_ERR_INVALID_STATE;
  }

  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;

  ESP_LOGI(
      TAG,
      "Received polar pan start command: delta_azimuth=%d, delta_altitude=%d",
      delta_azimuth, delta_altitude);

  motorhat_direction_t directions[MOTORHAT_NUM_AXES] = {
      [MOTORHAT_AXIS_AZIMUTH] = delta_to_direction(delta_azimuth),
      [MOTORHAT_AXIS_ALTITUDE] = delta_to_direction(delta_altitude),
      [MOTORHAT_AXIS_ALTITUDE_B] = delta_to_direction(delta_altitude),
  };
  if (s_handle->altitude_b_reversed) {
    directions[MOTORHAT_AXIS_ALTITUDE_B] =
        reverse_direction(directions[MOTORHAT_AXIS_ALTITUDE_B]);
  }

  xSemaphoreTake(s_axis_lock, portMAX_DELAY);
  esp_err_t err = start_axes(directions);
  xSemaphoreGive(s_axis_lock);

  return err;
}

esp_err_t motorhat_polar_pan_stop(void) {
  if (xEventGroupGetBits(g_motor_events) & HOMING_FLAG) {
    ESP_LOGW(TAG, "Homing in progress, cannot send commands");
    return ESP_ERR_INVALID_STATE;
  }

  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;

  ESP_LOGI(TAG, "Received polar pan stop command");

  xSemaphoreTake(s_axis_lock, portMAX_DELAY);
  for (int m = MOTORHAT_MOTOR1; m < MOTORHAT_NUM_MOTORS; m++) {
    motorhat_set_motor_speed(s_handle, m, 0);
    motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_BRAKE);
  }
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    s_axis_direction[axis] = MOTORHAT_DIRECTION_BRAKE;
  }
  xSemaphoreGive(s_axis_lock);
  return ESP_OK;
}

// Motor control to be used by socket command functions
esp_err_t motorhat_set_motor_speed(motorhat_handle_t* handle,
                                   motorhat_motor_t motor, uint16_t speed) {
  if (handle == NULL || motor < MOTORHAT_MOTOR1 ||
      motor >= MOTORHAT_NUM_MOTORS) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xEventGroupGetBits(g_motor_events) & CURRENT_ANY) {
    ESP_LOGW(TAG, "Motor %d in fault state, cannot set speed", motor);
    return ESP_ERR_INVALID_STATE;
  }

  if (speed > PCA9685_PWM_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  const motorhat_motor_channels_t* channels = &motor_channels[motor];

  return pca9685_set_duty_cycle(&handle->pca9685, channels->pwm_channel, speed);
}

esp_err_t motorhat_set_motor_direction(motorhat_handle_t* handle,
                                       motorhat_motor_t motor,
                                       motorhat_direction_t direction) {
  if (handle == NULL || motor < MOTORHAT_MOTOR1 ||
      motor >= MOTORHAT_NUM_MOTORS) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xEventGroupGetBits(g_motor_events) & CURRENT_ANY) {
    ESP_LOGW(TAG, "Motor %d in fault state, cannot set direction", motor);
    return ESP_ERR_INVALID_STATE;
  }

  const motorhat_motor_channels_t* channels = &motor_channels[motor];

  esp_err_t err;

  switch (direction) {
    case MOTORHAT_DIRECTION_FORWARD:
      err =
          pca9685_digital_write(&handle->pca9685, channels->in1_channel, true);
      if (err != ESP_OK) return err;
      err =
          pca9685_digital_write(&handle->pca9685, channels->in2_channel, false);
      err = ESP_OK;
      break;
    case MOTORHAT_DIRECTION_BACKWARD:
      err =
          pca9685_digital_write(&handle->pca9685, channels->in1_channel, false);
      if (err != ESP_OK) return err;
      err =
          pca9685_digital_write(&handle->pca9685, channels->in2_channel, true);
      err = ESP_OK;
      break;
    case MOTORHAT_DIRECTION_BRAKE:
      err =
          pca9685_digital_write(&handle->pca9685, channels->in1_channel, true);
      if (err != ESP_OK) return err;
      err =
          pca9685_digital_write(&handle->pca9685, channels->in2_channel, true);
      err = ESP_OK;
      break;
    case MOTORHAT_DIRECTION_RELEASE:
      err =
          pca9685_digital_write(&handle->pca9685, channels->in1_channel, false);
      if (err != ESP_OK) return err;
      err =
          pca9685_digital_write(&handle->pca9685, channels->in2_channel, false);
      err = ESP_OK;
      break;
    default:
      return ESP_ERR_INVALID_ARG;
  }

  return err;
}

// Emergency stop on all motors that bypasses normal logic flow
esp_err_t motorhat_emergency_stop(motorhat_handle_t* handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t first_err = ESP_OK;
  for (int m = MOTORHAT_MOTOR1; m < MOTORHAT_NUM_MOTORS; m++) {
    const motorhat_motor_channels_t* ch = &motor_channels[m];

    // Stop PWM cycle and release both control lines
    esp_err_t err =
        pca9685_set_duty_cycle(&handle->pca9685, ch->pwm_channel, 0);
    if (err != ESP_OK && first_err == ESP_OK) first_err = err;

    err = pca9685_digital_write(&handle->pca9685, ch->in1_channel, false);
    if (err != ESP_OK && first_err == ESP_OK) first_err = err;

    err = pca9685_digital_write(&handle->pca9685, ch->in2_channel, false);
    if (err != ESP_OK && first_err == ESP_OK) first_err = err;
  }
  return first_err;
}
