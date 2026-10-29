#ifndef _MOTORHAT_H_
#define _MOTORHAT_H_

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "pca9685.h"

#define DEFAULT_FREQUENCY_HZ 1526.0f

/**
 * @brief Motor enumeration
 */
typedef enum {
  MOTORHAT_MOTOR1 = 0,
  MOTORHAT_MOTOR2,
  MOTORHAT_MOTOR3,
  MOTORHAT_MOTOR4,
  MOTORHAT_NUM_MOTORS
} motorhat_motor_t;

/**
 * @brief Axis enumeration
 */
typedef enum {
  MOTORHAT_AXIS_AZIMUTH = 0,
  MOTORHAT_AXIS_ALTITUDE,
  MOTORHAT_AXIS_ALTITUDE_B, /**< Optional second altitude motor, must stay last */
  MOTORHAT_NUM_AXES
} motorhat_axis_t;

/**
 * @brief Soft limit bounds for one axis in raw encoder counts
 *
 * Count 0 is the position the arm was in when the ESP32 booted (the manually homed position)
 * An axis may not be driven past min_count or max_count
 */
typedef struct {
  int32_t min_count; /**< Lowest allowed count, always <= 0 */
  int32_t max_count; /**< Highest allowed count, always >= 0 */
} motorhat_soft_limit_t;

/**
 * @brief Function callback for reading the encoder count of one axis
 *
 * @param[in] ctx Callback context (axis_count_ctx)
 * @param[in] axis Axis to read
 * @param[out] count Raw encoder count of that axis
 *
 * @return ESP_OK on success
 */
typedef esp_err_t (*motorhat_axis_count_cb_t)(void* ctx, motorhat_axis_t axis, int* count);

/**
 * @brief Function callback for clearing encoder count
 */
typedef void (*motorhat_encoder_cb_t)(void* ctx);

/**
 * @brief Motor direction and state control
 */
typedef enum {
  MOTORHAT_DIRECTION_FORWARD = 0, /**< Motor rotates forward */
  MOTORHAT_DIRECTION_BACKWARD,    /**< Motor rotates backward */
  MOTORHAT_DIRECTION_BRAKE,       /**< Motor brakes (both inputs high) */
  MOTORHAT_DIRECTION_RELEASE /**< Motor released/coasting (both inputs low) */
} motorhat_direction_t;

/**
 * @brief Channel mapping for a single motor
 *
 * Maps the three PCA9685 channels required to control one motor:
 * - IN1 and IN2: Direction control inputs to the H-bridge
 * - PWM: Speed control via pulse-width modulation
 */
typedef struct {
  pca9685_channel_t in1_channel; /**< H-bridge IN1 control channel */
  pca9685_channel_t in2_channel; /**< H-bridge IN2 control channel */
  pca9685_channel_t pwm_channel; /**< PWM speed control channel */
} motorhat_motor_channels_t;

/**
 * @brief Motor HAT configuration structure
 */
typedef struct {
  pca9685_config_t pca9685_config;
  uint16_t polar_pan_speed; /**< Speed to use for polar pan movements */
  motorhat_encoder_cb_t encoder_cb; /**< Clear encoder count function callback */
  void* encoder_ctx; /** < Clear encoder count callback context (the encoder handle) */
  gpio_num_t limit_gpio; /**< GPIO number for limit switch pin */

  bool soft_limits_enabled; /**< Enforce soft limits when true */
  motorhat_soft_limit_t soft_limits[MOTORHAT_NUM_AXES]; /**< Per axis bounds indexed by motorhat_axis_t */
  motorhat_axis_count_cb_t axis_count_cb; /**< Reads an axis's encoder count, required when soft_limits_enabled */
  void* axis_count_ctx; /**< Context passed to axis_count_cb */
  bool forward_increases_count[MOTORHAT_NUM_AXES]; /**< Per axis, true if FORWARD makes the encoder count go up */
  uint32_t soft_limit_poll_ms; /**< How often the soft limit monitor checks moving axes */
  uint32_t max_run_ms; /**< Stop an axis driven this long without a new start command, 0 disables */
  motorhat_motor_t axis_motor[MOTORHAT_NUM_AXES]; /**< Motor HAT port driving each axis, indexed by motorhat_axis_t */
  bool altitude_b_enabled; /**< Altitude commands also drive MOTORHAT_AXIS_ALTITUDE_B */
  bool altitude_b_reversed; /**< Drive ALTITUDE_B opposite to ALTITUDE (pitch on a differential wrist) */
  uint16_t axis_speed[MOTORHAT_NUM_AXES]; /**< Polar pan PWM duty (0 to PCA9685_PWM_MAX) for each axis */
} motorhat_config_t;

/**
 * @brief Motor HAT device handle
 */
typedef struct {
  pca9685_handle_t pca9685; /**< Handle for the underlying PCA9685 controller */
  uint16_t polar_pan_speed; /**< Speed to use for polar pan movements */
  motorhat_encoder_cb_t encoder_cb; /**< Clear encoder count function callback */
  void* encoder_ctx; /** < Clear encoder count callback context (the encoder handle) */
  gpio_num_t limit_gpio; /**< GPIO number for limit switch pin */

  bool soft_limits_enabled; /**< Enforce soft limits when true */
  motorhat_soft_limit_t soft_limits[MOTORHAT_NUM_AXES]; /**< Per axis bounds indexed by motorhat_axis_t */
  motorhat_axis_count_cb_t axis_count_cb; /**< Reads an axis's encoder count, required when soft_limits_enabled */
  void* axis_count_ctx; /**< Context passed to axis_count_cb */
  bool forward_increases_count[MOTORHAT_NUM_AXES]; /**< Per axis, true if FORWARD makes the encoder count go up */
  uint32_t soft_limit_poll_ms; /**< How often the soft limit monitor checks moving axes */
  uint32_t max_run_ms; /**< Stop an axis driven this long without a new start command, 0 disables */
  motorhat_motor_t axis_motor[MOTORHAT_NUM_AXES]; /**< Motor HAT port driving each axis, indexed by motorhat_axis_t */
  bool altitude_b_reversed; /**< Drive ALTITUDE_B opposite to ALTITUDE (pitch on a differential wrist) */
  int num_axes; /**< Axes in use, MOTORHAT_NUM_AXES with ALTITUDE_B enabled and one fewer without */
  uint16_t axis_speed[MOTORHAT_NUM_AXES]; /**< Polar pan PWM duty (0 to PCA9685_PWM_MAX) for each axis */
} motorhat_handle_t;

/**
 * @brief Channel assignments for each motor on the Adafruit Motor HAT
 *
 * This mapping is specific to the Adafruit Motor HAT v2 pinout:
 * - Motor 1: IN1=CH10, IN2=CH9,  PWM=CH8
 * - Motor 2: IN1=CH11, IN2=CH12, PWM=CH13
 * - Motor 3: IN1=CH4,  IN2=CH3,  PWM=CH2
 * - Motor 4: IN1=CH5,  IN2=CH6,  PWM=CH7
 */
static const motorhat_motor_channels_t motor_channels[MOTORHAT_NUM_MOTORS] = {
    {PCA9685_CHANNEL10, PCA9685_CHANNEL9, PCA9685_CHANNEL8},    // Motor 1
    {PCA9685_CHANNEL11, PCA9685_CHANNEL12, PCA9685_CHANNEL13},  // Motor 2
    {PCA9685_CHANNEL4, PCA9685_CHANNEL3, PCA9685_CHANNEL2},     // Motor 3
    {PCA9685_CHANNEL5, PCA9685_CHANNEL6, PCA9685_CHANNEL7}      // Motor 4
};

/**
 * @brief Initialize the Motor HAT controller
 *
 * Initializes the underlying PCA9685 PWM controller with the provided
 * configuration. After successful initialization, motors can be controlled
 * using the speed and direction functions.
 *
 * @param[out] handle Pointer to Motor HAT handle structure
 * @param[in] config Pointer to configuration structure containing PCA9685
 * settings
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *    - ESP_ERR_*: Other ESP-IDF error codes from PCA9685 initialization
 *
 * @note The handle->pca9685 pointer must be allocated and point to a valid
 *       pca9685_handle_t structure before calling this function
 */
esp_err_t motorhat_init(motorhat_handle_t* handle,
                        const motorhat_config_t* config);

/**
 * @brief Polar pan command
 *
 * Triggers a polar pan movement based on the specified azimuth and altitude
 * deltas, with timing parameters for delay and duration. This is a high-level
 * command that can be implemented to control multiple motors in coordination to
 * achieve the desired pan movement.
 *
 * @param[in] delta_azimuth Azimuth delta for the pan movement
 * @param[in] delta_altitude Altitude delta for the pan movement
 * @param[in] delay_ms Delay before starting the movement
 * @param[in] time_ms Duration of the movement
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_*: Other ESP-IDF error codes propogated from motor control
 * functions
 */
esp_err_t motorhat_polar_pan(int16_t delta_azimuth, int16_t delta_altitude,
                             uint16_t delay_ms, uint16_t time_ms);

/**
 * @brief Polar pan start command
 *
 * Starts a continuous pan movement that will run until a stop command is
 * received. Only uses two axes currently, directly relating azimuth to axis 1 and altitude to axis 2.
 *
 * @param[in] delta_azimuth Azimuth delta for the pan movement
 * @param[in] delta_altitude Altitude delta for the pan movement
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_*: Other ESP-IDF error codes propogated from motor control
 * functions
 */
esp_err_t motorhat_polar_pan_start(int8_t delta_azimuth, int8_t delta_altitude);

/**
 * @brief Polar pan stop command
 *
 * Stops any ongoing polar pan movement initiated by polar_pan_start.
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_*: Other ESP-IDF error codes propogated from motor control
 * functions
 */
esp_err_t motorhat_polar_pan_stop(void);

/**
 * @brief Home command
 *
 * Moves the motors to a predefined home position after an optional delay. Blocks high level movement commands
 * such as polar pan, setting the motor speed and direction directly to operate around these commands.
 *
 * The homing sequence is modeled after the sequence demonstrated by the SCORBOT ER-V, since both robots
 * have the unusual placement of their limit switches at the centers of their corresponding axis's range,
 * rather than at the edges. The one difference is that this homing sequence starts with moving each motor FORWARD,
 * and ends with it moving BACKWARD to release the limit switch. This is because for the vertical axis, the slowed
 * down motion to release the limit switch and reach the final homed position currently stalls the motor when moving upward,
 * so I've reversed the order so that it happens while moving downward. This should be fixed by replacing the
 * current motor drivers with ones capable of pushing the proper amount of amps to drive the motors,
 * but in the meantime reversing the order of FORWARD and BACKWArd motion in homing avoids this issue.
 *
 * IMPORTANT: until current sensing is fully functional, the robot can't detect reaching the edge of it's range and will
 * break if you attempt to home it while the arm is on the wrong side of the limit switch.
 *
 * @param[in] delay_ms Delay before starting the homing movement
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_*: Other ESP-IDF error codes propogated from motor control
 * functions
 */
esp_err_t motorhat_home(uint16_t delay_ms);

/**
 * @brief Set motor speed
 *
 * Sets the PWM duty cycle controlling motor speed. The speed does not affect
 * direction - use motorhat_set_motor_direction() to control direction.
 *
 * @param[in] handle Pointer to Motor HAT handle
 * @param[in] motor Motor identifier (MOTORHAT_MOTOR1 through MOTORHAT_MOTOR4)
 * @param[in] speed Speed value (0-4096, where 0=stopped, 4096=full speed)
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL handle, invalid motor, or
 * speed > 4096)
 *    - ESP_ERR_INVALID_STATE: Motor is in fault state (stop bits set)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 *
 * @note For typical usage with 0-255 speed range, scale your value:
 *       actual_speed = (your_speed * PCA9685_PWM_MAX) / 255
 */
esp_err_t motorhat_set_motor_speed(motorhat_handle_t* handle,
                                   motorhat_motor_t motor, uint16_t speed);

/**
 * @brief Set motor direction
 *
 * Controls the motor's direction and braking state by setting the IN1 and IN2
 * control signals appropriately:
 * - FORWARD: IN1=HIGH, IN2=LOW
 * - BACKWARD: IN1=LOW, IN2=HIGH
 * - BRAKE: IN1=HIGH, IN2=HIGH (active braking)
 * - RELEASE: IN1=LOW, IN2=LOW (coasting/free-running)
 *
 * @param[in] handle Pointer to Motor HAT handle
 * @param[in] motor Motor identifier (MOTORHAT_MOTOR1 through MOTORHAT_MOTOR4)
 * @param[in] direction Desired direction/state
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL handle, invalid motor, or
 * invalid direction)
 *    - ESP_ERR_INVALID_STATE: Motor is in fault state (stop bits set)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 *
 * @note Setting direction does not affect speed. Set speed separately using
 *       motorhat_set_motor_speed()
 */
esp_err_t motorhat_set_motor_direction(motorhat_handle_t* handle,
                                       motorhat_motor_t motor,
                                       motorhat_direction_t direction);

/**
 * @brief Emergency stop all motors
 *
 * Immediately stops all motors and releases control signals.
 *
 * @param[in] handle Pointer to Motor HAT handle
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL handle)
 */

esp_err_t motorhat_emergency_stop(motorhat_handle_t* handle);

#endif  // _MOTORHAT_H_
