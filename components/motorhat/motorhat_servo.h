// Closed-loop jog for the polar pan commands. motorhat.c hands polar pan start
// and stop to these functions when motorhat_config_t.servo.enabled is set; the
// control law itself is servo.c.
#ifndef _MOTORHAT_SERVO_H_
#define _MOTORHAT_SERVO_H_

#include "motorhat.h"

/**
 * @brief Start the control task. Called by motorhat_init after the HATs are up.
 */
esp_err_t motorhat_servo_init(motorhat_handle_t* handle,
                              const motorhat_servo_config_t* config);

/**
 * @brief Polar pan start: each delta's sign is that joint's direction, 0
 * stops it. Commander repeats the start every 100 ms while tracking; a repeat
 * only keeps the joint running.
 */
void motorhat_servo_start(int8_t delta_azimuth, int8_t delta_altitude);

/**
 * @brief Polar pan stop: both joints ramp down and hold. Also clears a fault.
 */
void motorhat_servo_stop(void);

/**
 * @brief Stop driving at once (overcurrent). Motors are left as the caller set
 * them; the next start takes the reference from where they are.
 */
void motorhat_servo_halt(void);

#endif  // _MOTORHAT_SERVO_H_
