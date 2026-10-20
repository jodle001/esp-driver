// Up to eight Pololu G2 18v17 driver modules (rev 0.4: seven, the arm, the gripper and
// the rail): PWM from LEDC (20 kHz, one pin each), DIR through the DIR expander's port A,
// FLT read back through the sense expander, and the shared SLP line on a GPIO with the
// board's 2.2 k pull-down so the drivers stay asleep through reset. Duty in -1..+1;
// brake = PWM low with SLP high (the G2 brakes low-side when PWM is 0), disabled = SLP
// low (outputs off, motors coast).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "expander.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_MAX 8

typedef struct {
    int count;                /* drivers, 1..7 (LEDC channels 0..count-1; channel 7 is the counters' clock) */
    int pwm_gpio[MOTOR_MAX];
    uint32_t pwm_freq_hz;     /* 20000 */
    int slp_gpio;             /* shared !SLP to all drivers, high = awake */
    expander_t *dir_expander; /* the DIR chip */
} motor_config_t;

typedef struct {
    motor_config_t cfg;
    uint32_t duty_max;
    bool enabled;
    uint8_t port_a;
} motor_t;

esp_err_t motor_init(motor_t *m, const motor_config_t *cfg);
/// Apply one control period's outputs: duties (-1..+1), brake flags and the SLP line.
/// SLP drops first when disabling; one I2C write for DIR when it changed, then the LEDC
/// updates, then SLP rises when enabling.
esp_err_t motor_apply(motor_t *m, const float *duty, const bool *brake, bool drives_enabled);
/// FLT bits from the sense chip's port A (active low): bit i = driver i faulted.
uint8_t motor_faults_from_port(uint8_t sense_a, int count);

#ifdef __cplusplus
}
#endif
