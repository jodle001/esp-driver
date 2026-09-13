// Motor currents and the motor-rail voltage from two ADS1015 boards:
//   0x48: A0-A3 = CS1-4 (driver current-sense, ~20 mV/A plus an offset, +-256 mV range)
//   0x49: A0 = CS5, A1 = VSENSE (rail through 39 k / 10 k, read at +-4.096 V)
// A task cycles the channels in single-shot mode; the control loop reads a snapshot.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CURRENT_COUNT 5

typedef struct {
    i2c_master_bus_handle_t bus;
    uint8_t addr_a;          /* 0x48 */
    uint8_t addr_b;          /* 0x49 */
    uint32_t speed_hz;
    float mv_per_a;          /* 20 */
    float offset_mv;         /* ~50: CS output at 0 A */
    float vsense_ratio;      /* 4.9: rail = VSENSE x ratio */
    float rail_ok_v;         /* 9.0: below this the rail is off (e-stop) */
} current_config_t;

typedef struct {
    float current_a[CURRENT_COUNT]; /* filtered, magnitude */
    float rail_v;
    bool rail_ok;
    bool bus_error;
} current_snapshot_t;

esp_err_t current_init(const current_config_t *cfg);
void current_collect(current_snapshot_t *out);

#ifdef __cplusplus
}
#endif
