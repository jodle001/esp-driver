// Limit switches (SW1-7 on the sense chip's port B), driver fault lines (FLT1-7 on its
// port A) and the MOTORS ON chain sense (GPB0 of the DIR chip), with the count latch.
//
// The shared INT line fires on any change of a switch, a fault line or the chain sense.
// The task it wakes reads every counter first (the latch, about 40 us after the edge:
// at homing speed a fraction of a count), then INTF/INTCAP of both chips over I2C to
// learn which input moved and to clear the line. The control loop collects the result
// once per tick: per-joint switch level, whether an edge happened, and the count latched
// at it. (The LS7366R index latch, wired on the board, can replace the software latch
// once its polarity is confirmed; see encoder_ls7366r.h.)
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "expander.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LIMIT_SWITCH_MAX 8

typedef struct {
    int int_gpio;             /* the shared INT line */
    bool active_low;          /* switch pressed reads 0 */
    expander_t *sense;        /* FLT and SW */
    expander_t *dir;          /* ESTOP_SENSE on its GPB0 */
    int count;                /* joints, 1..7 */
    uint32_t debounce_us;     /* 2000 */
} limit_switch_config_t;

typedef struct {
    bool pressed;
    bool edge;             /* changed since the last collect AND latched_count is valid */
    int32_t latched_count; /* raw count read right after the interrupt */
} limit_switch_joint_t;

typedef struct {
    limit_switch_joint_t joint[LIMIT_SWITCH_MAX];
    uint8_t fault_mask;    /* driver FLT lines, bit i = driver i */
    bool chain_closed;     /* ESTOP_SENSE: the MOTORS ON chain carries 12 V */
    bool bus_error;        /* the last expander read failed */
} limit_switch_snapshot_t;

esp_err_t limit_switch_init(const limit_switch_config_t *cfg);
/// Called from the control loop: current levels plus edges and latches since last time.
void limit_switch_collect(limit_switch_snapshot_t *out);

#ifdef __cplusplus
}
#endif
