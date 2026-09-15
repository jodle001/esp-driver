// Limit switches (SW1-4 on GPB3-6, SW5 on GPA6) and driver fault lines through the
// MCP23017, with the encoder latch.
//
// INTA fires on any change of a switch or FLT input. The GPIO interrupt handler reads
// all five PCNT counts at once (the latch), then a task reads INTF/INTCAP over I2C to
// learn which input moved. The control loop collects the result once per tick:
// per-joint switch level, whether an edge happened, and the count latched at it.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "encoder.h"
#include "esp_err.h"
#include "expander.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LIMIT_SWITCH_COUNT 5

typedef struct {
    int int_gpio;                  /* MCP23017 INTA */
    bool active_low;               /* switch pressed reads 0 */
    expander_t *expander;
    encoder_handle_t *encoders[LIMIT_SWITCH_COUNT];
    uint32_t debounce_us;          /* 2000 */
} limit_switch_config_t;

typedef struct {
    bool pressed;
    bool edge;             /* changed since the last collect AND latched_count is valid */
    int32_t latched_count; /* raw PCNT count captured in the INTA interrupt */
} limit_switch_joint_t;

typedef struct {
    limit_switch_joint_t joint[LIMIT_SWITCH_COUNT];
    uint8_t fault_mask;    /* driver FLT lines, bit i = driver i */
    bool bus_error;        /* the last expander read failed */
} limit_switch_snapshot_t;

esp_err_t limit_switch_init(const limit_switch_config_t *cfg);
/// Called from the control loop: current levels plus edges and latches since last time.
void limit_switch_collect(limit_switch_snapshot_t *out);

#ifdef __cplusplus
}
#endif
