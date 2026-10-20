// Two MCP23017 16-bit I/O expanders on the I2C bus (rev 0.4 board, Oct 20 2026):
//   U7, 0x20, the DIR chip:   GPA0-6 DIR1-7 (out), GPA7 driven low;
//                             GPB0 ESTOP_SENSE (in: the MOTORS ON chain's 12 V through the
//                             30k/10k divider, high = chain closed), GPB1-6 spare, GPB7 low.
//   U11, 0x21, the sense chip: GPA0-6 FLT1-7 (in, pull-up, active low), GPA7 driven low;
//                             GPB0-6 SW1-7 (in; the board's 10 k pull-ups, a switch pulls
//                             to ground), GPB7 driven low.
// GPA7 and GPB7 are never inputs (Microchip datasheet rev D: an input that changes during
// a read corrupts SDA). Both chips' INTA sit on one open-drain line with both ports
// mirrored onto it; the limit_switch component owns that interrupt and clears both chips.
// The rev 0.3 map (one chip plus three bits on a second) is in git history.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EXPANDER_CHANNELS 7
#define EXPANDER_CHANNEL_MASK 0x7Fu    /* GPA0-6 or GPB0-6 */
#define EXPANDER_DIR_ESTOP_SENSE 0x01u /* GPB0 of the DIR chip */

typedef enum {
    EXPANDER_ROLE_DIR,   /* port A outputs (DIR), GPB0 input (ESTOP_SENSE) */
    EXPANDER_ROLE_SENSE, /* port A inputs (FLT, pull-ups), port B inputs (SW) */
} expander_role_t;

typedef struct {
    i2c_master_dev_handle_t dev;
    expander_role_t role;
    uint8_t olat_a; /* shadow of the output latch */
} expander_t;

/// Direction bit of driver i (0-based) on the DIR chip's port A.
static inline uint8_t expander_dir_bit(int i) { return (uint8_t)(1u << i); }
/// FLT of driver i from the sense chip's port A (active low): true = faulted.
static inline bool expander_fault(int i, uint8_t sense_a) { return ((sense_a >> i) & 1u) == 0; }
/// Level of limit switch i from the sense chip's port B.
static inline bool expander_switch_level(int i, uint8_t sense_b) { return ((sense_b >> i) & 1u) != 0; }

/// Configure directions, pull-ups and interrupt-on-change for the role; outputs start low.
esp_err_t expander_init(expander_t *x, i2c_master_bus_handle_t bus, uint8_t address, uint32_t speed_hz, expander_role_t role);
/// Set port A's outputs (the DIR chip: bits 0-6 = DIR1-7) from one 8-bit value.
esp_err_t expander_write_port_a(expander_t *x, uint8_t value);
/// Read both ports: *a = GPIOA, *b = GPIOB.
esp_err_t expander_read(expander_t *x, uint8_t *a, uint8_t *b);
/// Read the interrupt flag and capture registers (clears this chip's interrupt).
esp_err_t expander_read_interrupt(expander_t *x, uint8_t *intf_a, uint8_t *intf_b, uint8_t *cap_a, uint8_t *cap_b);

#ifdef __cplusplus
}
#endif
