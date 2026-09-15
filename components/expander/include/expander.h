// MCP23017 16-bit I/O expander on the I2C bus (rev 0.2 board, address 0x20):
//   GPA0-3  DIR1-4 (out)     GPA7 DIR5 (out)
//   GPA4-5  FLT1-2 (in, pull-up, active low)    GPB0-2 FLT3-5 (in)
//   GPA6    SW5 (in)         GPB3-6 SW1-4 (in; the board has 10 k pull-ups, the switch pulls to ground)
//   GPB7    unused, driven low
// GPA7 and GPB7 are output-only on this part (Microchip datasheet rev D: an input that
// changes during a read corrupts SDA), which is why DIR5 sits on GPA7 and GPB7 is not used.
// The shared driver SLP line is an ESP32 GPIO, not on the expander (see motor.h).
// INTA is configured to fire on any change of the FLT and SW inputs, both ports
// mirrored onto it; the limit_switch component owns that interrupt.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Port A bits (outputs and inputs) */
#define EXPANDER_A_FLT1 0x10u
#define EXPANDER_A_FLT2 0x20u
#define EXPANDER_A_SW5 0x40u
#define EXPANDER_A_DIR5 0x80u
#define EXPANDER_A_OUTPUTS 0x8Fu /* DIR1-4, DIR5 */
/* Port B bits */
#define EXPANDER_B_FLT_MASK 0x07u /* FLT3-5 on GPB0-2 */
#define EXPANDER_B_SW_SHIFT 3     /* SW1-4 on GPB3-6 */

/// Port A bit that carries DIR for driver i (0-based): GPA0-3 for drivers 1-4, GPA7 for driver 5.
static inline uint8_t expander_dir_bit(int i) { return i < 4 ? (uint8_t)(1u << i) : (uint8_t)EXPANDER_A_DIR5; }
/// Level of limit switch i (0-based) from a port A / port B reading.
static inline bool expander_switch_level(int i, uint8_t gpio_a, uint8_t gpio_b)
{
    return i < 4 ? ((gpio_b >> (EXPANDER_B_SW_SHIFT + i)) & 1u) != 0 : (gpio_a & EXPANDER_A_SW5) != 0;
}

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t olat_a; /* shadow of the output latch */
} expander_t;

/// Configure directions, pull-ups and interrupt-on-change; outputs start low (drivers asleep).
esp_err_t expander_init(expander_t *x, i2c_master_bus_handle_t bus, uint8_t address, uint32_t speed_hz);
/// Set the DIR lines (bits 0..3 and 7) from one 8-bit value of port A.
esp_err_t expander_write_port_a(expander_t *x, uint8_t value);
/// Read both ports: *a = GPIOA, *b = GPIOB.
esp_err_t expander_read(expander_t *x, uint8_t *a, uint8_t *b);
/// Read the interrupt flag and capture registers (clears the interrupt).
esp_err_t expander_read_interrupt(expander_t *x, uint8_t *intf_a, uint8_t *intf_b, uint8_t *cap_a, uint8_t *cap_b);

#ifdef __cplusplus
}
#endif
