// MCP23017 16-bit I/O expander on the I2C bus (rev 0.1 board, address 0x20):
//   GPA0-4  DIR1-5 (out)     GPA5 SLP, one line to all five drivers (out)
//   GPA6-7  FLT1-2 (in, pull-up, active low)    GPB0-2 FLT3-5 (in)
//   GPB3-7  SW1-5 (in; the board has 10 k pull-ups, the switch pulls to ground)
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

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t olat_a; /* shadow of the output latch */
} expander_t;

/// Configure directions, pull-ups and interrupt-on-change; outputs start low (drivers asleep).
esp_err_t expander_init(expander_t *x, i2c_master_bus_handle_t bus, uint8_t address, uint32_t speed_hz);
/// Set DIR (bits 0..4) and SLP (bit 5) together from one 8-bit value of port A.
esp_err_t expander_write_port_a(expander_t *x, uint8_t value);
/// Read both ports: *a = GPIOA, *b = GPIOB.
esp_err_t expander_read(expander_t *x, uint8_t *a, uint8_t *b);
/// Read the interrupt flag and capture registers (clears the interrupt).
esp_err_t expander_read_interrupt(expander_t *x, uint8_t *intf_a, uint8_t *intf_b, uint8_t *cap_a, uint8_t *cap_b);

#ifdef __cplusplus
}
#endif
