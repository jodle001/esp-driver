// LS7366R quadrature counters on SPI, one per channel (rev 0.4 board, Oct 20 2026): 32-bit
// 4x counting in silicon, every chip read once per control tick. Each chip's INDEX input
// is wired to its channel's limit-switch line, so the count at the switch edge can be
// latched in hardware (MDR0 index mode "load OTR") once the index polarity has been seen
// on the bench; until then the limit_switch task latches by reading CNTR right after the
// expander interrupt, as the rev 0.3 firmware did with the pulse counters.
//
// Register and opcode map (LSI/CSI LS7366R datasheet; ASSUMED from the common usage of
// the part, VERIFY on the first board): command byte = op << 6 | register << 3, op 0 CLR,
// 1 RD, 2 WR, 3 LOAD; register 1 MDR0, 2 MDR1, 3 DTR, 4 CNTR, 5 OTR, 6 STR. MDR0 bits
// 1:0 quadrature mode (3 = x4), 3:2 index mode (0 off, 1 load CNTR, 2 reset CNTR,
// 3 load OTR), 4 synchronous index, 5 filter clock divided by 2, 7:6 count mode (0 free
// running). MDR1 bits 1:0 counter size (0 = 4 bytes), 2 counting disabled. SPI mode 0,
// MSB first, MISO high impedance while SS is high, so the chips share MOSI/MISO/SCLK and
// have a chip select each. The chips need a filter clock on fCKi: one LEDC output feeds
// all of them (4 MHz, far above the 8 kHz the fastest encoder reaches).
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LS7366R_MAX 8

typedef struct {
    int sclk_gpio, mosi_gpio, miso_gpio;
    int cs_gpio[LS7366R_MAX];
    int count;            /* chips, 1..8 */
    int clk_gpio;         /* fCKi to every chip; -1 = the board has its own oscillator */
    uint32_t clk_hz;      /* 4000000 */
    uint32_t spi_hz;      /* 10000000 */
    int ledc_channel;     /* LEDC channel for the filter clock (the motors use 0..count-1) */
} ls7366r_config_t;

/// Bus, chip selects, filter clock; then every chip is configured for x4 free-running
/// counting and read back (a chip that does not answer fails the init: self-test).
esp_err_t ls7366r_init(const ls7366r_config_t *cfg);
/// CNTR of every chip, raw signed counts. Safe from more than one task.
esp_err_t ls7366r_read_all(int32_t *counts);
esp_err_t ls7366r_read(int chip, int32_t *count);
esp_err_t ls7366r_clear(int chip);

#ifdef __cplusplus
}
#endif
