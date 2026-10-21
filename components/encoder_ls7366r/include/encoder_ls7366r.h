// LS7366R quadrature counters on SPI, one per channel (rev 0.4 board, Oct 20 2026): 32-bit
// 4x counting in silicon, every chip read once per control tick. Each chip's INDEX/ input
// is wired to its channel's limit-switch line (active low, so a pressed switch is the
// index event with the board's active-low switches), so the count at the switch edge can
// be latched in hardware: MDR0 index mode "load OTR" copies CNTR into OTR on the index.
// That mode is behind `index_latch` (Kconfig SC_CNT_INDEX_LATCH, off by default) until the
// bench shows the OTR holds the count at the switch edge rather than following CNTR while
// the switch is held; with it off, the limit_switch task latches by reading CNTR right
// after the expander interrupt, as the rev 0.3 firmware did with the pulse counters (a few
// counts late at 8 kHz). Caveat from the datasheet either way: "RD CNTR" transfers CNTR
// into OTR before shifting it out, so every periodic read overwrites the latch; the latch
// read must come before the next tick's CNTR read, which the interrupt path does.
//
// Register and opcode map, from the LSI/CSI LS7366R datasheet (June 2014; checked Oct 21
// 2026): command byte = op << 6 | register << 3, op 0 CLR, 1 RD, 2 WR, 3 LOAD; register
// 1 MDR0, 2 MDR1, 3 DTR, 4 CNTR, 5 OTR, 6 STR. MDR0 bits 1:0 quadrature mode (3 = x4),
// 3:2 count mode (0 free running), 5:4 index mode (0 off, 1 load CNTR, 2 reset CNTR,
// 3 load OTR), 6 synchronous index, 7 filter clock divided by 2. MDR1 bits 1:0 counter
// size (0 = 4 bytes), 2 counting disabled, 7:4 flag enables. STR bit 4 is the index latch.
// SPI mode 0, MSB first, MISO high impedance while SS/ is high, so the chips share
// MOSI/MISO/SCLK and have a chip select each. At 3.3 V the SCK high and low times are
// 120 ns minimum, so the bus runs at 4 MHz (10 MHz is only valid at 5 V). The chips need
// a filter clock on fCKi (20 MHz maximum at 3.3 V; the quadrature rate must stay under a
// quarter of it): one LEDC output feeds all of them at 4 MHz, far above the 8 kHz the
// fastest encoder reaches, and fCKO (the crystal output) is left open.
// Pins (DIP-14): 1 fCKO, 2 fCKi, 3 VSS, 4 SS/, 5 SCK, 6 MISO, 7 MOSI, 8 LFLAG/, 9 DFLAG/,
// 10 INDEX/, 11 B, 12 A, 13 CNT_EN, 14 VDD.
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
    uint32_t spi_hz;      /* 4000000: the LS7366R's limit at 3.3 V */
    int ledc_channel;     /* LEDC channel for the filter clock (the motors use 0..count-1) */
    bool index_latch;     /* index mode "load OTR": the switch edge latches CNTR into OTR in silicon */
} ls7366r_config_t;

/// Bus, chip selects, filter clock; then every chip is configured for x4 free-running
/// counting and read back (a chip that does not answer fails the init: self-test).
esp_err_t ls7366r_init(const ls7366r_config_t *cfg);
/// CNTR of every chip, raw signed counts. Safe from more than one task. Note that this
/// read also copies CNTR into OTR (the chip's RD CNTR does that), overwriting a latch.
esp_err_t ls7366r_read_all(int32_t *counts);
/// The count latched at the last index (switch) edge, every chip: OTR when index_latch is
/// on, else CNTR (the software latch: the caller reads as soon as the switch interrupt lands).
esp_err_t ls7366r_read_latch_all(int32_t *counts);
esp_err_t ls7366r_read(int chip, int32_t *count);
esp_err_t ls7366r_clear(int chip);

#ifdef __cplusplus
}
#endif
