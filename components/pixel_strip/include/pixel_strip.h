// The status pixel chain on one GPIO: ten 5 mm addressable RGB pixels behind the level
// shifter U22, driven by the RMT peripheral.
//
// What each pixel shows is decided in scorbot_core/pixels.h; this component only puts a
// frame on the wire. The control task hands frames over without waiting: a small task
// does the transmission, and a frame that has not gone out yet is replaced by the next.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PIXEL_STRIP_MAX 16

typedef struct {
    int gpio;            /* data out, to the level shifter */
    int count;           /* pixels on the chain, 1..PIXEL_STRIP_MAX */
    bool grb;            /* byte order on the wire: G, R, B (WS2812B) instead of R, G, B */
    uint16_t t0h_ns;     /* a 0 bit: high time */
    uint16_t t0l_ns;     /*          low time */
    uint16_t t1h_ns;     /* a 1 bit: high time */
    uint16_t t1l_ns;     /*          low time */
} pixel_strip_config_t;

/// Set up the RMT channel and the task, and send one dark frame.
esp_err_t pixel_strip_init(const pixel_strip_config_t *cfg);

/// Hand over the next frame: count x 3 bytes, R, G, B per pixel, already at the level that
/// goes on the wire. Never blocks. Before pixel_strip_init succeeded it does nothing.
void pixel_strip_show(const uint8_t *rgb);

#ifdef __cplusplus
}
#endif
