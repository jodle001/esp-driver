// The ten status pixels: what each one shows, as portable C with no ESP-IDF calls.
//
// The board has ten addressable RGB pixels, silk "1 2 3 4 5 6 7 SYS LINK E-STOP" (design
// doc 4.6). This file turns the core's state into a pattern per pixel (a colour and a
// rate) and renders the patterns into the bytes for the wire; components/pixel_strip
// puts them on the wire. Kept here so the table is tested on the host.
//
// Rules of the design: every colour is scaled by max_level before it leaves this file
// (the pixel rail's current is part of the encoder 5 V sense margin), the first frame
// after boot is dark, and the rates come from the control tick: slow blink 1 Hz, fast
// blink 4 Hz, breathing a 2 s triangle.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "scorbot_core/core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SC_PIXEL_COUNT 10
#define SC_PIXEL_CHANNELS 7 /* pixels 0..6: one per motor channel, joint i on pixel i */
#define SC_PIXEL_SYS 7
#define SC_PIXEL_LINK 8
#define SC_PIXEL_ESTOP 9

typedef enum {
    SC_PIX_OFF = 0,
    SC_PIX_RED,
    SC_PIX_GREEN,
    SC_PIX_BLUE,
    SC_PIX_AMBER,
    SC_PIX_CYAN,
} sc_pix_colour_t;

typedef enum {
    SC_PIX_STEADY = 0,
    SC_PIX_SLOW_BLINK, /* 1 Hz */
    SC_PIX_FAST_BLINK, /* 4 Hz */
    SC_PIX_BREATHING,  /* 2 s triangle, never fully dark */
} sc_pix_rate_t;

typedef struct {
    sc_pix_colour_t colour;
    sc_pix_rate_t rate;
} sc_pix_pattern_t;

/// What the glue knows that the core does not.
typedef struct {
    bool self_test_failed;    /* the hardware self-test failed at boot */
    /* the link to the Pi */
    bool link_detecting;      /* auto mode, protocol not decided yet */
    bool link_protobuf;       /* the protobuf link is in use (false: the Eshed dialect) */
    bool link_seen;           /* at least one byte arrived since boot */
    float link_idle_s;        /* time since the last byte */
    /* the e-stop chain as the firmware reads it */
    bool chain_valid;         /* the chain sense and the rail voltage were read */
    bool chain_closed;        /* the mushroom button and the external loop are closed */
    bool rail_on;             /* the motor rail is live (MOTORS ON latched) */
} sc_pix_input_t;

typedef struct {
    uint8_t r, g, b;
} sc_rgb_t;

typedef struct {
    uint8_t max_level;   /* the brightest value put on the wire, 1..255 */
    float elapsed_ms;    /* pattern time base */
    bool first_done;     /* the dark first frame has been rendered */
} sc_pixels_t;

/// max_level 0 is taken as 32, the design's default.
void sc_pixels_init(sc_pixels_t *p, uint8_t max_level);

/// The pattern of every pixel for the state now (no time involved): the table of 4.6.
void sc_pixels_patterns(const sc_core_t *core, const sc_pix_input_t *in, sc_pix_pattern_t out[SC_PIXEL_COUNT]);

/// One pattern at a moment of the time base, scaled by max_level.
sc_rgb_t sc_pixels_colour(sc_pix_pattern_t pattern, uint32_t t_ms, uint8_t max_level);

/// Advance the time base by dt_s and render the frame for the wire. The first frame after
/// sc_pixels_init is dark whatever the state.
void sc_pixels_render(sc_pixels_t *p, float dt_s, const sc_core_t *core, const sc_pix_input_t *in,
                      sc_rgb_t out[SC_PIXEL_COUNT]);

const char *sc_pix_colour_name(sc_pix_colour_t c);
const char *sc_pix_rate_name(sc_pix_rate_t r);

#ifdef __cplusplus
}
#endif
