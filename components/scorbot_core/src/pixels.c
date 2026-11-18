// The status pixels' table (design doc 4.6) and its rendering. See pixels.h.
#include "scorbot_core/pixels.h"

#include <math.h>
#include <string.h>

#define LINK_IDLE_S 1.0f          /* "traffic arrived within the last second" */
#define MOVING_FRACTION 0.02f     /* of the joint's maximum speed */
#define MOVING_FLOOR_RAD_S 0.01f  /* without a calibration */
#define BREATHING_FLOOR 0.15f     /* breathing never goes fully dark: it is not a blink */

static sc_pix_pattern_t pat(sc_pix_colour_t colour, sc_pix_rate_t rate)
{
    const sc_pix_pattern_t p = {colour, rate};
    return p;
}

/* SYS: the controller's own state. */
static sc_pix_pattern_t sys_pattern(const sc_core_t *core, const sc_pix_input_t *in)
{
    if (in->self_test_failed) {
        return pat(SC_PIX_RED, SC_PIX_STEADY); /* refuses to run */
    }
    switch (core->state) {
    case scorbot_v1_SystemState_SYSTEM_STATE_BOOT:
        return pat(SC_PIX_BLUE, SC_PIX_SLOW_BLINK);
    case scorbot_v1_SystemState_SYSTEM_STATE_FAULT:
        /* The watchdog stops the motors but is not a hardware fault: its own pattern. */
        return core->fault == scorbot_v1_FaultCode_FAULT_WATCHDOG ? pat(SC_PIX_AMBER, SC_PIX_SLOW_BLINK)
                                                                : pat(SC_PIX_RED, SC_PIX_FAST_BLINK);
    default: break;
    }
    if (core->cfg.require_calibration) {
        for (uint8_t i = 0; i < core->joint_count; ++i) {
            if (!core->joint[i].has_cal) {
                return pat(SC_PIX_RED, SC_PIX_STEADY); /* Enable is refused until it is set */
            }
        }
    }
    bool soft_limit = false;
    for (uint8_t i = 0; i < core->joint_count; ++i) {
        soft_limit = soft_limit || core->joint[i].at_soft_limit;
    }
    switch (core->state) {
    case scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE:
        return soft_limit ? pat(SC_PIX_AMBER, SC_PIX_STEADY) : pat(SC_PIX_GREEN, SC_PIX_BREATHING);
    case scorbot_v1_SystemState_SYSTEM_STATE_READY:
        return soft_limit ? pat(SC_PIX_AMBER, SC_PIX_STEADY) : pat(SC_PIX_GREEN, SC_PIX_STEADY);
    default: /* UNHOMED, HOMING: a joint is unhomed */
        return pat(SC_PIX_AMBER, SC_PIX_STEADY);
    }
}

/* LINK: the Pi link. */
static sc_pix_pattern_t link_pattern(const sc_pix_input_t *in)
{
    if (in->link_detecting) {
        return pat(SC_PIX_BLUE, SC_PIX_SLOW_BLINK);
    }
    if (!in->link_seen) {
        return pat(SC_PIX_OFF, SC_PIX_STEADY);
    }
    const sc_pix_colour_t colour = in->link_protobuf ? SC_PIX_CYAN : SC_PIX_GREEN;
    return pat(colour, in->link_idle_s > LINK_IDLE_S ? SC_PIX_SLOW_BLINK : SC_PIX_STEADY);
}

/* E-STOP: the chain as the firmware reads it. */
static sc_pix_pattern_t estop_pattern(const sc_pix_input_t *in)
{
    if (!in->chain_valid) {
        return pat(SC_PIX_RED, SC_PIX_FAST_BLINK); /* cannot read it: not as expected */
    }
    if (in->chain_closed) {
        return in->rail_on ? pat(SC_PIX_GREEN, SC_PIX_STEADY) : pat(SC_PIX_AMBER, SC_PIX_STEADY);
    }
    /* Open. A live rail behind an open chain is a relay or sense disagreement. */
    return in->rail_on ? pat(SC_PIX_RED, SC_PIX_FAST_BLINK) : pat(SC_PIX_RED, SC_PIX_STEADY);
}

/* One motor channel. */
static sc_pix_pattern_t channel_pattern(const sc_core_t *core, uint8_t i)
{
    if (i >= core->joint_count) {
        return pat(SC_PIX_OFF, SC_PIX_STEADY); /* not configured, e.g. no rail on this robot */
    }
    const sc_joint_t *j = &core->joint[i];
    if (j->fault) {
        return pat(SC_PIX_RED, SC_PIX_FAST_BLINK);
    }
    if (j->homing) {
        return pat(SC_PIX_BLUE, SC_PIX_SLOW_BLINK);
    }
    if (!j->homed) {
        return pat(SC_PIX_AMBER, SC_PIX_STEADY);
    }
    float threshold = MOVING_FLOOR_RAD_S;
    if (j->has_cal && j->cal.max_velocity_rad_s * MOVING_FRACTION > threshold) {
        threshold = j->cal.max_velocity_rad_s * MOVING_FRACTION;
    }
    return fabsf(j->velocity) > threshold ? pat(SC_PIX_GREEN, SC_PIX_BREATHING) : pat(SC_PIX_GREEN, SC_PIX_STEADY);
}

void sc_pixels_init(sc_pixels_t *p, uint8_t max_level)
{
    memset(p, 0, sizeof(*p));
    p->max_level = max_level ? max_level : 32;
}

void sc_pixels_patterns(const sc_core_t *core, const sc_pix_input_t *in, sc_pix_pattern_t out[SC_PIXEL_COUNT])
{
    for (uint8_t i = 0; i < SC_PIXEL_CHANNELS; ++i) {
        out[i] = channel_pattern(core, i);
    }
    out[SC_PIXEL_SYS] = sys_pattern(core, in);
    out[SC_PIXEL_LINK] = link_pattern(in);
    out[SC_PIXEL_ESTOP] = estop_pattern(in);
}

sc_rgb_t sc_pixels_colour(sc_pix_pattern_t pattern, uint32_t t_ms, uint8_t max_level)
{
    static const uint8_t kBase[][3] = {
        [SC_PIX_OFF] = {0, 0, 0},     [SC_PIX_RED] = {255, 0, 0},    [SC_PIX_GREEN] = {0, 255, 0},
        [SC_PIX_BLUE] = {0, 0, 255},  [SC_PIX_AMBER] = {255, 110, 0}, [SC_PIX_CYAN] = {0, 255, 255},
    };
    float level = 1.0f;
    switch (pattern.rate) {
    case SC_PIX_SLOW_BLINK: level = (t_ms % 1000u) < 500u ? 1.0f : 0.0f; break;
    case SC_PIX_FAST_BLINK: level = (t_ms % 250u) < 125u ? 1.0f : 0.0f; break;
    case SC_PIX_BREATHING: {
        const float phase = (float)(t_ms % 2000u) / 2000.0f;
        const float triangle = phase < 0.5f ? phase * 2.0f : (1.0f - phase) * 2.0f;
        level = BREATHING_FLOOR + (1.0f - BREATHING_FLOOR) * triangle;
        break;
    }
    default: break;
    }
    const uint8_t *base = kBase[pattern.colour <= SC_PIX_CYAN ? pattern.colour : SC_PIX_OFF];
    const float scale = level * (float)max_level / 255.0f;
    sc_rgb_t c;
    c.r = (uint8_t)((float)base[0] * scale + 0.5f);
    c.g = (uint8_t)((float)base[1] * scale + 0.5f);
    c.b = (uint8_t)((float)base[2] * scale + 0.5f);
    return c;
}

void sc_pixels_render(sc_pixels_t *p, float dt_s, const sc_core_t *core, const sc_pix_input_t *in,
                      sc_rgb_t out[SC_PIXEL_COUNT])
{
    if (!p->first_done) {
        p->first_done = true;
        memset(out, 0, sizeof(sc_rgb_t) * SC_PIXEL_COUNT);
        return;
    }
    if (dt_s > 0.0f) {
        p->elapsed_ms += dt_s * 1000.0f;
        if (p->elapsed_ms >= 4.0e6f) {
            p->elapsed_ms -= 4.0e6f; /* a whole number of every period: no jump */
        }
    }
    sc_pix_pattern_t pattern[SC_PIXEL_COUNT];
    sc_pixels_patterns(core, in, pattern);
    const uint32_t t_ms = (uint32_t)p->elapsed_ms;
    for (uint8_t i = 0; i < SC_PIXEL_COUNT; ++i) {
        out[i] = sc_pixels_colour(pattern[i], t_ms, p->max_level);
    }
}

const char *sc_pix_colour_name(sc_pix_colour_t c)
{
    switch (c) {
    case SC_PIX_RED: return "red";
    case SC_PIX_GREEN: return "green";
    case SC_PIX_BLUE: return "blue";
    case SC_PIX_AMBER: return "amber";
    case SC_PIX_CYAN: return "cyan";
    default: return "off";
    }
}

const char *sc_pix_rate_name(sc_pix_rate_t r)
{
    switch (r) {
    case SC_PIX_SLOW_BLINK: return "slow blink";
    case SC_PIX_FAST_BLINK: return "fast blink";
    case SC_PIX_BREATHING: return "breathing";
    default: return "steady";
    }
}
