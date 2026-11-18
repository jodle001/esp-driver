// The status pixels' table (design doc 4.6) checked against the core's state machine:
// every state has one SYS pattern, each pixel shows what the table says, the brightness
// cap holds for every pattern at every moment, and the first frame is dark.
#include <math.h>

#include "minitest.h"
#include "scorbot_core/core.h"
#include "scorbot_core/pixels.h"

#define N 5

static const char *kNames[N] = {"base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"};

/* A core in a known place: booted, calibrated, every joint homed and still. */
static void make_core(sc_core_t *core, uint8_t joints, bool require_calibration)
{
    sc_core_config_t cfg;
    sc_core_config_default(&cfg);
    cfg.joint_count = joints;
    cfg.require_calibration = require_calibration;
    for (uint8_t i = 0; i < joints; ++i) {
        cfg.joint_names[i] = kNames[i < N ? i : N - 1];
        cfg.has_calibration[i] = true;
        cfg.calibration[i].max_velocity_rad_s = 1.0f;
    }
    sc_core_init(core, &cfg);
    core->state = scorbot_v1_SystemState_SYSTEM_STATE_READY;
    for (uint8_t i = 0; i < joints; ++i) {
        core->joint[i].homed = true;
    }
}

/* The glue's side on a healthy, idle board: dialect link with traffic, MOTORS ON latched. */
static sc_pix_input_t healthy(void)
{
    sc_pix_input_t in = {0};
    in.link_seen = true;
    in.link_idle_s = 0.1f;
    in.chain_valid = true;
    in.chain_closed = true;
    in.rail_on = true;
    return in;
}

#define EXPECT(p, c, r)                                                                                       \
    MT_CHECK((p).colour == (c) && (p).rate == (r), "%s %s, expected %s %s", sc_pix_colour_name((p).colour),   \
             sc_pix_rate_name((p).rate), sc_pix_colour_name(c), sc_pix_rate_name(r))

static sc_pix_pattern_t one(const sc_core_t *core, const sc_pix_input_t *in, int pixel)
{
    sc_pix_pattern_t p[SC_PIXEL_COUNT];
    sc_pixels_patterns(core, in, p);
    return p[pixel];
}

static void test_every_core_state_has_one_sys_pattern(void)
{
    sc_core_t core;
    make_core(&core, N, true);
    sc_pix_input_t in = healthy();
    const struct {
        scorbot_v1_SystemState state;
        sc_pix_colour_t colour;
        sc_pix_rate_t rate;
    } table[] = {
        {scorbot_v1_SystemState_SYSTEM_STATE_BOOT, SC_PIX_BLUE, SC_PIX_SLOW_BLINK},
        {scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED, SC_PIX_AMBER, SC_PIX_STEADY},
        {scorbot_v1_SystemState_SYSTEM_STATE_HOMING, SC_PIX_AMBER, SC_PIX_STEADY},
        {scorbot_v1_SystemState_SYSTEM_STATE_READY, SC_PIX_GREEN, SC_PIX_STEADY},
        {scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE, SC_PIX_GREEN, SC_PIX_BREATHING},
        {scorbot_v1_SystemState_SYSTEM_STATE_FAULT, SC_PIX_RED, SC_PIX_FAST_BLINK},
    };
    /* Every value of the enum is in the table above: a new state must be given a pattern. */
    MT_EQ(sizeof(table) / sizeof(table[0]), (size_t)scorbot_v1_SystemState_SYSTEM_STATE_FAULT + 1);
    for (size_t k = 0; k < sizeof(table) / sizeof(table[0]); ++k) {
        core.state = table[k].state;
        core.fault = table[k].state == scorbot_v1_SystemState_SYSTEM_STATE_FAULT ? scorbot_v1_FaultCode_FAULT_OVERCURRENT
                                                                                : scorbot_v1_FaultCode_FAULT_NONE;
        const sc_pix_pattern_t first = one(&core, &in, SC_PIXEL_SYS);
        EXPECT(first, table[k].colour, table[k].rate);
        /* ... and it is the same whatever the link or the chain is doing. */
        sc_pix_input_t other = in;
        other.link_seen = false;
        other.chain_closed = false;
        other.rail_on = false;
        const sc_pix_pattern_t second = one(&core, &other, SC_PIXEL_SYS);
        MT_TRUE(first.colour == second.colour && first.rate == second.rate);
    }
}

static void test_sys_faults_and_refusals(void)
{
    sc_core_t core;
    make_core(&core, N, true);
    sc_pix_input_t in = healthy();
    /* Every latched fault but the watchdog is red, fast. */
    core.state = scorbot_v1_SystemState_SYSTEM_STATE_FAULT;
    for (int f = scorbot_v1_FaultCode_FAULT_WATCHDOG; f <= scorbot_v1_FaultCode_FAULT_INTERNAL; ++f) {
        core.fault = (scorbot_v1_FaultCode)f;
        if (f == scorbot_v1_FaultCode_FAULT_WATCHDOG) {
            EXPECT(one(&core, &in, SC_PIXEL_SYS), SC_PIX_AMBER, SC_PIX_SLOW_BLINK);
        } else {
            EXPECT(one(&core, &in, SC_PIXEL_SYS), SC_PIX_RED, SC_PIX_FAST_BLINK);
        }
    }
    /* The controller refuses to run: red, steady, whatever the state. */
    in.self_test_failed = true;
    EXPECT(one(&core, &in, SC_PIXEL_SYS), SC_PIX_RED, SC_PIX_STEADY);
    core.state = scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED;
    EXPECT(one(&core, &in, SC_PIXEL_SYS), SC_PIX_RED, SC_PIX_STEADY);
    in.self_test_failed = false;
    core.joint[2].has_cal = false; /* calibration missing on the device */
    EXPECT(one(&core, &in, SC_PIXEL_SYS), SC_PIX_RED, SC_PIX_STEADY);
    core.cfg.require_calibration = false; /* the host build runs without one */
    EXPECT(one(&core, &in, SC_PIXEL_SYS), SC_PIX_AMBER, SC_PIX_STEADY);
    /* A soft limit engaged turns ready or active amber. */
    core.cfg.require_calibration = true;
    core.joint[2].has_cal = true;
    core.joint[1].at_soft_limit = true;
    core.state = scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE;
    EXPECT(one(&core, &in, SC_PIXEL_SYS), SC_PIX_AMBER, SC_PIX_STEADY);
    core.state = scorbot_v1_SystemState_SYSTEM_STATE_READY;
    EXPECT(one(&core, &in, SC_PIXEL_SYS), SC_PIX_AMBER, SC_PIX_STEADY);
}

static void test_link_pixel(void)
{
    sc_core_t core;
    make_core(&core, N, true);
    sc_pix_input_t in = healthy();
    in.link_seen = false;
    EXPECT(one(&core, &in, SC_PIXEL_LINK), SC_PIX_OFF, SC_PIX_STEADY);       /* nothing since boot */
    in.link_detecting = true;
    EXPECT(one(&core, &in, SC_PIXEL_LINK), SC_PIX_BLUE, SC_PIX_SLOW_BLINK);  /* auto mode, undecided */
    in.link_seen = true;
    EXPECT(one(&core, &in, SC_PIXEL_LINK), SC_PIX_BLUE, SC_PIX_SLOW_BLINK);
    in.link_detecting = false;
    in.link_idle_s = 0.5f;
    EXPECT(one(&core, &in, SC_PIXEL_LINK), SC_PIX_GREEN, SC_PIX_STEADY);     /* the dialect, traffic */
    in.link_protobuf = true;
    EXPECT(one(&core, &in, SC_PIXEL_LINK), SC_PIX_CYAN, SC_PIX_STEADY);      /* protobuf, traffic */
    in.link_idle_s = 1.5f;
    EXPECT(one(&core, &in, SC_PIXEL_LINK), SC_PIX_CYAN, SC_PIX_SLOW_BLINK);  /* up but idle */
    in.link_protobuf = false;
    EXPECT(one(&core, &in, SC_PIXEL_LINK), SC_PIX_GREEN, SC_PIX_SLOW_BLINK);
}

static void test_estop_pixel(void)
{
    sc_core_t core;
    make_core(&core, N, true);
    sc_pix_input_t in = healthy();
    EXPECT(one(&core, &in, SC_PIXEL_ESTOP), SC_PIX_GREEN, SC_PIX_STEADY);    /* MOTORS ON latched */
    in.rail_on = false;
    EXPECT(one(&core, &in, SC_PIXEL_ESTOP), SC_PIX_AMBER, SC_PIX_STEADY);    /* closed, not pressed yet */
    in.chain_closed = false;
    EXPECT(one(&core, &in, SC_PIXEL_ESTOP), SC_PIX_RED, SC_PIX_STEADY);      /* the chain is open */
    in.rail_on = true;
    EXPECT(one(&core, &in, SC_PIXEL_ESTOP), SC_PIX_RED, SC_PIX_FAST_BLINK);  /* open, yet the rail is live */
    in = healthy();
    in.chain_valid = false;
    EXPECT(one(&core, &in, SC_PIXEL_ESTOP), SC_PIX_RED, SC_PIX_FAST_BLINK);  /* cannot be read */
}

static void test_channel_pixels(void)
{
    sc_core_t core;
    make_core(&core, N, true);
    sc_pix_input_t in = healthy();
    sc_pix_pattern_t p[SC_PIXEL_COUNT];
    sc_pixels_patterns(&core, &in, p);
    for (int i = 0; i < N; ++i) {
        EXPECT(p[i], SC_PIX_GREEN, SC_PIX_STEADY); /* homed, holding */
    }
    EXPECT(p[5], SC_PIX_OFF, SC_PIX_STEADY);       /* five joints: no gripper, no rail */
    EXPECT(p[6], SC_PIX_OFF, SC_PIX_STEADY);
    core.joint[0].velocity = -0.5f;                /* moving, either way */
    core.joint[1].velocity = 0.005f;               /* encoder noise at rest is not moving */
    core.joint[2].homed = false;
    core.joint[3].homing = true;
    core.joint[3].homed = false;
    core.joint[4].fault = true;
    core.joint[4].homing = true;                   /* a fault wins over everything */
    sc_pixels_patterns(&core, &in, p);
    EXPECT(p[0], SC_PIX_GREEN, SC_PIX_BREATHING);
    EXPECT(p[1], SC_PIX_GREEN, SC_PIX_STEADY);
    EXPECT(p[2], SC_PIX_AMBER, SC_PIX_STEADY);
    EXPECT(p[3], SC_PIX_BLUE, SC_PIX_SLOW_BLINK);
    EXPECT(p[4], SC_PIX_RED, SC_PIX_FAST_BLINK);
    /* Seven channels: the gripper on 6, the rail on 7. */
    make_core(&core, 7, true);
    sc_pixels_patterns(&core, &in, p);
    EXPECT(p[5], SC_PIX_GREEN, SC_PIX_STEADY);
    EXPECT(p[6], SC_PIX_GREEN, SC_PIX_STEADY);
}

static void test_first_frame_is_dark_then_the_table_shows(void)
{
    sc_core_t core;
    make_core(&core, N, true);
    sc_pix_input_t in = healthy();
    sc_pixels_t px;
    sc_pixels_init(&px, 32);
    sc_rgb_t frame[SC_PIXEL_COUNT];
    memset(frame, 0xff, sizeof(frame));
    sc_pixels_render(&px, 0.02f, &core, &in, frame);
    for (int i = 0; i < SC_PIXEL_COUNT; ++i) {
        MT_EQ(frame[i].r + frame[i].g + frame[i].b, 0);
    }
    sc_pixels_render(&px, 0.02f, &core, &in, frame);
    MT_EQ(frame[SC_PIXEL_SYS].g, 32); /* ready: green, steady, at the cap */
    MT_EQ(frame[SC_PIXEL_SYS].r, 0);
    MT_EQ(frame[SC_PIXEL_SYS].b, 0);
    MT_EQ(frame[5].g, 0);             /* an unconfigured channel stays dark */
}

static void test_brightness_cap_holds_for_every_pattern(void)
{
    const uint8_t caps[] = {1, 32, 255};
    for (size_t c = 0; c < sizeof(caps); ++c) {
        int brightest = 0;
        for (int colour = SC_PIX_OFF; colour <= SC_PIX_CYAN; ++colour) {
            for (int rate = SC_PIX_STEADY; rate <= SC_PIX_BREATHING; ++rate) {
                const sc_pix_pattern_t p = {(sc_pix_colour_t)colour, (sc_pix_rate_t)rate};
                for (uint32_t t = 0; t < 4000; ++t) {
                    const sc_rgb_t v = sc_pixels_colour(p, t, caps[c]);
                    MT_CHECK(v.r <= caps[c] && v.g <= caps[c] && v.b <= caps[c], "over the cap %u at t=%u", caps[c], t);
                    brightest = v.r > brightest ? v.r : brightest;
                    brightest = v.g > brightest ? v.g : brightest;
                    brightest = v.b > brightest ? v.b : brightest;
                }
            }
        }
        MT_EQ(brightest, caps[c]); /* and the cap is reached: nothing is dimmer than asked */
    }
    sc_pixels_t px;
    sc_pixels_init(&px, 0);
    MT_EQ(px.max_level, 32); /* the design's default */
}

static void test_rates(void)
{
    const sc_pix_pattern_t slow = {SC_PIX_BLUE, SC_PIX_SLOW_BLINK};
    const sc_pix_pattern_t fast = {SC_PIX_RED, SC_PIX_FAST_BLINK};
    const sc_pix_pattern_t breath = {SC_PIX_GREEN, SC_PIX_BREATHING};
    /* Slow blink: 1 Hz, half on. Fast blink: 4 Hz, half on. */
    int slow_on = 0, fast_on = 0, fast_edges = 0, last = -1;
    for (uint32_t t = 0; t < 1000; ++t) {
        slow_on += sc_pixels_colour(slow, t, 32).b > 0;
        const int f = sc_pixels_colour(fast, t, 32).r > 0;
        fast_on += f;
        fast_edges += last == 0 && f == 1;
        last = f;
    }
    MT_EQ(slow_on, 500);
    MT_EQ(fast_on, 500);
    MT_EQ(fast_edges, 3); /* on at 0, 250, 500, 750: three rising edges inside the second */
    MT_EQ(sc_pixels_colour(slow, 0, 32).b, 32);
    MT_EQ(sc_pixels_colour(slow, 500, 32).b, 0);
    MT_EQ(sc_pixels_colour(slow, 1000, 32).b, 32);
    /* Breathing: a 2 s triangle that peaks at the cap and never goes dark. */
    int lowest = 255, highest = 0;
    for (uint32_t t = 0; t < 2000; ++t) {
        const int g = sc_pixels_colour(breath, t, 32).g;
        lowest = g < lowest ? g : lowest;
        highest = g > highest ? g : highest;
    }
    MT_EQ(highest, 32);
    MT_GT(lowest, 0);
    MT_EQ(sc_pixels_colour(breath, 1000, 32).g, 32);
    MT_EQ(sc_pixels_colour(breath, 0, 32).g, sc_pixels_colour(breath, 2000, 32).g);
    /* The time base wraps without a jump in any pattern. */
    sc_core_t core;
    make_core(&core, N, true);
    core.state = scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE;
    sc_pix_input_t in = healthy();
    sc_pixels_t px;
    sc_pixels_init(&px, 32);
    sc_rgb_t frame[SC_PIXEL_COUNT];
    sc_pixels_render(&px, 0.0f, &core, &in, frame); /* the dark first frame */
    px.elapsed_ms = 4.0e6f - 20.0f;
    sc_pixels_render(&px, 0.0f, &core, &in, frame);
    const int before = frame[SC_PIXEL_SYS].g;
    sc_pixels_render(&px, 0.02f, &core, &in, frame);
    MT_LT(px.elapsed_ms, 1.0f);
    MT_NEAR(frame[SC_PIXEL_SYS].g, before, 1.0);
}

int main(void)
{
    MT_RUN(test_every_core_state_has_one_sys_pattern);
    MT_RUN(test_sys_faults_and_refusals);
    MT_RUN(test_link_pixel);
    MT_RUN(test_estop_pixel);
    MT_RUN(test_channel_pixels);
    MT_RUN(test_first_frame_is_dark_then_the_table_shows);
    MT_RUN(test_brightness_cap_holds_for_every_pattern);
    MT_RUN(test_rates);
    return MT_SUMMARY();
}
