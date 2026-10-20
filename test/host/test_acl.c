// The Eshed-dialect front-end (scorbot_core/acl) driven the way the operator drives the
// real box: bytes in, text out, against the core and the host plant. The expectations
// come from reference_only/operator/src/ER_V (what it sends, what it parses, how it
// waits for the prompt); design doc section 7 is the written form.
#include <stdlib.h>

#include "minitest.h"
#include "plant.h"
#include "scorbot_core/acl.h"
#include "scorbot_core/core.h"
#include "scorbot_core/link_mode.h"

#define OUT_CAP 16384

typedef struct {
    sc_core_t core;
    plant_t plant;
    sc_core_config_t cfg;
    sc_acl_t acl;
    scorbot_v1_JointCalibration cal[SC_CORE_MAX_JOINTS];
    int n;
    char out[OUT_CAP];
    size_t out_len;
    bool saw_flow_control_byte;
} h_t;

static const char *kNames[8] = {"base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint",
                                "wrist_roll_joint", "slide_joint", "j7", "j8"};

static void h_init(h_t *h, int joints)
{
    memset(h, 0, sizeof(*h));
    h->n = joints;
    for (int i = 0; i < joints; ++i) {
        h->cal[i] = plant_default_calibration((uint32_t)i);
    }
    plant_init(&h->plant, (uint8_t)joints, h->cal, NULL, NULL);
    sc_core_config_default(&h->cfg);
    h->cfg.joint_count = (uint8_t)joints;
    for (int i = 0; i < joints; ++i) {
        h->cfg.joint_names[i] = kNames[i];
        h->cfg.has_calibration[i] = true;
        h->cfg.calibration[i] = h->cal[i];
    }
    h->cfg.firmware_version = "host-acl";
    sc_core_init(&h->core, &h->cfg);
    sc_acl_config_t ac;
    sc_acl_config_default(&ac);
    sc_acl_init(&h->acl, &h->core, &ac);
}

static void drain(h_t *h)
{
    uint8_t buf[256];
    size_t n;
    while ((n = sc_acl_read(&h->acl, buf, sizeof(buf))) > 0) {
        for (size_t k = 0; k < n; ++k) {
            if (buf[k] == 0x11 || buf[k] == 0x13) {
                h->saw_flow_control_byte = true;
            }
            if (h->out_len < OUT_CAP - 1) {
                h->out[h->out_len++] = (char)buf[k];
            }
        }
        h->out[h->out_len] = '\0';
    }
}

static void tick(h_t *h)
{
    sc_input_t in;
    sc_output_t out;
    plant_read(&h->plant, &in);
    sc_core_tick(&h->core, 0.001f, &in, &out);
    plant_step(&h->plant, &out, 0.001);
    sc_acl_tick(&h->acl, 0.001f);
    drain(h);
}

static void run(h_t *h, double seconds)
{
    const int steps = (int)(seconds / 0.001 + 0.5);
    for (int i = 0; i < steps; ++i) {
        tick(h);
    }
}

static void feed(h_t *h, const char *s)
{
    sc_acl_feed(&h->acl, (const uint8_t *)s, strlen(s));
    drain(h);
}

static size_t mark(const h_t *h) { return h->out_len; }

static int prompts_since(const h_t *h, size_t from)
{
    int n = 0;
    for (size_t k = from; k < h->out_len; ++k) {
        n += h->out[k] == '>';
    }
    return n;
}

static bool seen_since(const h_t *h, size_t from, const char *needle) { return strstr(h->out + from, needle) != NULL; }

/// The operator's parse of a LISTPV reply: " %*d:%ld" five times from the start of a line.
static bool last_position(const h_t *h, size_t from, long v[5])
{
    const char *best = NULL;
    for (const char *p = h->out + from; (p = strstr(p, "1:")) != NULL; ++p) {
        best = p;
    }
    if (!best) {
        return false;
    }
    while (best > h->out && best[-1] != '\n' && best[-1] != '\r') {
        --best;
    }
    return sscanf(best, " %*d:%ld %*d:%ld %*d:%ld %*d:%ld %*d:%ld", &v[0], &v[1], &v[2], &v[3], &v[4]) == 5;
}

/// What the operator does at start-up: flush, DEFP, HOME, HERE, each after the prompt.
static void start_like_the_operator(h_t *h)
{
    run(h, 0.1); /* boot */
    feed(h, "\r");
    feed(h, "DEFP DELTA\r");
    feed(h, "HOME\r");
    run(h, 15.0);
    feed(h, "HERE DELTA\r");
    run(h, 0.01);
}

/* ---- tests ------------------------------------------------------------------------------ */

static void test_bare_cr_gets_a_prompt(void)
{
    h_t h;
    h_init(&h, 5);
    run(&h, 0.1);
    const size_t m = mark(&h);
    feed(&h, "\r");
    MT_EQ(prompts_since(&h, m), 1);
    feed(&h, "\r\n");
    MT_EQ(prompts_since(&h, m), 2); /* the LF of a CR LF pair is not a second command */
}

static void test_home_prompts_after_the_cycle_and_listpv_parses(void)
{
    h_t h;
    h_init(&h, 5);
    run(&h, 0.1);
    feed(&h, "\r");
    size_t m = mark(&h);
    feed(&h, "DEFP DELTA\r");
    MT_EQ(prompts_since(&h, m), 1);
    m = mark(&h);
    feed(&h, "HOME\r");
    run(&h, 0.5);
    MT_EQ(h.core.state, scorbot_v1_SystemState_SYSTEM_STATE_HOMING);
    MT_EQ(prompts_since(&h, m), 0); /* the operator waits: HERE must capture the home pose */
    run(&h, 15.0);
    MT_EQ(h.core.state, scorbot_v1_SystemState_SYSTEM_STATE_READY);
    MT_EQ(prompts_since(&h, m), 1);
    for (int i = 0; i < 5; ++i) {
        MT_TRUE(h.core.joint[i].homed);
    }
    m = mark(&h);
    feed(&h, "LISTPV POSITION\r");
    MT_EQ(prompts_since(&h, m), 1);
    long v[5];
    MT_TRUE(last_position(&h, m, v));
    for (int i = 0; i < 5; ++i) {
        MT_EQ(v[i], sc_acl_virtual_counts(&h.acl, (uint8_t)i));
        MT_LT(labs(v[i]), 60); /* the home pose, within the switch width in ER-V counts */
    }
}

static void test_shift_move_reaches_the_target(void)
{
    h_t h;
    h_init(&h, 5);
    start_like_the_operator(&h);
    /* HERE captured the home pose, which sits within the switch width of zero, not at it */
    const long before = sc_acl_virtual_counts(&h.acl, 0);
    const long before3 = sc_acl_virtual_counts(&h.acl, 3);
    size_t m = mark(&h);
    feed(&h, "SHIFT DELTA BY 1 300\r");
    feed(&h, "SHIFT DELTA BY 4 -40\r");
    MT_EQ(prompts_since(&h, m), 2);
    m = mark(&h);
    feed(&h, "MOVE DELTA\r");
    MT_EQ(prompts_since(&h, m), 1); /* MOVE answers at once; the motion runs behind it */
    MT_EQ(h.core.state, scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    run(&h, 4.0);
    MT_FALSE(h.acl.moving);
    MT_NEAR(sc_acl_virtual_counts(&h.acl, 0), before + 300, 3);
    MT_NEAR(sc_acl_virtual_counts(&h.acl, 3), before3 - 40, 3);
    MT_EQ(h.core.state, scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE); /* held, watchdog fed */
    m = mark(&h);
    feed(&h, "LISTPV POSITION\r");
    long v[5];
    MT_TRUE(last_position(&h, m, v));
    MT_NEAR(v[0], before + 300, 3);
    /* The operator's JointMoveRelative sequence: HERE, SHIFT x3, clrbuf, MOVE. */
    m = mark(&h);
    const long before1 = sc_acl_virtual_counts(&h.acl, 1), before2 = sc_acl_virtual_counts(&h.acl, 2);
    feed(&h, "HERE DELTA\r");
    feed(&h, "SHIFT DELTA BY 2 100\r");
    feed(&h, "SHIFT DELTA BY 3 -100\r");
    feed(&h, "SHIFT DELTA BY 4 0\r");
    feed(&h, "clrbuf\r");
    feed(&h, "MOVE DELTA\r");
    MT_EQ(prompts_since(&h, m), 6);
    run(&h, 4.0);
    MT_NEAR(sc_acl_virtual_counts(&h.acl, 1), before1 + 100, 3);
    MT_NEAR(sc_acl_virtual_counts(&h.acl, 2), before2 - 100, 3);
    MT_NEAR(sc_acl_virtual_counts(&h.acl, 0), before + 300, 3); /* untouched axes stay */
}

static void test_moved_prompts_only_when_done(void)
{
    h_t h;
    h_init(&h, 5);
    start_like_the_operator(&h);
    const long before = sc_acl_virtual_counts(&h.acl, 0);
    feed(&h, "SHIFT DELTA BY 1 400\r");
    const size_t m = mark(&h);
    feed(&h, "MOVED DELTA\r");
    run(&h, 0.2);
    MT_EQ(prompts_since(&h, m), 0);
    run(&h, 5.0);
    MT_EQ(prompts_since(&h, m), 1);
    MT_NEAR(sc_acl_virtual_counts(&h.acl, 0), before + 400, 3);
}

static void test_speed_caps_the_move(void)
{
    double t_full = 0, t_slow = 0;
    for (int pass = 0; pass < 2; ++pass) {
        h_t h;
        h_init(&h, 5);
        start_like_the_operator(&h);
        feed(&h, pass ? "SPEED 25\r" : "SPEED 100\r");
        feed(&h, "SHIFT DELTA BY 1 600\r");
        feed(&h, "MOVE DELTA\r");
        double t = 0;
        while (h.acl.moving && t < 20.0) {
            tick(&h);
            t += 0.001;
        }
        if (pass) t_slow = t; else t_full = t;
    }
    MT_GT(t_full, 0.3);
    MT_GT(t_slow, 2.0 * t_full);
}

static void test_clrbuf_stops_a_move(void)
{
    h_t h;
    h_init(&h, 5);
    start_like_the_operator(&h);
    feed(&h, "SHIFT DELTA BY 1 2000\r");
    feed(&h, "MOVE DELTA\r");
    run(&h, 0.4);
    MT_TRUE(h.acl.moving);
    MT_GT(fabs(h.plant.joint[0].velocity), 0.05);
    const size_t m = mark(&h);
    feed(&h, "clrbuf\r");
    MT_EQ(prompts_since(&h, m), 1);
    MT_FALSE(h.acl.moving);
    run(&h, 0.5);
    MT_LT(fabs(h.plant.joint[0].velocity), 0.01);
    MT_LT(labs(sc_acl_virtual_counts(&h.acl, 0)), 1500);
}

static void test_manual_mode_jogs_while_keys_arrive(void)
{
    h_t h;
    h_init(&h, 5);
    start_like_the_operator(&h);
    size_t m = mark(&h);
    feed(&h, "~");
    MT_TRUE(h.acl.manual);
    MT_EQ(h.core.state, scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    const long start = sc_acl_virtual_counts(&h.acl, 0);
    /* the operator's key spam: one byte every 10 ms for as long as the jog lasts */
    for (int k = 0; k < 60; ++k) {
        feed(&h, "q");
        run(&h, 0.01);
    }
    const long moved = sc_acl_virtual_counts(&h.acl, 0) - start;
    MT_GT(moved, 20);
    MT_GT(h.plant.joint[0].velocity, 0.01);
    run(&h, 0.3); /* keys stop: so does the joint, after the 50 ms hold and the deceleration ramp */
    MT_LT(fabs(h.plant.joint[0].velocity), 0.01);
    MT_NEAR(sc_acl_virtual_counts(&h.acl, 0) - start, moved, 60); /* 60 ER-V counts = 1.4 degrees of coast */
    /* the other way, on another axis */
    const long w0 = sc_acl_virtual_counts(&h.acl, 1);
    for (int k = 0; k < 40; ++k) {
        feed(&h, "w");
        run(&h, 0.01);
    }
    MT_LT(sc_acl_virtual_counts(&h.acl, 1), w0 - 10);
    run(&h, 0.3);
    m = mark(&h);
    feed(&h, "~");
    MT_FALSE(h.acl.manual);
    MT_EQ(prompts_since(&h, m), 1); /* the operator will not send its next command without it */
    /* and a line works again */
    m = mark(&h);
    feed(&h, "DEFP DELTA\r");
    MT_EQ(prompts_since(&h, m), 1);
}

static void test_key_bytes_outside_manual_mode_are_not_jogs(void)
{
    h_t h;
    h_init(&h, 5);
    start_like_the_operator(&h);
    const long start = sc_acl_virtual_counts(&h.acl, 0);
    for (int k = 0; k < 20; ++k) {
        feed(&h, "q");
        run(&h, 0.01);
    }
    MT_EQ(sc_acl_virtual_counts(&h.acl, 0), start);
    const size_t m = mark(&h);
    feed(&h, "\r"); /* "qqqq...q" is not a command */
    MT_TRUE(seen_since(&h, m, "ACL - unrecognized request"));
    MT_EQ(prompts_since(&h, m), 1);
}

static void test_estop_reports_control_disabled_and_con_waits_for_the_button(void)
{
    h_t h;
    h_init(&h, 5);
    start_like_the_operator(&h);
    feed(&h, "SHIFT DELTA BY 1 2000\r");
    feed(&h, "MOVE DELTA\r");
    run(&h, 0.3);
    size_t m = mark(&h);
    h.plant.motor_power = false; /* the mushroom */
    run(&h, 0.05);
    MT_EQ(h.core.state, scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
    MT_TRUE(seen_since(&h, m, "CONTROL DISABLED"));
    MT_EQ(prompts_since(&h, m), 1);
    MT_FALSE(h.acl.moving);
    /* commander's EnableControl -> the operator's CON: refused while the chain is open */
    m = mark(&h);
    feed(&h, "CON\r");
    MT_TRUE(seen_since(&h, m, "CONTROL DISABLED"));
    MT_EQ(prompts_since(&h, m), 1);
    MT_TRUE(h.core.state != scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    run(&h, 1.0);
    MT_EQ(prompts_since(&h, m), 1); /* nothing unsolicited while it stays off */
    /* a person presses MOTORS ON */
    h.plant.motor_power = true;
    run(&h, 0.05);
    m = mark(&h);
    feed(&h, "CON\r");
    MT_FALSE(seen_since(&h, m, "DISABLED"));
    MT_EQ(prompts_since(&h, m), 1);
    MT_EQ(h.core.state, scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    MT_TRUE(h.core.joint[0].homed); /* positions survived the e-stop: no re-homing */
    /* and the next move works */
    feed(&h, "HERE DELTA\r");
    feed(&h, "SHIFT DELTA BY 1 50\r");
    feed(&h, "MOVE DELTA\r");
    const long before = sc_acl_virtual_counts(&h.acl, 0);
    run(&h, 3.0);
    MT_NEAR(sc_acl_virtual_counts(&h.acl, 0), before + 50, 4);
}

static void test_estop_while_idle_is_reported_once(void)
{
    h_t h;
    h_init(&h, 5);
    start_like_the_operator(&h);
    feed(&h, "COFF\r");
    MT_TRUE(h.core.state != scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    const size_t m = mark(&h);
    h.plant.motor_power = false;
    run(&h, 1.0);
    MT_TRUE(seen_since(&h, m, "CONTROL DISABLED"));
    MT_EQ(prompts_since(&h, m), 1);
    h.plant.motor_power = true;
    run(&h, 0.1);
    MT_EQ(prompts_since(&h, m), 1);
}

static void test_a_stall_is_an_impact(void)
{
    h_t h;
    h_init(&h, 5);
    start_like_the_operator(&h);
    const size_t m = mark(&h);
    sc_core_raise_fault(&h.core, scorbot_v1_FaultCode_FAULT_STALL, 2, false);
    run(&h, 0.01);
    MT_TRUE(seen_since(&h, m, "IMPACT"));
    MT_TRUE(seen_since(&h, m, "axis 3"));
    MT_EQ(prompts_since(&h, m), 1);
    /* a stall loses positions on the real core: CON clears it, HOME is needed again */
    feed(&h, "CON\r");
    MT_TRUE(h.core.state != scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
}

static void test_home_fault_ends_the_wait(void)
{
    h_t h;
    h_init(&h, 5);
    run(&h, 0.1);
    feed(&h, "\r");
    const size_t m = mark(&h);
    feed(&h, "HOME\r");
    run(&h, 0.5);
    sc_core_raise_fault(&h.core, scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT, -1, false);
    run(&h, 0.01);
    MT_TRUE(seen_since(&h, m, "FAULT"));
    MT_EQ(prompts_since(&h, m), 1);
    MT_FALSE(h.acl.homing_pending);
}

static void test_unknown_command_and_ctrl_c(void)
{
    h_t h;
    h_init(&h, 5);
    run(&h, 0.1);
    size_t m = mark(&h);
    feed(&h, "FROBNICATE 3\r");
    MT_TRUE(seen_since(&h, m, "ACL - unrecognized request"));
    MT_EQ(prompts_since(&h, m), 1);
    m = mark(&h);
    feed(&h, "\x03");
    MT_EQ(prompts_since(&h, m), 1);
    m = mark(&h);
    feed(&h, "SPEED 250\r");
    MT_TRUE(seen_since(&h, m, "ACL - unrecognized request"));
    MT_EQ(h.acl.speed_percent, 100);
    feed(&h, "speed 40\r");
    MT_EQ(h.acl.speed_percent, 40);
}

static void test_the_slide_is_axis_7(void)
{
    h_t h;
    h_init(&h, 6);
    start_like_the_operator(&h);
    size_t m = mark(&h);
    feed(&h, "LISTPV POSITION\r");
    MT_TRUE(seen_since(&h, m, " 7:"));
    long v[5];
    MT_TRUE(last_position(&h, m, v)); /* the operator's five-pair parse still works */
    const long before = sc_acl_virtual_counts(&h.acl, 5);
    feed(&h, "SHIFT DELTA BY 7 150\r");
    feed(&h, "MOVE DELTA\r");
    run(&h, 4.0);
    MT_NEAR(sc_acl_virtual_counts(&h.acl, 5), before + 150, 3);
    MT_EQ(sc_acl_joint_for_axis(&h.acl, 6), -1); /* no gripper */
    MT_EQ(sc_acl_joint_for_axis(&h.acl, 7), 5);
    m = mark(&h);
    feed(&h, "SHIFT DELTA BY 6 10\r");
    MT_TRUE(seen_since(&h, m, "ACL - unrecognized request"));
}

static void test_the_gripper_is_axis_6_with_seven_channels(void)
{
    /* Rev 0.4: seven channels in the Eshed numbering, the gripper 6 and the rail 7. */
    h_t h;
    h_init(&h, 7);
    start_like_the_operator(&h);
    MT_EQ(sc_acl_joint_for_axis(&h.acl, 6), 5);
    MT_EQ(sc_acl_joint_for_axis(&h.acl, 7), 6);
    size_t m = mark(&h);
    feed(&h, "LISTPV POSITION\r");
    MT_TRUE(seen_since(&h, m, " 6:"));
    MT_TRUE(seen_since(&h, m, " 7:"));
    long v[5];
    MT_TRUE(last_position(&h, m, v)); /* the operator's five-pair parse still works */
    const long before = sc_acl_virtual_counts(&h.acl, 6);
    feed(&h, "SHIFT DELTA BY 7 150\r");
    feed(&h, "MOVE DELTA\r");
    run(&h, 4.0);
    MT_NEAR(sc_acl_virtual_counts(&h.acl, 6), before + 150, 3); /* axis 7 moved the last channel, the rail */
}

static void test_virtual_units_follow_the_configuration(void)
{
    h_t h;
    h_init(&h, 5);
    sc_acl_config_t ac;
    sc_acl_config_default(&ac);
    ac.virtual_sign[0] = -1.0f;
    ac.virtual_counts_per_rad[1] = 1000.0f;
    sc_acl_init(&h.acl, &h.core, &ac);
    start_like_the_operator(&h);
    const long b0 = sc_acl_virtual_counts(&h.acl, 0);
    const double p0 = h.plant.joint[0].true_position, p1 = h.plant.joint[1].true_position;
    feed(&h, "SHIFT DELTA BY 1 100\r");
    feed(&h, "SHIFT DELTA BY 2 500\r");
    feed(&h, "MOVE DELTA\r");
    run(&h, 5.0);
    MT_NEAR(sc_acl_virtual_counts(&h.acl, 0), b0 + 100, 3);
    MT_LT(h.plant.joint[0].true_position, p0 - 0.02); /* the base went the other way */
    MT_NEAR(h.plant.joint[1].true_position, p1 + 0.5, 0.01); /* 500 counts at 1000 per rad */
}

static void test_output_is_text_only(void)
{
    h_t h;
    h_init(&h, 5);
    start_like_the_operator(&h);
    feed(&h, "~");
    for (int k = 0; k < 20; ++k) {
        feed(&h, "1");
        run(&h, 0.01);
    }
    feed(&h, "~");
    feed(&h, "LISTPV POSITION\r");
    feed(&h, "NOPE\r");
    sc_core_raise_fault(&h.core, scorbot_v1_FaultCode_FAULT_OVERCURRENT, 0, false);
    run(&h, 0.1);
    MT_FALSE(h.saw_flow_control_byte);
    for (size_t k = 0; k < h.out_len; ++k) {
        const unsigned char c = (unsigned char)h.out[k];
        MT_CHECK(c == '\r' || c == '\n' || (c >= 0x20 && c < 0x7F), "byte 0x%02x at %zu", c, k);
        if (!(c == '\r' || c == '\n' || (c >= 0x20 && c < 0x7F))) break;
    }
}

static void test_link_mode_sniffer(void)
{
    sc_link_sniffer_t s;
    size_t used = 0;
    uint8_t held[SC_LINK_SNIFF_MAX];
    sc_link_sniffer_init(&s);
    MT_EQ(sc_link_sniffer_feed(&s, (const uint8_t *)"\r", 1, &used), SC_LINK_ACL);
    MT_EQ(used, 1);
    MT_EQ(sc_link_sniffer_take(&s, held, sizeof(held)), 1);
    MT_EQ(held[0], '\r');

    sc_link_sniffer_init(&s);
    const uint8_t frame[] = {0x05, 0x11, 0x22, 0x33, 0x44, 0x00, 0x07, 0x08};
    MT_EQ(sc_link_sniffer_feed(&s, frame, sizeof(frame), &used), SC_LINK_PROTOBUF);
    MT_EQ(used, 6); /* up to and including the delimiter; the rest is the caller's */
    MT_EQ(sc_link_sniffer_take(&s, held, sizeof(held)), 6);
    MT_MEMEQ(held, frame, 6);

    sc_link_sniffer_init(&s);
    MT_EQ(sc_link_sniffer_feed(&s, (const uint8_t *)"DEFP DEL", 8, &used), SC_LINK_UNDECIDED);
    MT_EQ(sc_link_sniffer_feed(&s, (const uint8_t *)"TA\rHOME\r", 8, &used), SC_LINK_ACL);
    MT_EQ(used, 3);
    MT_EQ(sc_link_sniffer_take(&s, held, sizeof(held)), 11);

    sc_link_sniffer_init(&s);
    MT_EQ(sc_link_sniffer_feed(&s, (const uint8_t *)"~", 1, &used), SC_LINK_ACL);

    sc_link_sniffer_init(&s);
    uint8_t junk[SC_LINK_SNIFF_MAX];
    memset(junk, 'a', sizeof(junk));
    MT_EQ(sc_link_sniffer_feed(&s, junk, sizeof(junk), &used), SC_LINK_PROTOBUF);
}

int main(void)
{
    MT_RUN(test_bare_cr_gets_a_prompt);
    MT_RUN(test_home_prompts_after_the_cycle_and_listpv_parses);
    MT_RUN(test_shift_move_reaches_the_target);
    MT_RUN(test_moved_prompts_only_when_done);
    MT_RUN(test_speed_caps_the_move);
    MT_RUN(test_clrbuf_stops_a_move);
    MT_RUN(test_manual_mode_jogs_while_keys_arrive);
    MT_RUN(test_key_bytes_outside_manual_mode_are_not_jogs);
    MT_RUN(test_estop_reports_control_disabled_and_con_waits_for_the_button);
    MT_RUN(test_estop_while_idle_is_reported_once);
    MT_RUN(test_a_stall_is_an_impact);
    MT_RUN(test_home_fault_ends_the_wait);
    MT_RUN(test_unknown_command_and_ctrl_c);
    MT_RUN(test_the_slide_is_axis_7);
    MT_RUN(test_the_gripper_is_axis_6_with_seven_channels);
    MT_RUN(test_virtual_units_follow_the_configuration);
    MT_RUN(test_output_is_text_only);
    MT_RUN(test_link_mode_sniffer);
    return MT_SUMMARY();
}
