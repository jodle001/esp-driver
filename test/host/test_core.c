// The simulator's behaviour tests (scorbot_esp_sim/test/test_controller_sim.cpp), run
// against the real firmware core driving the host plant model, plus the hardware-only
// behaviours the simulator does not have (driver fault line, motor power loss,
// calibration required). Tolerances are wider than the simulator's where a real inner
// loop and a first-order motor replace its ideal integrator.
#include <math.h>
#include <stdlib.h>

#include "minitest.h"
#include "plant.h"
#include "scorbot_core/core.h"

#define MAX_EVENTS 512
#define N 5

typedef struct {
    sc_core_t core;
    plant_t plant;
    sc_core_config_t cfg;
    /* persisted store, what nvs_store does on the device */
    char robot_type[SC_CORE_ROBOT_TYPE_LEN];
    bool has_cal[SC_CORE_MAX_JOINTS];
    scorbot_v1_JointCalibration cal[SC_CORE_MAX_JOINTS];
    uint32_t boot_count;
    /* observations */
    uint32_t next_id;
    uint32_t n_states, n_responses, n_events;
    scorbot_v1_JointState last_state;
    bool have_state;
    scorbot_v1_Event events[MAX_EVENTS];
    scorbot_v1_Response last_response;
    uint32_t last_response_id;
    uint32_t last_seq;
    bool seq_gap;
} harness_t;

static const char *kNames[N] = {"base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint"};

static void persist_cal(void *ctx, const scorbot_v1_JointCalibration *cal)
{
    harness_t *h = (harness_t *)ctx;
    h->cal[cal->index] = *cal;
    h->has_cal[cal->index] = true;
}
static void persist_type(void *ctx, const char *t) { strncpy(((harness_t *)ctx)->robot_type, t, SC_CORE_ROBOT_TYPE_LEN - 1); }

typedef struct {
    const scorbot_v1_JointCalibration *cal; /* N entries or NULL */
    const double *initial;                  /* N entries or NULL */
    const bool *encoder_reversed;           /* N entries or NULL */
    uint32_t home_timeout_ms;
} options_t;

static void boot_core(harness_t *h)
{
    sc_core_config_default(&h->cfg);
    h->cfg.joint_count = N;
    for (int i = 0; i < N; ++i) {
        h->cfg.joint_names[i] = kNames[i];
        h->cfg.has_calibration[i] = h->has_cal[i];
        h->cfg.calibration[i] = h->cal[i];
    }
    strncpy(h->cfg.robot_type, h->robot_type, sizeof(h->cfg.robot_type) - 1);
    h->cfg.boot_count = h->boot_count;
    h->cfg.persist.ctx = h;
    h->cfg.persist.calibration = persist_cal;
    h->cfg.persist.robot_type = persist_type;
    h->cfg.firmware_version = "host-0.1.0";
    if (h->cfg.home_timeout_ms == 0) {
        h->cfg.home_timeout_ms = 30000;
    }
    sc_core_init(&h->core, &h->cfg);
    h->last_seq = 0;
}

static void harness_init(harness_t *h, const options_t *o)
{
    memset(h, 0, sizeof(*h));
    h->next_id = 1;
    h->boot_count = 1;
    strcpy(h->robot_type, "er_4pc");
    for (int i = 0; i < N; ++i) {
        h->cal[i] = o && o->cal ? o->cal[i] : plant_default_calibration((uint32_t)i);
        h->has_cal[i] = true;
    }
    plant_init(&h->plant, N, h->cal, o ? o->initial : NULL, o ? o->encoder_reversed : NULL);
    boot_core(h);
    if (o && o->home_timeout_ms) {
        h->cfg.home_timeout_ms = o->home_timeout_ms;
        sc_core_init(&h->core, &h->cfg);
    }
}

static void drain(harness_t *h)
{
    scorbot_v1_Envelope e;
    while (sc_core_pop(&h->core, &e)) {
        if (h->last_seq && e.seq != h->last_seq + 1) {
            h->seq_gap = true;
        }
        h->last_seq = e.seq;
        switch (e.which_payload) {
        case scorbot_v1_Envelope_joint_state_tag:
            ++h->n_states;
            h->last_state = e.payload.joint_state;
            h->have_state = true;
            break;
        case scorbot_v1_Envelope_response_tag:
            ++h->n_responses;
            h->last_response = e.payload.response;
            h->last_response_id = e.payload.response.request_id;
            break;
        case scorbot_v1_Envelope_event_tag:
            if (h->n_events < MAX_EVENTS) {
                h->events[h->n_events] = e.payload.event;
            }
            ++h->n_events;
            break;
        default: break;
        }
    }
}

static void tick(harness_t *h, double dt)
{
    sc_input_t in;
    sc_output_t out;
    plant_read(&h->plant, &in);
    sc_core_tick(&h->core, (float)dt, &in, &out);
    plant_step(&h->plant, &out, dt);
    drain(h);
}

static void run(harness_t *h, double seconds)
{
    const int steps = (int)(seconds / 0.001 + 0.5);
    for (int i = 0; i < steps; ++i) {
        tick(h, 0.001);
    }
}

static scorbot_v1_Request *begin_request(scorbot_v1_Envelope *e, harness_t *h)
{
    memset(e, 0, sizeof(*e));
    e->seq = 1;
    e->which_payload = scorbot_v1_Envelope_request_tag;
    e->payload.request.request_id = h->next_id++;
    return &e->payload.request;
}

static scorbot_v1_Response send_request(harness_t *h, const scorbot_v1_Envelope *e)
{
    const uint32_t id = e->payload.request.request_id;
    h->last_response_id = 0;
    sc_core_handle(&h->core, e);
    drain(h);
    MT_CHECK(h->last_response_id == id, "no response to request %u", id);
    return h->last_response;
}

static scorbot_v1_Response req_simple(harness_t *h, pb_size_t tag)
{
    scorbot_v1_Envelope e;
    scorbot_v1_Request *r = begin_request(&e, h);
    r->which_body = tag;
    return send_request(h, &e);
}
static scorbot_v1_Response req_enable(harness_t *h, bool allow_unhomed)
{
    scorbot_v1_Envelope e;
    scorbot_v1_Request *r = begin_request(&e, h);
    r->which_body = scorbot_v1_Request_enable_tag;
    r->body.enable.allow_unhomed = allow_unhomed;
    return send_request(h, &e);
}
static scorbot_v1_Response req_home(harness_t *h, uint32_t mask, uint32_t timeout_ms)
{
    scorbot_v1_Envelope e;
    scorbot_v1_Request *r = begin_request(&e, h);
    r->which_body = scorbot_v1_Request_home_tag;
    r->body.home.joint_mask = mask;
    r->body.home.timeout_ms = timeout_ms;
    return send_request(h, &e);
}
static scorbot_v1_Response req_watchdog(harness_t *h, uint32_t ms)
{
    scorbot_v1_Envelope e;
    scorbot_v1_Request *r = begin_request(&e, h);
    r->which_body = scorbot_v1_Request_set_watchdog_tag;
    r->body.set_watchdog.watchdog_ms = ms;
    return send_request(h, &e);
}
static scorbot_v1_Response req_ping(harness_t *h, uint32_t nonce)
{
    scorbot_v1_Envelope e;
    scorbot_v1_Request *r = begin_request(&e, h);
    r->which_body = scorbot_v1_Request_ping_tag;
    r->body.ping.nonce = nonce;
    return send_request(h, &e);
}
static scorbot_v1_Response req_set_cal(harness_t *h, const scorbot_v1_JointCalibration *c)
{
    scorbot_v1_Envelope e;
    scorbot_v1_Request *r = begin_request(&e, h);
    r->which_body = scorbot_v1_Request_set_calibration_tag;
    r->body.set_calibration.has_joint = true;
    r->body.set_calibration.joint = *c;
    return send_request(h, &e);
}
static scorbot_v1_Response req_robot_type(harness_t *h, const char *t)
{
    scorbot_v1_Envelope e;
    scorbot_v1_Request *r = begin_request(&e, h);
    r->which_body = scorbot_v1_Request_set_robot_type_tag;
    strncpy(r->body.set_robot_type.robot_type, t, sizeof(r->body.set_robot_type.robot_type) - 1);
    return send_request(h, &e);
}
#define req_disable(h) req_simple((h), scorbot_v1_Request_disable_tag)
#define req_clear(h) req_simple((h), scorbot_v1_Request_clear_fault_tag)
#define req_info(h) req_simple((h), scorbot_v1_Request_get_info_tag)

static void command(harness_t *h, int joint, scorbot_v1_ControlMode mode, float value)
{
    scorbot_v1_Envelope e;
    memset(&e, 0, sizeof(e));
    e.seq = 1;
    e.which_payload = scorbot_v1_Envelope_joint_command_tag;
    scorbot_v1_JointCommand *c = &e.payload.joint_command;
    c->mode_count = c->position_count = c->velocity_count = N;
    for (int i = 0; i < N; ++i) {
        c->mode[i] = scorbot_v1_ControlMode_CONTROL_MODE_NONE;
    }
    c->mode[joint] = mode;
    if (mode == scorbot_v1_ControlMode_CONTROL_MODE_POSITION) {
        c->position[joint] = value;
    } else {
        c->velocity[joint] = value;
    }
    sc_core_handle(&h->core, &e);
}

/// Every joint in velocity mode at `fraction` of its own maximum, wrist pitch the other
/// way (unhomed, so no soft limits: this is the direction with room on every joint).
static void command_all(harness_t *h, float fraction)
{
    scorbot_v1_Envelope e;
    memset(&e, 0, sizeof(e));
    e.seq = 1;
    e.which_payload = scorbot_v1_Envelope_joint_command_tag;
    scorbot_v1_JointCommand *c = &e.payload.joint_command;
    c->mode_count = c->position_count = c->velocity_count = N;
    for (int i = 0; i < N; ++i) {
        c->mode[i] = scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY;
        c->velocity[i] = (i == 3 ? -fraction : fraction) * h->cal[i].max_velocity_rad_s;
    }
    sc_core_handle(&h->core, &e);
}

static double summed_current(const harness_t *h)
{
    double sum = 0.0;
    for (int i = 0; i < N; ++i) {
        sum += h->plant.joint[i].current;
    }
    return sum;
}

/// Keep the watchdog fed at 100 Hz while running.
static void run_commanding(harness_t *h, double seconds, int joint, scorbot_v1_ControlMode mode, float value)
{
    const int steps = (int)(seconds / 0.01 + 0.5);
    for (int i = 0; i < steps; ++i) {
        command(h, joint, mode, value);
        run(h, 0.01);
    }
}

static void boot(harness_t *h) { run(h, 0.1); }
static scorbot_v1_SystemState state(const harness_t *h) { return h->core.state; }
static const sc_joint_t *cj(const harness_t *h, int i) { return &h->core.joint[i]; }
static const plant_joint_t *pj(const harness_t *h, int i) { return &h->plant.joint[i]; }

static bool saw_event(const harness_t *h, pb_size_t tag, uint32_t value, uint32_t from)
{
    const uint32_t n = h->n_events < MAX_EVENTS ? h->n_events : MAX_EVENTS;
    for (uint32_t k = from; k < n; ++k) {
        const scorbot_v1_Event *ev = &h->events[k];
        if (ev->which_body != tag) {
            continue;
        }
        switch (tag) {
        case scorbot_v1_Event_state_changed_tag: if ((uint32_t)ev->body.state_changed == value) return true; break;
        case scorbot_v1_Event_fault_raised_tag: if ((uint32_t)ev->body.fault_raised == value) return true; break;
        case scorbot_v1_Event_homing_complete_mask_tag: if (ev->body.homing_complete_mask == value) return true; break;
        case scorbot_v1_Event_homing_progress_mask_tag: return true;
        default: return true;
        }
    }
    return false;
}

static void reboot(harness_t *h)
{
    ++h->boot_count;
    plant_reset_counters(&h->plant);
    boot_core(h);
}

/* ---- the simulator's tests --------------------------------------------------------- */

static void test_boots_into_unhomed_and_streams_state(void)
{
    harness_t h;
    harness_init(&h, NULL);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_BOOT);
    run(&h, 1.0);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
    MT_NEAR(h.n_states, 100.0, 2.0);
    MT_TRUE(h.have_state);
    MT_EQ(h.last_state.position_count, 5);
    for (int i = 0; i < 5; ++i) {
        MT_NEAR(h.last_state.position[i], 0.0f, 1e-6);
        MT_FALSE(h.last_state.flags[i].homed);
    }
    MT_TRUE(saw_event(&h, scorbot_v1_Event_state_changed_tag, scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED, 0));
    MT_FALSE(h.seq_gap);
}

static void test_ping_and_info(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    scorbot_v1_Response r = req_ping(&h, 1234);
    MT_EQ(r.result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(r.which_body, scorbot_v1_Response_pong_tag);
    MT_EQ(r.body.pong, 1234u);
    r = req_info(&h);
    MT_EQ(r.which_body, scorbot_v1_Response_get_info_tag);
    MT_EQ(r.body.get_info.protocol_major, SC_PROTOCOL_MAJOR);
    MT_STREQ(r.body.get_info.robot_type, "er_4pc");
    MT_EQ(r.body.get_info.joint_names_count, 5);
    MT_STREQ(r.body.get_info.joint_names[4], "wrist_roll_joint");
    MT_EQ(r.body.get_info.boot_count, 1u);
    MT_EQ(r.body.get_info.watchdog_ms, 200u);
    MT_EQ(r.body.get_info.calibration_count, 5);
    MT_NEAR(r.body.get_info.calibration[0].gear_ratio, 635.5f, 1e-3);
}

static void test_enable_requires_homing_unless_overridden(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    MT_EQ(req_enable(&h, false).result, scorbot_v1_Result_RESULT_INVALID_STATE);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
    MT_EQ(req_enable(&h, true).result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    MT_TRUE(h.core.unhomed_active);
    MT_EQ(req_disable(&h).result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
}

static void test_commands_ignored_unless_active(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    command(&h, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.2f);
    run(&h, 0.5);
    MT_EQ(h.core.commands_ignored, 1u);
    MT_EQ(h.core.commands_received, 0u);
    MT_NEAR(pj(&h, 0)->velocity, 0.0, 1e-9);
}

static void test_velocity_mode_with_acceleration_limit(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    req_enable(&h, true);
    const double vmax = cj(&h, 0)->cal.max_velocity_rad_s;
    run_commanding(&h, 0.05, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 10.0f);
    MT_LT(pj(&h, 0)->velocity, vmax);
    MT_GT(pj(&h, 0)->velocity, 0.0);
    run_commanding(&h, 2.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 10.0f);
    MT_NEAR(pj(&h, 0)->velocity, vmax, 0.01);
    MT_GT(pj(&h, 0)->true_position - pj(&h, 0)->boot_position, 0.3);
    MT_NEAR(h.last_state.velocity[0], vmax, 0.02);
    MT_GT(h.last_state.current[0], 0.15f);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
}

static void test_watchdog_faults_when_commands_stop(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    req_enable(&h, true);
    run_commanding(&h, 0.2, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.1f);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    run(&h, 0.15);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    run(&h, 0.1);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
    MT_EQ(h.core.fault, scorbot_v1_FaultCode_FAULT_WATCHDOG);
    MT_NEAR(pj(&h, 0)->velocity, 0.0, 0.005);
    MT_TRUE(saw_event(&h, scorbot_v1_Event_fault_raised_tag, scorbot_v1_FaultCode_FAULT_WATCHDOG, 0));
    command(&h, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.1f);
    MT_EQ(h.core.commands_ignored, 1u);
    MT_EQ(req_enable(&h, true).result, scorbot_v1_Result_RESULT_INVALID_STATE);
    MT_EQ(req_clear(&h).result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
}

static void test_watchdog_can_be_disabled(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    req_watchdog(&h, 0);
    req_enable(&h, true);
    run(&h, 2.0);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
}

static void test_homing_finds_switches_and_zeroes_encoders(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    for (int i = 0; i < N; ++i) {
        MT_NEAR(pj(&h, i)->true_position, 0.4, 1e-9);
    }
    scorbot_v1_Response r = req_home(&h, 0, 0);
    MT_EQ(r.result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_HOMING);
    MT_EQ(req_home(&h, 0, 0).result, scorbot_v1_Result_RESULT_BUSY);
    MT_EQ(req_enable(&h, false).result, scorbot_v1_Result_RESULT_BUSY);
    run(&h, 15.0);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    for (int i = 0; i < N; ++i) {
        MT_TRUE(cj(&h, i)->homed);
        MT_NEAR(cj(&h, i)->position, pj(&h, i)->true_position, 2e-3);
        MT_NEAR(pj(&h, i)->true_position, pj(&h, i)->cal.home_offset_rad, pj(&h, i)->switch_width + 1e-6);
        MT_TRUE(pj(&h, i)->limit_switch);
    }
    MT_TRUE(saw_event(&h, scorbot_v1_Event_homing_complete_mask_tag, 0x1F, 0));
    MT_EQ(req_enable(&h, false).result, scorbot_v1_Result_RESULT_OK);
    MT_FALSE(h.core.unhomed_active);
    run_commanding(&h, 0.5, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.1f);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
}

static void test_homing_reverses_at_the_hard_stop(void)
{
    scorbot_v1_JointCalibration cal[N];
    double initial[N];
    for (int i = 0; i < N; ++i) {
        cal[i] = plant_default_calibration((uint32_t)i);
        cal[i].soft_limit_min_rad = -0.6f;
        cal[i].soft_limit_max_rad = 0.6f;
        initial[i] = -0.4;
    }
    options_t o = {cal, initial, NULL, 0};
    harness_t h;
    harness_init(&h, &o);
    boot(&h);
    req_home(&h, 0, 0);
    run(&h, 25.0);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    for (int i = 0; i < N; ++i) {
        MT_TRUE(cj(&h, i)->homed);
        MT_TRUE(cj(&h, i)->homing_reversed);
        MT_NEAR(pj(&h, i)->true_position, pj(&h, i)->cal.home_offset_rad, pj(&h, i)->switch_width + 1e-6);
        MT_NEAR(cj(&h, i)->position, pj(&h, i)->true_position, 2e-3);
    }
}

static void test_homing_again_from_the_switch_backs_off(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    req_home(&h, 0, 0);
    run(&h, 15.0);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    for (int i = 0; i < N; ++i) {
        MT_TRUE(pj(&h, i)->limit_switch);
    }
    const uint32_t events_before = h.n_events;
    req_home(&h, 0, 0);
    run(&h, 0.02);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_HOMING);
    MT_FALSE(saw_event(&h, scorbot_v1_Event_homing_progress_mask_tag, 0, events_before));
    bool released = false;
    double farthest = 0.0;
    int steps = 20;
    for (; steps < 20000 && state(&h) == scorbot_v1_SystemState_SYSTEM_STATE_HOMING; ++steps) {
        tick(&h, 0.001);
        released |= !pj(&h, 0)->limit_switch;
        const double d = fabs(pj(&h, 0)->true_position - pj(&h, 0)->cal.home_offset_rad);
        farthest = d > farthest ? d : farthest;
    }
    MT_TRUE(released);
    MT_GT(farthest, h.cfg.backoff_margin_rad);
    MT_GT(steps, 200);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    for (int i = 0; i < N; ++i) {
        MT_TRUE(cj(&h, i)->homed);
        MT_TRUE(pj(&h, i)->limit_switch);
        MT_NEAR(cj(&h, i)->position, pj(&h, i)->true_position, 2e-3);
    }
}

static void test_homing_times_out(void)
{
    options_t o = {NULL, NULL, NULL, 500};
    harness_t h;
    harness_init(&h, &o);
    boot(&h);
    req_home(&h, 0, 0);
    run(&h, 1.0);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
    MT_EQ(h.core.fault, scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT);
    MT_EQ(req_clear(&h).result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
}

static void test_position_mode_tracks_and_soft_limits_clamp(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    req_home(&h, 0, 0);
    run(&h, 15.0);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    req_enable(&h, false);
    run_commanding(&h, 4.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_POSITION, 0.5f);
    MT_NEAR(cj(&h, 0)->position, 0.5, 0.01);
    MT_FALSE(cj(&h, 0)->at_soft_limit);
    run_commanding(&h, 20.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_POSITION, 10.0f);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    MT_NEAR(cj(&h, 0)->position, plant_soft_max(pj(&h, 0)), 0.01);
    MT_TRUE(h.last_state.flags[0].at_soft_limit);
    MT_LT(h.last_state.position[0], (float)plant_soft_max(pj(&h, 0)) + 5e-3f);
}

static void test_velocity_into_soft_limit_ramps_to_zero(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    req_home(&h, 0, 0);
    run(&h, 15.0);
    req_enable(&h, false);
    run_commanding(&h, 30.0, 1, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 5.0f);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    MT_NEAR(cj(&h, 1)->position, plant_soft_max(pj(&h, 1)), 0.02);
    MT_NEAR(pj(&h, 1)->velocity, 0.0, 0.05);
    MT_LT(pj(&h, 1)->current, pj(&h, 1)->cal.current_limit_a);
}

static void test_unhomed_active_has_no_soft_limits_and_stalls(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    req_watchdog(&h, 0);
    req_enable(&h, true);
    run_commanding(&h, 60.0, 1, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 1.0f);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
    MT_EQ(h.core.fault, scorbot_v1_FaultCode_FAULT_OVERCURRENT);
    MT_TRUE(h.last_state.flags[1].fault);
    MT_FALSE(h.last_state.flags[0].fault);
}

static void test_clear_fault_keeps_homing_when_positions_are_trusted(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    req_home(&h, 0, 0);
    run(&h, 15.0);
    req_enable(&h, false);
    sc_core_raise_fault(&h.core, scorbot_v1_FaultCode_FAULT_OVERCURRENT, 2, false);
    drain(&h);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
    MT_EQ(req_clear(&h).result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    MT_TRUE(cj(&h, 2)->homed);
    MT_FALSE(cj(&h, 2)->fault);
    req_enable(&h, false);
    sc_core_raise_fault(&h.core, scorbot_v1_FaultCode_FAULT_ENCODER, 2, false);
    drain(&h);
    req_clear(&h);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
    MT_FALSE(cj(&h, 0)->homed);
}

static void test_reboot_bumps_boot_count_and_forgets_homing(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    req_watchdog(&h, 50);
    req_robot_type(&h, "er_v");
    req_home(&h, 0, 0);
    run(&h, 15.0);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    reboot(&h);
    run(&h, 0.1);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
    scorbot_v1_Response r = req_info(&h);
    MT_EQ(r.body.get_info.boot_count, 2u);
    MT_STREQ(r.body.get_info.robot_type, "er_v");
    MT_EQ(r.body.get_info.watchdog_ms, 200u);
    MT_FALSE(cj(&h, 0)->homed);
    MT_NEAR(cj(&h, 0)->position, 0.0, 1e-6);
    MT_EQ(h.last_state.boot_count, 2u);
}

static void test_calibration_is_validated_and_applied(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    scorbot_v1_JointCalibration cal = plant_default_calibration(3);
    cal.soft_limit_max_rad = 0.3f;
    cal.max_velocity_rad_s = 0.2f;
    MT_EQ(req_set_cal(&h, &cal).result, scorbot_v1_Result_RESULT_OK);
    MT_NEAR(cj(&h, 3)->cal.soft_limit_max_rad, 0.3f, 1e-9);
    MT_NEAR(h.cal[3].soft_limit_max_rad, 0.3f, 1e-9); /* persisted */
    scorbot_v1_JointCalibration bad = cal;
    bad.index = 9;
    MT_EQ(req_set_cal(&h, &bad).result, scorbot_v1_Result_RESULT_INVALID_ARGUMENT);
    bad = cal;
    bad.gear_ratio = -1.0f;
    MT_EQ(req_set_cal(&h, &bad).result, scorbot_v1_Result_RESULT_INVALID_ARGUMENT);
    req_enable(&h, true);
    MT_EQ(req_set_cal(&h, &cal).result, scorbot_v1_Result_RESULT_INVALID_STATE);
}

static void test_unknown_request_is_unsupported(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    scorbot_v1_Envelope e;
    scorbot_v1_Request *r = begin_request(&e, &h);
    r->which_body = 0;
    MT_EQ(send_request(&h, &e).result, scorbot_v1_Result_RESULT_UNSUPPORTED);
}

static void test_invert_flips_reported_sign(void)
{
    scorbot_v1_JointCalibration cal[N];
    for (int i = 0; i < N; ++i) {
        cal[i] = plant_default_calibration((uint32_t)i);
    }
    cal[0].invert = true;
    options_t o = {cal, NULL, NULL, 0};
    harness_t h;
    harness_init(&h, &o);
    boot(&h);
    req_watchdog(&h, 0);
    req_enable(&h, true);
    run_commanding(&h, 2.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.2f);
    MT_GT(h.last_state.velocity[0], 0.1f);
    MT_LT(pj(&h, 0)->velocity, 0.0);
}

static void test_swapped_encoder_phases_corrected_by_encoder_invert(void)
{
    scorbot_v1_JointCalibration cal[N];
    bool reversed[N] = {true, false, false, false, false};
    for (int i = 0; i < N; ++i) {
        cal[i] = plant_default_calibration((uint32_t)i);
    }
    cal[0].encoder_invert = true;
    options_t o = {cal, NULL, reversed, 0};
    harness_t h;
    harness_init(&h, &o);
    boot(&h);
    req_watchdog(&h, 0);
    req_enable(&h, true);
    run_commanding(&h, 2.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.2f);
    MT_GT(pj(&h, 0)->velocity, 0.1);
    MT_GT(h.last_state.velocity[0], 0.1f);
    MT_GT(h.last_state.position[0], 0.2f);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
}

static void test_encoder_phase_mismatch_is_a_runaway(void)
{
    bool reversed[N] = {true, false, false, false, false};
    options_t o = {NULL, NULL, reversed, 0};
    harness_t h;
    harness_init(&h, &o);
    boot(&h);
    req_watchdog(&h, 0);
    req_enable(&h, true);
    run_commanding(&h, 1.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.2f);
    MT_GT(pj(&h, 0)->velocity, 0.1);
    MT_LT(h.last_state.velocity[0], -0.1f);
    run_commanding(&h, 12.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_POSITION, 0.1f);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
    MT_EQ(h.core.fault, scorbot_v1_FaultCode_FAULT_OVERCURRENT);
}

/* ---- hardware behaviours the simulator does not model ------------------------------ */

static void test_driver_fault_line_faults_and_loses_positions(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    req_home(&h, 0, 0);
    run(&h, 15.0);
    req_enable(&h, false);
    sc_input_t in;
    sc_output_t out;
    plant_read(&h.plant, &in);
    in.joint[3].driver_fault = true;
    sc_core_tick(&h.core, 0.001f, &in, &out);
    drain(&h);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
    MT_EQ(h.core.fault, scorbot_v1_FaultCode_FAULT_DRIVER);
    MT_TRUE(cj(&h, 3)->fault);
    MT_FALSE(out.drives_enabled);
    req_clear(&h);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
}

static void test_estop_power_loss_keeps_positions(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    req_home(&h, 0, 0);
    run(&h, 15.0);
    req_enable(&h, false);
    run_commanding(&h, 0.2, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.1f);
    h.plant.motor_power = false; /* e-stop pressed */
    run(&h, 0.01);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
    MT_EQ(h.core.fault, scorbot_v1_FaultCode_FAULT_DRIVER);
    MT_TRUE(saw_event(&h, scorbot_v1_Event_fault_raised_tag, scorbot_v1_FaultCode_FAULT_DRIVER, 0));
    MT_EQ(req_clear(&h).result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    MT_TRUE(cj(&h, 0)->homed);
    run(&h, 0.01);
    MT_EQ(req_enable(&h, false).result, scorbot_v1_Result_RESULT_INVALID_STATE); /* still off */
    MT_EQ(req_home(&h, 0, 0).result, scorbot_v1_Result_RESULT_INVALID_STATE);
    h.plant.motor_power = true;
    run(&h, 0.01);
    MT_EQ(req_enable(&h, false).result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
}

static void test_enable_requires_calibration_on_the_device(void)
{
    harness_t h;
    harness_init(&h, NULL);
    for (int i = 0; i < N; ++i) {
        h.has_cal[i] = false;
    }
    boot_core(&h);
    boot(&h);
    scorbot_v1_Response r = req_info(&h);
    MT_EQ(r.body.get_info.calibration_count, 0);
    MT_EQ(req_enable(&h, true).result, scorbot_v1_Result_RESULT_INVALID_STATE);
    MT_EQ(req_home(&h, 0, 0).result, scorbot_v1_Result_RESULT_INVALID_STATE);
    for (int i = 0; i < N; ++i) {
        scorbot_v1_JointCalibration c = plant_default_calibration((uint32_t)i);
        MT_EQ(req_set_cal(&h, &c).result, scorbot_v1_Result_RESULT_OK);
    }
    MT_EQ(req_enable(&h, true).result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(req_info(&h).body.get_info.calibration_count, 5);
}

static void test_events_precede_responses_without_seq_gaps(void)
{
    harness_t h;
    harness_init(&h, NULL);
    boot(&h);
    const uint32_t before = h.last_seq;
    scorbot_v1_Envelope e;
    scorbot_v1_Request *r = begin_request(&e, &h);
    r->which_body = scorbot_v1_Request_enable_tag;
    r->body.enable.allow_unhomed = true;
    sc_core_handle(&h.core, &e);
    scorbot_v1_Envelope first, second;
    MT_TRUE(sc_core_pop(&h.core, &first));
    MT_TRUE(sc_core_pop(&h.core, &second));
    MT_EQ(first.which_payload, scorbot_v1_Envelope_event_tag);
    MT_EQ(first.payload.event.body.state_changed, scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    MT_EQ(second.which_payload, scorbot_v1_Envelope_response_tag);
    MT_EQ(first.seq, before + 1);
    MT_EQ(second.seq, before + 2);
}

/* ---- rev 0.2 hardware: the summed-current cap ------------------------------------- */

static void test_total_current_cap_scales_every_duty(void)
{
    harness_t h;
    harness_init(&h, NULL);
    h.cfg.total_current_limit_a = 3.0f; /* five axes at 60 % speed draw about 3.75 A on the plant */
    sc_core_init(&h.core, &h.cfg);
    boot(&h);
    req_enable(&h, true);
    MT_CHECK(state(&h) == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE, "enable_unhomed");
    double peak = 0.0;
    for (int k = 0; k < 200; ++k) {
        command_all(&h, 0.6f);
        run(&h, 0.01);
        if (k >= 50 && summed_current(&h) > peak) peak = summed_current(&h); /* after the start-up transient */
    }
    const double capped = summed_current(&h);
    MT_CHECK(capped <= 3.0 * 1.15, "summed current %.2f A settles near the 3 A cap", capped);
    MT_CHECK(peak <= 3.0 * 1.25, "summed current never runs far past the cap (peak %.2f A)", peak);
    MT_CHECK(h.core.current_scale < 1.0f, "the cap is active (scale %.2f)", (double)h.core.current_scale);
    MT_CHECK(state(&h) == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE, "no fault under the cap (%s)", sc_fault_name(h.core.fault));
    for (int i = 0; i < N; ++i) {
        const double v = fabs(pj(&h, i)->velocity), vcmd = 0.6 * h.cal[i].max_velocity_rad_s;
        MT_CHECK(v > 0.2 * vcmd && v < 0.9 * vcmd, "joint %d still moves, slower (%.2f of the command)", i, v / vcmd);
    }
    /* Lift the cap: the same command draws more than the cap allowed, and no fault. */
    h.core.cfg.total_current_limit_a = 0.0f;
    for (int k = 0; k < 100; ++k) {
        command_all(&h, 0.6f);
        run(&h, 0.01);
    }
    const double free = summed_current(&h);
    MT_CHECK(free > 3.0 * 1.15, "without the cap the same command draws %.2f A", free);
    MT_CHECK(h.core.current_scale == 1.0f, "scale back to 1");
    MT_CHECK(state(&h) == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE, "no fault after lifting the cap (%s)", sc_fault_name(h.core.fault));
}

/* ---- the encoder 5 V sense (design doc 8.3) ------------------------------------------ */

static void enc_pins(harness_t *h, bool h_pin, bool l_pin)
{
    h->plant.enc_supply_valid = true;
    h->plant.enc_supply_h = h_pin;
    h->plant.enc_supply_l = l_pin;
}
#define enc_present(h) enc_pins((h), true, false)
#define enc_absent(h) enc_pins((h), false, true)

static void test_enc_supply_rule_is_off_without_the_sense(void)
{
    harness_t h;
    harness_init(&h, NULL); /* the plant does not report the pins */
    boot(&h);
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_UNKNOWN);
    MT_FALSE(h.core.enc_seen);
    MT_EQ(req_home(&h, 0, 0).result, scorbot_v1_Result_RESULT_OK);
    run(&h, 15.0);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    MT_EQ(req_enable(&h, false).result, scorbot_v1_Result_RESULT_OK);
}

static void test_enc_supply_must_be_present_to_home_and_enable(void)
{
    harness_t h;
    harness_init(&h, NULL);
    enc_absent(&h); /* PC supply off, the Pi powers the DevKit */
    boot(&h);
    run(&h, 0.1);
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_LOST);
    MT_EQ(h.core.enc_losses, 0); /* never present since boot: not a loss */
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
    MT_EQ(req_home(&h, 0, 0).result, scorbot_v1_Result_RESULT_INVALID_STATE);
    MT_EQ(req_enable(&h, true).result, scorbot_v1_Result_RESULT_INVALID_STATE);
    /* The supply comes up: refused until it has been present for the return time. */
    enc_present(&h);
    run(&h, 0.099);
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_LOST);
    MT_EQ(req_home(&h, 0, 0).result, scorbot_v1_Result_RESULT_INVALID_STATE);
    run(&h, 0.001);
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_PRESENT);
    /* Nothing was lost, so an unhomed Enable is allowed as after any boot. */
    MT_EQ(req_enable(&h, true).result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(req_disable(&h).result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(req_home(&h, 0, 0).result, scorbot_v1_Result_RESULT_OK);
    run(&h, 15.0);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    MT_EQ(req_enable(&h, false).result, scorbot_v1_Result_RESULT_OK);
}

static void test_enc_supply_no_verdict_before_the_pins_settle(void)
{
    harness_t h;
    harness_init(&h, NULL);
    enc_absent(&h);
    run(&h, 0.049); /* three bad samples came long ago, but the settle time has not passed */
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_UNKNOWN);
    MT_EQ(h.core.enc_losses, 0);
    run(&h, 0.002);
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_LOST);
}

static void test_enc_supply_loss_while_ready_unhomes_without_a_fault(void)
{
    harness_t h;
    harness_init(&h, NULL);
    enc_present(&h);
    boot(&h);
    run(&h, 0.1);
    MT_EQ(req_home(&h, 0, 0).result, scorbot_v1_Result_RESULT_OK);
    run(&h, 15.0);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    enc_absent(&h);
    run(&h, 0.002); /* two samples: inside the debounce */
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    MT_TRUE(cj(&h, 0)->homed);
    run(&h, 0.001); /* the third */
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_LOST);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
    MT_EQ(h.core.fault, scorbot_v1_FaultCode_FAULT_NONE);
    for (int i = 0; i < N; ++i) {
        MT_FALSE(cj(&h, i)->homed);
    }
    /* Back, and stable: Enable stays refused until Home has completed again. */
    enc_present(&h);
    run(&h, 0.2);
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_PRESENT);
    MT_EQ(req_enable(&h, false).result, scorbot_v1_Result_RESULT_INVALID_STATE);
    MT_EQ(req_enable(&h, true).result, scorbot_v1_Result_RESULT_INVALID_STATE); /* not even unhomed */
    MT_TRUE(h.core.enc_rehome);
    MT_EQ(req_home(&h, 0, 0).result, scorbot_v1_Result_RESULT_OK);
    run(&h, 15.0);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    MT_FALSE(h.core.enc_rehome);
    MT_EQ(req_enable(&h, false).result, scorbot_v1_Result_RESULT_OK);
}

static void test_enc_supply_loss_while_active_faults_and_loses_positions(void)
{
    harness_t h;
    harness_init(&h, NULL);
    enc_present(&h);
    boot(&h);
    run(&h, 0.1);
    req_home(&h, 0, 0);
    run(&h, 15.0);
    req_enable(&h, false);
    run_commanding(&h, 0.2, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.1f);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    enc_absent(&h);
    run(&h, 0.003);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
    MT_EQ(h.core.fault, scorbot_v1_FaultCode_FAULT_ENCODER);
    MT_TRUE(saw_event(&h, scorbot_v1_Event_fault_raised_tag, scorbot_v1_FaultCode_FAULT_ENCODER, 0));
    sc_input_t in;
    sc_output_t out;
    plant_read(&h.plant, &in);
    sc_core_tick(&h.core, 0.001f, &in, &out);
    drain(&h);
    MT_FALSE(out.drives_enabled);
    MT_EQ(req_clear(&h).result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
    MT_FALSE(cj(&h, 0)->homed);
    MT_EQ(req_home(&h, 0, 0).result, scorbot_v1_Result_RESULT_INVALID_STATE); /* still absent */
    MT_EQ(req_enable(&h, true).result, scorbot_v1_Result_RESULT_INVALID_STATE);
}

static void test_enc_supply_loss_during_homing_faults(void)
{
    harness_t h;
    harness_init(&h, NULL);
    enc_present(&h);
    boot(&h);
    run(&h, 0.1);
    req_home(&h, 0, 0);
    run(&h, 0.5);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_HOMING);
    enc_absent(&h);
    run(&h, 0.003);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
    MT_EQ(h.core.fault, scorbot_v1_FaultCode_FAULT_ENCODER);
}

static void test_enc_supply_loss_during_an_estop_fault_still_loses_positions(void)
{
    harness_t h;
    harness_init(&h, NULL);
    enc_present(&h);
    boot(&h);
    run(&h, 0.1);
    req_home(&h, 0, 0);
    run(&h, 15.0);
    req_enable(&h, false);
    run_commanding(&h, 0.1, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.1f);
    h.plant.motor_power = false; /* the e-stop: positions would survive this fault */
    run(&h, 0.01);
    MT_EQ(h.core.fault, scorbot_v1_FaultCode_FAULT_DRIVER);
    enc_absent(&h);              /* then the whole supply goes */
    run(&h, 0.01);
    h.plant.motor_power = true;
    enc_present(&h);
    run(&h, 0.2);
    MT_EQ(req_clear(&h).result, scorbot_v1_Result_RESULT_OK);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
}

static void test_enc_supply_debounce(void)
{
    harness_t h;
    harness_init(&h, NULL);
    enc_present(&h);
    boot(&h);
    run(&h, 0.1);
    req_home(&h, 0, 0);
    run(&h, 15.0);
    /* A glitch of two samples is not a loss. */
    enc_absent(&h);
    run(&h, 0.002);
    enc_present(&h);
    run(&h, 0.05);
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_PRESENT);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_READY);
    MT_EQ(h.core.enc_losses, 0);
    /* The return count restarts on any sample that is not "present". */
    enc_absent(&h);
    run(&h, 0.01);
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_LOST);
    enc_present(&h);
    run(&h, 0.099);
    enc_absent(&h);
    run(&h, 0.001);
    enc_present(&h);
    run(&h, 0.099);
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_LOST);
    run(&h, 0.001);
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_PRESENT);
}

static void test_enc_supply_disagreement_is_a_loss_and_is_flagged(void)
{
    harness_t h;
    harness_init(&h, NULL);
    enc_present(&h);
    boot(&h);
    run(&h, 0.1);
    req_home(&h, 0, 0);
    run(&h, 15.0);
    enc_pins(&h, true, true); /* H says present, L says absent */
    run(&h, 0.003);
    MT_EQ(h.core.enc_supply, SC_ENC_SUPPLY_LOST);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
    MT_FALSE(h.core.enc_disagree);
    run(&h, 5.1);
    MT_TRUE(h.core.enc_disagree);
    MT_FALSE(h.core.enc_stuck_low);
    enc_pins(&h, false, false); /* both low: the comparator channel stuck */
    run(&h, 0.001);
    MT_FALSE(h.core.enc_stuck_low);
    run(&h, 0.001);
    MT_TRUE(h.core.enc_stuck_low);
}

static void test_enc_supply_marginal_rail_is_flagged(void)
{
    harness_t h;
    harness_init(&h, NULL);
    enc_present(&h);
    boot(&h);
    run(&h, 0.1);
    req_home(&h, 0, 0);
    run(&h, 15.0);
    req_enable(&h, false);
    /* One-sample dips, never three in a row: no loss, but the rail is hovering. */
    for (int k = 0; k < 6; ++k) {
        command(&h, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.0f);
        enc_absent(&h);
        run(&h, 0.001);
        enc_present(&h);
        run(&h, 0.099);
    }
    run_commanding(&h, 1.0, 0, scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY, 0.0f);
    MT_EQ(state(&h), scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    MT_EQ(h.core.enc_losses, 0);
    MT_TRUE(h.core.enc_marginal);
}

int main(void)
{
    MT_RUN(test_boots_into_unhomed_and_streams_state);
    MT_RUN(test_ping_and_info);
    MT_RUN(test_enable_requires_homing_unless_overridden);
    MT_RUN(test_commands_ignored_unless_active);
    MT_RUN(test_velocity_mode_with_acceleration_limit);
    MT_RUN(test_watchdog_faults_when_commands_stop);
    MT_RUN(test_watchdog_can_be_disabled);
    MT_RUN(test_homing_finds_switches_and_zeroes_encoders);
    MT_RUN(test_homing_reverses_at_the_hard_stop);
    MT_RUN(test_homing_again_from_the_switch_backs_off);
    MT_RUN(test_homing_times_out);
    MT_RUN(test_position_mode_tracks_and_soft_limits_clamp);
    MT_RUN(test_velocity_into_soft_limit_ramps_to_zero);
    MT_RUN(test_unhomed_active_has_no_soft_limits_and_stalls);
    MT_RUN(test_clear_fault_keeps_homing_when_positions_are_trusted);
    MT_RUN(test_reboot_bumps_boot_count_and_forgets_homing);
    MT_RUN(test_calibration_is_validated_and_applied);
    MT_RUN(test_unknown_request_is_unsupported);
    MT_RUN(test_invert_flips_reported_sign);
    MT_RUN(test_swapped_encoder_phases_corrected_by_encoder_invert);
    MT_RUN(test_encoder_phase_mismatch_is_a_runaway);
    MT_RUN(test_driver_fault_line_faults_and_loses_positions);
    MT_RUN(test_estop_power_loss_keeps_positions);
    MT_RUN(test_enable_requires_calibration_on_the_device);
    MT_RUN(test_events_precede_responses_without_seq_gaps);
    MT_RUN(test_total_current_cap_scales_every_duty);
    MT_RUN(test_enc_supply_rule_is_off_without_the_sense);
    MT_RUN(test_enc_supply_must_be_present_to_home_and_enable);
    MT_RUN(test_enc_supply_no_verdict_before_the_pins_settle);
    MT_RUN(test_enc_supply_loss_while_ready_unhomes_without_a_fault);
    MT_RUN(test_enc_supply_loss_while_active_faults_and_loses_positions);
    MT_RUN(test_enc_supply_loss_during_homing_faults);
    MT_RUN(test_enc_supply_loss_during_an_estop_fault_still_loses_positions);
    MT_RUN(test_enc_supply_debounce);
    MT_RUN(test_enc_supply_disagreement_is_a_loss_and_is_flagged);
    MT_RUN(test_enc_supply_marginal_rail_is_flagged);
    return MT_SUMMARY();
}
