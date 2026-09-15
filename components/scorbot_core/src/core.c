// scorbot_core implementation. Behaviour follows scorbot_protocol/docs/protocol.md
// section 4 and scorbot_esp_sim/src/controller_sim.cpp (the executable spec); the
// comments call out where this differs because a real motor sits behind it.
#include "scorbot_core/core.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f

/* ---- small helpers -------------------------------------------------------------- */

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void copy_str(char *dst, size_t cap, const char *src)
{
    size_t n = 0;
    if (src) {
        while (n + 1 < cap && src[n]) {
            dst[n] = src[n];
            ++n;
        }
    }
    dst[n] = '\0';
}

static float counts_per_rad(const scorbot_v1_JointCalibration *c)
{
    return (float)c->counts_per_motor_rev * c->gear_ratio / (2.0f * PI_F);
}

static float joint_sign(const sc_joint_t *j) { return j->cal.invert ? -1.0f : 1.0f; }

static bool drives_enabled(const sc_core_t *core)
{
    return core->state == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE ||
           core->state == scorbot_v1_SystemState_SYSTEM_STATE_HOMING;
}

static bool positions_trusted(const sc_core_t *core)
{
    for (uint8_t i = 0; i < core->joint_count; ++i) {
        if (!core->joint[i].homed) {
            return false;
        }
    }
    return true;
}

static sc_gains_t effective_gains(const sc_core_t *core, uint8_t i)
{
    sc_gains_t g = core->cfg.gains[i];
    const float vmax = core->joint[i].cal.max_velocity_rad_s > 0.0f ? core->joint[i].cal.max_velocity_rad_s : 1.0f;
    const float v_full = 1.5f * vmax; /* duty 1.0 assumed to reach this; tune on the bench */
    if (g.kff <= 0.0f) {
        g.kff = 1.0f / v_full;
    }
    if (g.kp <= 0.0f) {
        g.kp = 2.0f / v_full;
    }
    if (g.ki <= 0.0f) {
        g.ki = 8.0f / v_full;
    }
    return g;
}

/* ---- outbound queue ------------------------------------------------------------- */

static scorbot_v1_Envelope *new_envelope(sc_core_t *core)
{
    if (core->out_count == SC_CORE_OUTBOUND) {
        /* Queue full: drop the oldest. The glue drains every tick, so this only happens
           if the link is dead, in which case nothing is lost that matters. */
        core->out_head = (uint8_t)((core->out_head + 1) % SC_CORE_OUTBOUND);
        --core->out_count;
        ++core->out_dropped;
    }
    const uint8_t idx = (uint8_t)((core->out_head + core->out_count) % SC_CORE_OUTBOUND);
    ++core->out_count;
    scorbot_v1_Envelope *e = &core->outbound[idx];
    memset(e, 0, sizeof(*e));
    e->seq = ++core->seq;
    e->timestamp_us = core->now_us;
    return e;
}

bool sc_core_pop(sc_core_t *core, scorbot_v1_Envelope *out)
{
    if (core->out_count == 0) {
        return false;
    }
    *out = core->outbound[core->out_head];
    core->out_head = (uint8_t)((core->out_head + 1) % SC_CORE_OUTBOUND);
    --core->out_count;
    return true;
}

uint8_t sc_core_outbound_count(const sc_core_t *core) { return core->out_count; }

static void emit_event_u32(sc_core_t *core, pb_size_t tag, uint32_t value)
{
    scorbot_v1_Envelope *e = new_envelope(core);
    e->which_payload = scorbot_v1_Envelope_event_tag;
    e->payload.event.which_body = tag;
    switch (tag) {
    case scorbot_v1_Event_fault_raised_tag: e->payload.event.body.fault_raised = (scorbot_v1_FaultCode)value; break;
    case scorbot_v1_Event_homing_progress_mask_tag: e->payload.event.body.homing_progress_mask = value; break;
    case scorbot_v1_Event_homing_complete_mask_tag: e->payload.event.body.homing_complete_mask = value; break;
    case scorbot_v1_Event_limit_switch_mask_tag: e->payload.event.body.limit_switch_mask = value; break;
    case scorbot_v1_Event_state_changed_tag: e->payload.event.body.state_changed = (scorbot_v1_SystemState)value; break;
    default: break;
    }
}

static void emit_joint_state(sc_core_t *core)
{
    scorbot_v1_Envelope *e = new_envelope(core);
    e->which_payload = scorbot_v1_Envelope_joint_state_tag;
    scorbot_v1_JointState *s = &e->payload.joint_state;
    s->state = core->state;
    s->fault = core->fault;
    s->position_count = s->velocity_count = s->current_count = s->flags_count = core->joint_count;
    for (uint8_t i = 0; i < core->joint_count; ++i) {
        const sc_joint_t *j = &core->joint[i];
        const float sign = joint_sign(j);
        s->position[i] = sign * j->position;
        s->velocity[i] = sign * j->velocity;
        s->current[i] = sign * (j->duty < 0.0f ? -j->current : j->current);
        s->flags[i].homed = j->homed;
        s->flags[i].limit_switch = j->limit_switch;
        s->flags[i].at_soft_limit = j->at_soft_limit;
        s->flags[i].fault = j->fault;
    }
    s->commands_received = core->commands_received;
    s->commands_ignored = core->commands_ignored;
    s->control_loop_period_us = (uint32_t)(core->last_dt_s * 1e6f);
    s->boot_count = core->cfg.boot_count;
}

/* ---- state machine -------------------------------------------------------------- */

static void stop_joint_control(sc_joint_t *j)
{
    j->mode = scorbot_v1_ControlMode_CONTROL_MODE_NONE;
    j->v_ref = 0.0f;
    j->integral = 0.0f;
    j->duty = 0.0f;
    j->brake = true;
}

static void enter_state(sc_core_t *core, scorbot_v1_SystemState next)
{
    if (next == core->state) {
        return;
    }
    core->state = next;
    emit_event_u32(core, scorbot_v1_Event_state_changed_tag, (uint32_t)next);
    if (next != scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE) {
        core->unhomed_active = core->unhomed_active && next == scorbot_v1_SystemState_SYSTEM_STATE_FAULT;
    }
    if (next != scorbot_v1_SystemState_SYSTEM_STATE_HOMING) {
        for (uint8_t i = 0; i < core->joint_count; ++i) {
            core->joint[i].homing = false;
        }
    }
    if (!drives_enabled(core)) {
        for (uint8_t i = 0; i < core->joint_count; ++i) {
            stop_joint_control(&core->joint[i]);
        }
    }
}

static bool fault_keeps_positions(scorbot_v1_FaultCode code)
{
    switch (code) {
    case scorbot_v1_FaultCode_FAULT_ENCODER:
    case scorbot_v1_FaultCode_FAULT_DRIVER:
    case scorbot_v1_FaultCode_FAULT_STALL:
    case scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT:
    case scorbot_v1_FaultCode_FAULT_INTERNAL:
        return false;
    default:
        return true;
    }
}

void sc_core_raise_fault(sc_core_t *core, scorbot_v1_FaultCode code, int joint, bool trusted)
{
    if (core->state == scorbot_v1_SystemState_SYSTEM_STATE_FAULT) {
        return;
    }
    core->fault = code;
    core->fault_trusted = trusted || fault_keeps_positions(code);
    if (joint >= 0 && joint < core->joint_count) {
        core->joint[joint].fault = true;
    }
    emit_event_u32(core, scorbot_v1_Event_fault_raised_tag, (uint32_t)code);
    enter_state(core, scorbot_v1_SystemState_SYSTEM_STATE_FAULT);
}

/* Summed-current cap: time constant of the integral limiter for a 100 % error, and the
   floor of the velocity-demand factor (never all the way to zero, so a capped robot
   still creeps and the current measurement stays meaningful). */
#define SC_CURRENT_CAP_TAU_S 0.05f
#define SC_CURRENT_CAP_MIN_SCALE 0.05f

static void raise_fault(sc_core_t *core, scorbot_v1_FaultCode code, int joint)
{
    sc_core_raise_fault(core, code, joint, false);
}

/* ---- configuration -------------------------------------------------------------- */

void sc_core_config_default(sc_core_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->firmware_version = "0.1.0";
    copy_str(cfg->robot_type, sizeof(cfg->robot_type), "unset");
    cfg->boot_count = 1;
    cfg->watchdog_ms = 200;
    cfg->home_timeout_ms = 30000;
    cfg->homing_speed_fraction = 0.25f;
    cfg->backoff_margin_rad = 0.02f;
    cfg->state_period_s = 0.010f;
    cfg->boot_delay_s = 0.050f;
    cfg->hard_stop_current_fraction = 0.9f;
    cfg->hard_stop_ms = 50;
    cfg->overcurrent_ms = 30;
    cfg->stall_ms = 300;
    cfg->soft_limit_overrun_rad = 0.05f;
    cfg->limit_switch_faults_when_active = false;
    cfg->require_calibration = true;
}

void sc_core_init(sc_core_t *core, const sc_core_config_t *cfg)
{
    memset(core, 0, sizeof(*core));
    core->cfg = *cfg;
    if (core->cfg.homing_speed_fraction <= 0.0f) core->cfg.homing_speed_fraction = 0.25f;
    if (core->cfg.backoff_margin_rad <= 0.0f) core->cfg.backoff_margin_rad = 0.02f;
    if (core->cfg.state_period_s <= 0.0f) core->cfg.state_period_s = 0.010f;
    if (core->cfg.hard_stop_current_fraction <= 0.0f) core->cfg.hard_stop_current_fraction = 0.9f;
    if (core->cfg.hard_stop_ms == 0) core->cfg.hard_stop_ms = 50;
    if (core->cfg.overcurrent_ms == 0) core->cfg.overcurrent_ms = 30;
    if (core->cfg.soft_limit_overrun_rad <= 0.0f) core->cfg.soft_limit_overrun_rad = 0.05f;
    if (core->cfg.home_timeout_ms == 0) core->cfg.home_timeout_ms = 30000;
    core->joint_count = cfg->joint_count > SC_CORE_MAX_JOINTS ? SC_CORE_MAX_JOINTS : cfg->joint_count;
    for (uint8_t i = 0; i < core->joint_count; ++i) {
        sc_joint_t *j = &core->joint[i];
        j->has_cal = cfg->has_calibration[i];
        j->cal = cfg->calibration[i];
        j->cal.index = i;
        stop_joint_control(j);
    }
    copy_str(core->robot_type, sizeof(core->robot_type), cfg->robot_type[0] ? cfg->robot_type : "unset");
    core->watchdog_ms = cfg->watchdog_ms;
    core->state = scorbot_v1_SystemState_SYSTEM_STATE_BOOT;
    core->fault = scorbot_v1_FaultCode_FAULT_NONE;
    core->last_dt_s = 0.001f;
    core->first_tick = true;
    core->current_scale = 1.0f;
}

/* ---- inbound -------------------------------------------------------------------- */

static void on_joint_command(sc_core_t *core, const scorbot_v1_JointCommand *cmd)
{
    if (core->state != scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE) {
        ++core->commands_ignored;
        return;
    }
    ++core->commands_received;
    core->since_command_s = 0.0f;
    for (uint8_t i = 0; i < core->joint_count; ++i) {
        sc_joint_t *j = &core->joint[i];
        const float sign = joint_sign(j);
        j->mode = i < cmd->mode_count ? cmd->mode[i] : scorbot_v1_ControlMode_CONTROL_MODE_NONE;
        j->setpoint_position = i < cmd->position_count ? sign * cmd->position[i] : j->position;
        j->setpoint_velocity = i < cmd->velocity_count ? sign * cmd->velocity[i] : 0.0f;
    }
}

static scorbot_v1_Result do_set_calibration(sc_core_t *core, const scorbot_v1_JointCalibration *cal, const char **msg)
{
    if (cal->index >= core->joint_count) {
        *msg = "joint index out of range";
        return scorbot_v1_Result_RESULT_INVALID_ARGUMENT;
    }
    if (drives_enabled(core)) {
        *msg = "cannot change calibration while moving";
        return scorbot_v1_Result_RESULT_INVALID_STATE;
    }
    if (cal->counts_per_motor_rev <= 0 || cal->gear_ratio <= 0.0f || cal->max_velocity_rad_s <= 0.0f ||
        cal->max_accel_rad_s2 <= 0.0f || cal->soft_limit_min_rad >= cal->soft_limit_max_rad ||
        (cal->home_direction != 1 && cal->home_direction != -1)) {
        *msg = "invalid calibration values";
        return scorbot_v1_Result_RESULT_INVALID_ARGUMENT;
    }
    sc_joint_t *j = &core->joint[cal->index];
    j->cal = *cal;
    j->has_cal = true;
    if (core->cfg.persist.calibration) {
        core->cfg.persist.calibration(core->cfg.persist.ctx, cal);
    }
    return scorbot_v1_Result_RESULT_OK;
}

static scorbot_v1_Result do_home(sc_core_t *core, uint32_t mask, uint32_t timeout_ms, const char **msg)
{
    switch (core->state) {
    case scorbot_v1_SystemState_SYSTEM_STATE_HOMING: *msg = "homing in progress"; return scorbot_v1_Result_RESULT_BUSY;
    case scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE: *msg = "disable before homing"; return scorbot_v1_Result_RESULT_INVALID_STATE;
    case scorbot_v1_SystemState_SYSTEM_STATE_FAULT: *msg = "clear the fault first"; return scorbot_v1_Result_RESULT_INVALID_STATE;
    case scorbot_v1_SystemState_SYSTEM_STATE_BOOT: *msg = "booting"; return scorbot_v1_Result_RESULT_BUSY;
    default: break;
    }
    const uint32_t all = (1u << core->joint_count) - 1u;
    if (mask == 0) {
        mask = all;
    }
    if (mask & ~all) {
        *msg = "joint mask names a joint that does not exist";
        return scorbot_v1_Result_RESULT_INVALID_ARGUMENT;
    }
    for (uint8_t i = 0; i < core->joint_count; ++i) {
        if ((mask & (1u << i)) && core->cfg.require_calibration && !core->joint[i].has_cal) {
            *msg = "no calibration for a requested joint";
            return scorbot_v1_Result_RESULT_INVALID_STATE;
        }
    }
    if (core->motor_power_seen) {
        *msg = "motor power is off";
        return scorbot_v1_Result_RESULT_INVALID_STATE;
    }
    core->homing_mask = mask;
    core->homed_progress_mask = 0;
    core->homing_elapsed_s = 0.0f;
    core->homing_timeout_s = (float)(timeout_ms ? timeout_ms : core->cfg.home_timeout_ms) / 1000.0f;
    for (uint8_t i = 0; i < core->joint_count; ++i) {
        sc_joint_t *j = &core->joint[i];
        if (!(mask & (1u << i))) {
            continue;
        }
        j->homing = true;
        j->homed = false;
        j->homing_direction = j->cal.home_direction >= 0 ? 1 : -1;
        j->homing_reversed = false;
        j->homing_passed = false;
        /* Already sitting on the switch (parked there by the last homing): back off until it
           releases and a margin further, then approach at homing speed so the edge is
           latched, not wherever we happen to be. */
        j->homing_backoff = j->limit_switch;
        j->homing_released = false;
        j->hard_stop_s = 0.0f;
        if (j->homing_backoff) {
            j->homing_direction = (int8_t)-j->homing_direction;
        }
    }
    enter_state(core, scorbot_v1_SystemState_SYSTEM_STATE_HOMING);
    return scorbot_v1_Result_RESULT_OK;
}

static scorbot_v1_Result do_enable(sc_core_t *core, bool allow_unhomed, const char **msg)
{
    switch (core->state) {
    case scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE: return scorbot_v1_Result_RESULT_OK;
    case scorbot_v1_SystemState_SYSTEM_STATE_READY: break;
    case scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED:
        if (!allow_unhomed) {
            *msg = "not homed";
            return scorbot_v1_Result_RESULT_INVALID_STATE;
        }
        break;
    case scorbot_v1_SystemState_SYSTEM_STATE_HOMING: *msg = "homing in progress"; return scorbot_v1_Result_RESULT_BUSY;
    case scorbot_v1_SystemState_SYSTEM_STATE_FAULT: *msg = "clear the fault first"; return scorbot_v1_Result_RESULT_INVALID_STATE;
    default: *msg = "booting"; return scorbot_v1_Result_RESULT_BUSY;
    }
    if (core->cfg.require_calibration) {
        for (uint8_t i = 0; i < core->joint_count; ++i) {
            if (!core->joint[i].has_cal) {
                *msg = "calibration not set for every joint";
                return scorbot_v1_Result_RESULT_INVALID_STATE;
            }
        }
    }
    if (core->motor_power_seen) {
        *msg = "motor power is off (e-stop?)";
        return scorbot_v1_Result_RESULT_INVALID_STATE;
    }
    core->unhomed_active = core->state == scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED;
    for (uint8_t i = 0; i < core->joint_count; ++i) {
        sc_joint_t *j = &core->joint[i];
        j->mode = scorbot_v1_ControlMode_CONTROL_MODE_NONE;
        j->setpoint_position = j->position;
        j->setpoint_velocity = 0.0f;
        j->v_ref = 0.0f;
        j->integral = 0.0f;
    }
    core->since_command_s = 0.0f;
    enter_state(core, scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE);
    return scorbot_v1_Result_RESULT_OK;
}

static scorbot_v1_Result do_disable(sc_core_t *core)
{
    if (drives_enabled(core)) {
        enter_state(core, positions_trusted(core) ? scorbot_v1_SystemState_SYSTEM_STATE_READY
                                                  : scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
    }
    return scorbot_v1_Result_RESULT_OK;
}

static scorbot_v1_Result do_clear_fault(sc_core_t *core)
{
    if (core->state != scorbot_v1_SystemState_SYSTEM_STATE_FAULT) {
        return scorbot_v1_Result_RESULT_OK;
    }
    const bool trusted = core->fault_trusted && positions_trusted(core);
    for (uint8_t i = 0; i < core->joint_count; ++i) {
        sc_joint_t *j = &core->joint[i];
        if (!trusted) {
            j->homed = false;
        }
        j->fault = false;
    }
    core->fault = scorbot_v1_FaultCode_FAULT_NONE;
    core->unhomed_active = false;
    enter_state(core, trusted ? scorbot_v1_SystemState_SYSTEM_STATE_READY : scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
    return scorbot_v1_Result_RESULT_OK;
}

static void on_request(sc_core_t *core, const scorbot_v1_Request *req)
{
    /* Events a request causes (state_changed on Enable, ...) are queued before the
       response, so the peer sees them first and never a seq gap. */
    scorbot_v1_Response resp;
    memset(&resp, 0, sizeof(resp));
    const char *msg = "";
    scorbot_v1_Result result = scorbot_v1_Result_RESULT_OK;

    switch (req->which_body) {
    case scorbot_v1_Request_ping_tag:
        resp.which_body = scorbot_v1_Response_pong_tag;
        resp.body.pong = req->body.ping.nonce;
        break;

    case scorbot_v1_Request_get_info_tag: {
        resp.which_body = scorbot_v1_Response_get_info_tag;
        scorbot_v1_GetInfoResponse *info = &resp.body.get_info;
        info->protocol_major = SC_PROTOCOL_MAJOR;
        info->protocol_minor = SC_PROTOCOL_MINOR;
        copy_str(info->firmware_version, sizeof(info->firmware_version), core->cfg.firmware_version);
        copy_str(info->robot_type, sizeof(info->robot_type), core->robot_type);
        info->joint_names_count = core->joint_count;
        for (uint8_t i = 0; i < core->joint_count; ++i) {
            copy_str(info->joint_names[i], sizeof(info->joint_names[i]), core->cfg.joint_names[i]);
        }
        info->boot_count = core->cfg.boot_count;
        info->watchdog_ms = core->watchdog_ms;
        info->calibration_count = 0;
        for (uint8_t i = 0; i < core->joint_count; ++i) {
            if (core->joint[i].has_cal) {
                info->calibration[info->calibration_count++] = core->joint[i].cal;
            }
        }
        break;
    }

    case scorbot_v1_Request_set_calibration_tag:
        if (!req->body.set_calibration.has_joint) {
            result = scorbot_v1_Result_RESULT_INVALID_ARGUMENT;
            msg = "missing joint calibration";
        } else {
            result = do_set_calibration(core, &req->body.set_calibration.joint, &msg);
        }
        break;

    case scorbot_v1_Request_set_robot_type_tag:
        copy_str(core->robot_type, sizeof(core->robot_type), req->body.set_robot_type.robot_type);
        if (core->cfg.persist.robot_type) {
            core->cfg.persist.robot_type(core->cfg.persist.ctx, core->robot_type);
        }
        break;

    case scorbot_v1_Request_set_watchdog_tag:
        core->watchdog_ms = req->body.set_watchdog.watchdog_ms;
        break;

    case scorbot_v1_Request_home_tag:
        result = do_home(core, req->body.home.joint_mask, req->body.home.timeout_ms, &msg);
        break;

    case scorbot_v1_Request_enable_tag:
        result = do_enable(core, req->body.enable.allow_unhomed, &msg);
        break;

    case scorbot_v1_Request_disable_tag:
        result = do_disable(core);
        break;

    case scorbot_v1_Request_clear_fault_tag:
        result = do_clear_fault(core);
        break;

    default:
        result = scorbot_v1_Result_RESULT_UNSUPPORTED;
        msg = "unknown request";
        break;
    }

    scorbot_v1_Envelope *e = new_envelope(core);
    e->which_payload = scorbot_v1_Envelope_response_tag;
    scorbot_v1_Response *out = &e->payload.response;
    *out = resp;
    out->request_id = req->request_id;
    out->result = result;
    copy_str(out->message, sizeof(out->message), msg);
}

void sc_core_handle(sc_core_t *core, const scorbot_v1_Envelope *in)
{
    switch (in->which_payload) {
    case scorbot_v1_Envelope_joint_command_tag: on_joint_command(core, &in->payload.joint_command); break;
    case scorbot_v1_Envelope_request_tag: on_request(core, &in->payload.request); break;
    default: break; /* states, responses and events from a peer are ignored */
    }
}

/* ---- measurement ---------------------------------------------------------------- */

static void measure(sc_core_t *core, float dt, const sc_input_t *in, uint32_t *switch_edges)
{
    for (uint8_t i = 0; i < core->joint_count; ++i) {
        sc_joint_t *j = &core->joint[i];
        const sc_joint_input_t *ji = &in->joint[i];
        j->raw_counts = ji->counts;
        j->corrected_counts = j->cal.encoder_invert ? -ji->counts : ji->counts;
        if (core->first_tick) {
            j->zero_counts = j->corrected_counts; /* reads 0 at boot */
            for (int k = 0; k < SC_CORE_VEL_WINDOW; ++k) {
                j->history[k] = j->corrected_counts;
            }
            j->history_filled = SC_CORE_VEL_WINDOW;
        }
        const float cpr = j->has_cal || j->cal.counts_per_motor_rev > 0 ? counts_per_rad(&j->cal) : 0.0f;
        const float scale = cpr > 0.0f ? 1.0f / cpr : 0.0f;
        j->position = (float)(j->corrected_counts - j->zero_counts) * scale + (j->homed ? j->cal.home_offset_rad : 0.0f);

        /* Velocity: counts over the last SC_CORE_VEL_WINDOW ticks, then a light IIR. */
        const int32_t oldest = j->history[j->history_head];
        j->history[j->history_head] = j->corrected_counts;
        j->history_head = (uint8_t)((j->history_head + 1) % SC_CORE_VEL_WINDOW);
        const float window_s = dt * (float)SC_CORE_VEL_WINDOW;
        const float raw_v = (float)(j->corrected_counts - oldest) * scale / (window_s > 0.0f ? window_s : 1.0f);
        j->velocity += 0.5f * (raw_v - j->velocity);

        j->current = ji->current_a < 0.0f ? -ji->current_a : ji->current_a;
        if (ji->limit_switch != j->limit_switch) {
            *switch_edges |= (1u << i);
        }
        j->limit_switch = ji->limit_switch;
    }
}

/* ---- homing --------------------------------------------------------------------- */

static void finish_homing_joint(sc_core_t *core, uint8_t i, const sc_joint_input_t *ji)
{
    sc_joint_t *j = &core->joint[i];
    /* The count latched in the switch interrupt is the edge; the position a tick later
       (with the motor still braking) is not. */
    const int32_t edge = ji->switch_edge ? (j->cal.encoder_invert ? -ji->latched_counts : ji->latched_counts)
                                         : j->corrected_counts;
    j->zero_counts = edge;
    j->homed = true;
    j->homing = false;
    j->position = (float)(j->corrected_counts - j->zero_counts) / counts_per_rad(&j->cal) + j->cal.home_offset_rad;
    stop_joint_control(j); /* hard brake on the edge */
    core->homed_progress_mask |= (1u << i);
    emit_event_u32(core, scorbot_v1_Event_homing_progress_mask_tag, core->homed_progress_mask);
}

static void step_homing(sc_core_t *core, float dt, const sc_input_t *in)
{
    core->homing_elapsed_s += dt;
    bool all_done = true;
    for (uint8_t i = 0; i < core->joint_count; ++i) {
        sc_joint_t *j = &core->joint[i];
        if (!j->homing) {
            continue;
        }
        all_done = false;
        j->mode = scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY;
        j->setpoint_velocity = (float)j->homing_direction * core->cfg.homing_speed_fraction * j->cal.max_velocity_rad_s;
        j->brake = false;

        const int8_t home_dir = j->cal.home_direction >= 0 ? 1 : -1;
        if (j->homing_backoff) {
            if (!j->homing_released) {
                if (j->limit_switch) {
                    continue; /* still pressed: keep backing off */
                }
                j->homing_released = true;
                j->homing_release_pos = j->position;
            }
            if (fabsf(j->position - j->homing_release_pos) < core->cfg.backoff_margin_rad) {
                continue; /* a definite distance past the release point */
            }
            j->homing_backoff = false;
            j->homing_direction = home_dir;
            continue;
        }
        if (j->limit_switch && j->homing_direction != home_dir) {
            /* Reversed pass: crossing the switch from its far side. Drive through and
               re-approach from the calibrated direction. */
            j->homing_passed = true;
            continue;
        }
        if (j->homing_passed && !j->limit_switch && j->homing_direction != home_dir) {
            j->homing_direction = home_dir; /* final approach */
            j->homing_passed = false;
            continue;
        }
        if (j->limit_switch) {
            finish_homing_joint(core, i, &in->joint[i]);
            continue;
        }
        /* Hard stop without finding the switch: the current rises (or the motor stops
           moving at full duty). Search the other way, once. */
        const float imax = j->cal.current_limit_a > 0.0f ? j->cal.current_limit_a : 1e9f;
        const bool pushing = j->current > core->cfg.hard_stop_current_fraction * imax ||
                             (fabsf(j->duty) > 0.8f && fabsf(j->velocity) < 0.05f * j->cal.max_velocity_rad_s);
        if (j->hard_stop_s < 0.0f) {
            /* Grace period right after a reversal: the motor is still leaning on the stop
               while the reference turns around, which must not count as a second stop. */
            j->hard_stop_s += dt;
            if (j->hard_stop_s > 0.0f) {
                j->hard_stop_s = 0.0f;
            }
            continue;
        }
        j->hard_stop_s = pushing ? j->hard_stop_s + dt : 0.0f;
        if (j->hard_stop_s * 1000.0f >= (float)core->cfg.hard_stop_ms) {
            if (j->homing_reversed) {
                raise_fault(core, scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT, i);
                return;
            }
            j->homing_reversed = true;
            j->homing_direction = (int8_t)-j->homing_direction;
            j->v_ref = 0.0f; /* the motor is stalled anyway; ramp up fresh the other way */
            j->integral = 0.0f;
            j->hard_stop_s = -0.3f;
        }
    }
    if (all_done) {
        emit_event_u32(core, scorbot_v1_Event_homing_complete_mask_tag, core->homed_progress_mask);
        enter_state(core, positions_trusted(core) ? scorbot_v1_SystemState_SYSTEM_STATE_READY
                                                  : scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
        return;
    }
    if (core->homing_elapsed_s > core->homing_timeout_s) {
        raise_fault(core, scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT, -1);
    }
}

/* ---- control law ---------------------------------------------------------------- */

static void step_joints(sc_core_t *core, float dt, const sc_input_t *in, sc_output_t *out)
{
    const bool enabled = drives_enabled(core);
    const bool enforce_soft = enabled && !core->unhomed_active && core->state != scorbot_v1_SystemState_SYSTEM_STATE_HOMING;

    /* Summed-current cap: the fuse and the rail copper see the sum of every motor. An
       integral limiter winds one factor down while the measured sum is over the limit
       and back up while it is under; the factor multiplies every joint's velocity
       demand, so the loops keep tracking (slower) and a joint pushing against something
       has its demand, and with it its current, wound down along with the rest. Scaling
       the duties instead would not work: the velocity loop compensates it. */
    float scale = 1.0f;
    if (core->cfg.total_current_limit_a > 0.0f) {
        const float cap = core->cfg.total_current_limit_a;
        float sum = 0.0f;
        for (uint8_t i = 0; i < core->joint_count; ++i) {
            sum += core->joint[i].current;
        }
        scale = clampf(core->current_scale + (cap - sum) / cap * dt / SC_CURRENT_CAP_TAU_S, SC_CURRENT_CAP_MIN_SCALE, 1.0f);
    }
    core->current_scale = scale;

    for (uint8_t i = 0; i < core->joint_count; ++i) {
        sc_joint_t *j = &core->joint[i];
        const float vmax = j->cal.max_velocity_rad_s > 0.0f ? j->cal.max_velocity_rad_s : 0.0f;
        const float amax = j->cal.max_accel_rad_s2 > 0.0f ? j->cal.max_accel_rad_s2 : 1.0f;
        j->at_soft_limit = false;

        if (!enabled) {
            stop_joint_control(j);
            out->duty[i] = 0.0f;
            out->brake[i] = true;
            continue;
        }
        j->brake = false;

        /* Reference generation, exactly as the simulator does it. */
        float v_des = 0.0f;
        switch (j->mode) {
        case scorbot_v1_ControlMode_CONTROL_MODE_POSITION: {
            float target = j->setpoint_position;
            if (enforce_soft) {
                const float clamped = clampf(target, j->cal.soft_limit_min_rad, j->cal.soft_limit_max_rad);
                j->at_soft_limit = clamped != target;
                target = clamped;
            }
            v_des = clampf(10.0f * (target - j->position), -vmax, vmax);
            break;
        }
        case scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY:
            v_des = clampf(j->setpoint_velocity, -vmax, vmax);
            break;
        default:
            v_des = 0.0f;
            break;
        }
        v_des *= scale; /* the summed-current cap */
        if (enforce_soft) {
            /* Never exceed the speed that can still stop before the limit. */
            const float room_up = j->cal.soft_limit_max_rad - j->position;
            const float room_dn = j->position - j->cal.soft_limit_min_rad;
            const float cap_up = room_up > 0.0f ? sqrtf(2.0f * amax * room_up) : 0.0f;
            const float cap_dn = room_dn > 0.0f ? sqrtf(2.0f * amax * room_dn) : 0.0f;
            if (v_des > cap_up) { v_des = cap_up; j->at_soft_limit = true; }
            if (v_des < -cap_dn) { v_des = -cap_dn; j->at_soft_limit = true; }
            if (j->position > j->cal.soft_limit_max_rad + core->cfg.soft_limit_overrun_rad ||
                j->position < j->cal.soft_limit_min_rad - core->cfg.soft_limit_overrun_rad) {
                raise_fault(core, scorbot_v1_FaultCode_FAULT_SOFT_LIMIT, i);
                return;
            }
        }
        const float dv = clampf(v_des - j->v_ref, -amax * dt, amax * dt);
        j->v_ref += dv;

        /* Inner loop: feedforward + PI on the velocity error, duty saturates at +-1. */
        const sc_gains_t g = effective_gains(core, i);
        const float err = j->v_ref - j->velocity;
        float duty = g.kff * j->v_ref + g.kp * err + g.ki * j->integral;
        if (duty > 1.0f) {
            duty = 1.0f;
        } else if (duty < -1.0f) {
            duty = -1.0f;
        } else {
            j->integral += err * dt; /* anti-windup: integrate only while unsaturated */
        }
        if (j->mode == scorbot_v1_ControlMode_CONTROL_MODE_NONE && fabsf(j->v_ref) < 1e-6f) {
            duty = 0.0f;
            j->integral = 0.0f;
            j->brake = true;
        }
        j->duty = duty;
        out->duty[i] = duty;
        out->brake[i] = j->brake;
        if (j->brake) {
            out->duty[i] = 0.0f;
        }

        /* Protection while ACTIVE. Homing uses the current as its hard-stop detector. */
        if (core->state == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE) {
            const float imax = j->cal.current_limit_a;
            if (imax > 0.0f && j->current > imax) {
                j->over_current_s += dt;
                if (j->over_current_s * 1000.0f >= (float)core->cfg.overcurrent_ms) {
                    raise_fault(core, scorbot_v1_FaultCode_FAULT_OVERCURRENT, i);
                    return;
                }
            } else {
                j->over_current_s = 0.0f;
            }
            if (core->cfg.stall_ms && fabsf(duty) > 0.9f && fabsf(j->velocity) < 0.02f * vmax) {
                j->stall_s += dt;
                if (j->stall_s * 1000.0f >= (float)core->cfg.stall_ms) {
                    raise_fault(core, scorbot_v1_FaultCode_FAULT_STALL, i);
                    return;
                }
            } else {
                j->stall_s = 0.0f;
            }
            if (core->cfg.limit_switch_faults_when_active && j->limit_switch) {
                raise_fault(core, scorbot_v1_FaultCode_FAULT_LIMIT_SWITCH, i);
                return;
            }
        }
    }
}

/* ---- tick ----------------------------------------------------------------------- */

void sc_core_tick(sc_core_t *core, float dt, const sc_input_t *in, sc_output_t *out)
{
    memset(out, 0, sizeof(*out));
    if (dt <= 0.0f) {
        out->drives_enabled = drives_enabled(core);
        return;
    }
    core->last_dt_s = dt;
    core->now_us += (uint64_t)(dt * 1e6f);

    uint32_t switch_edges = 0;
    measure(core, dt, in, &switch_edges);
    core->first_tick = false;

    /* Hardware faults reported by the glue. */
    for (uint8_t i = 0; i < core->joint_count; ++i) {
        if (in->joint[i].driver_fault) {
            raise_fault(core, scorbot_v1_FaultCode_FAULT_DRIVER, i);
        }
    }
    /* Motor rail lost while the drives are on: the e-stop. Positions are still good, so
       ClearFault then Enable resumes without homing. Idle states just remember it so
       Enable/Home can refuse. */
    core->motor_power_seen = in->motor_power_valid && !in->motor_power;
    if (core->motor_power_seen && drives_enabled(core)) {
        sc_core_raise_fault(core, scorbot_v1_FaultCode_FAULT_DRIVER, -1, true);
    }

    if (core->state == scorbot_v1_SystemState_SYSTEM_STATE_BOOT) {
        core->boot_elapsed_s += dt;
        if (core->boot_elapsed_s >= core->cfg.boot_delay_s) {
            enter_state(core, scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED);
        }
    }

    if (core->state == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE && core->watchdog_ms > 0) {
        core->since_command_s += dt;
        if (core->since_command_s * 1000.0f > (float)core->watchdog_ms) {
            raise_fault(core, scorbot_v1_FaultCode_FAULT_WATCHDOG, -1);
        }
    }

    if (core->state == scorbot_v1_SystemState_SYSTEM_STATE_HOMING) {
        step_homing(core, dt, in);
    }

    step_joints(core, dt, in, out);
    out->drives_enabled = drives_enabled(core);
    if (!out->drives_enabled) {
        for (uint8_t i = 0; i < core->joint_count; ++i) {
            out->duty[i] = 0.0f;
            out->brake[i] = true;
        }
    }

    if (switch_edges) {
        emit_event_u32(core, scorbot_v1_Event_limit_switch_mask_tag, switch_edges);
    }

    core->since_state_s += dt;
    if (core->since_state_s + 1e-6f >= core->cfg.state_period_s) {
        core->since_state_s = 0.0f;
        emit_joint_state(core);
    }
}

/* ---- names ---------------------------------------------------------------------- */

const char *sc_state_name(scorbot_v1_SystemState s)
{
    switch (s) {
    case scorbot_v1_SystemState_SYSTEM_STATE_BOOT: return "BOOT";
    case scorbot_v1_SystemState_SYSTEM_STATE_UNHOMED: return "UNHOMED";
    case scorbot_v1_SystemState_SYSTEM_STATE_HOMING: return "HOMING";
    case scorbot_v1_SystemState_SYSTEM_STATE_READY: return "READY";
    case scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE: return "ACTIVE";
    case scorbot_v1_SystemState_SYSTEM_STATE_FAULT: return "FAULT";
    default: return "?";
    }
}

const char *sc_fault_name(scorbot_v1_FaultCode f)
{
    switch (f) {
    case scorbot_v1_FaultCode_FAULT_NONE: return "NONE";
    case scorbot_v1_FaultCode_FAULT_WATCHDOG: return "WATCHDOG";
    case scorbot_v1_FaultCode_FAULT_OVERCURRENT: return "OVERCURRENT";
    case scorbot_v1_FaultCode_FAULT_LIMIT_SWITCH: return "LIMIT_SWITCH";
    case scorbot_v1_FaultCode_FAULT_SOFT_LIMIT: return "SOFT_LIMIT";
    case scorbot_v1_FaultCode_FAULT_STALL: return "STALL";
    case scorbot_v1_FaultCode_FAULT_HOMING_TIMEOUT: return "HOMING_TIMEOUT";
    case scorbot_v1_FaultCode_FAULT_ENCODER: return "ENCODER";
    case scorbot_v1_FaultCode_FAULT_DRIVER: return "DRIVER";
    case scorbot_v1_FaultCode_FAULT_INTERNAL: return "INTERNAL";
    default: return "?";
    }
}
