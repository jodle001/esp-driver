// The Eshed dialect front-end. See acl.h, and design doc section 7 for the spec it is
// written from (the operator's acl_private.hpp and erv.cpp). Where the real box's
// behaviour is not known from the operator's code, the choice is noted as ASSUMED and
// is what the Bingo capture must settle.
#include "scorbot_core/acl.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI_F 3.14159265358979f
#define DEG (PI_F / 180.0f)

/* ---- output ---------------------------------------------------------------------- */

static void put_bytes(sc_acl_t *a, const char *s, size_t n)
{
    for (size_t k = 0; k < n; ++k) {
        char c = s[k];
        if (c == 0x11 || c == 0x13) {
            c = '?'; /* never XON/XOFF: the operator's port has software flow control on */
        }
        if (a->out_len == SC_ACL_OUT_MAX) {
            ++a->out_dropped;
            return;
        }
        a->out[(a->out_head + a->out_len) % SC_ACL_OUT_MAX] = (uint8_t)c;
        ++a->out_len;
    }
}

static void put(sc_acl_t *a, const char *s) { put_bytes(a, s, strlen(s)); }

/* ASSUMED: the box ends every reply with CR LF and a '>' on its own; the operator only
   needs the '>' (any of CR, LF, NUL and '>' ends a line for it). */
static void prompt(sc_acl_t *a)
{
    put(a, "\r\n>");
    ++a->prompts;
}

static void line_out(sc_acl_t *a, const char *s)
{
    put(a, "\r\n");
    put(a, s);
    put(a, "\r\n");
}

size_t sc_acl_read(sc_acl_t *a, uint8_t *out, size_t cap)
{
    size_t n = 0;
    while (n < cap && a->out_len > 0) {
        out[n++] = a->out[a->out_head];
        a->out_head = (uint16_t)((a->out_head + 1) % SC_ACL_OUT_MAX);
        --a->out_len;
    }
    return n;
}

size_t sc_acl_pending(const sc_acl_t *a) { return a->out_len; }

/* ---- units ------------------------------------------------------------------------- */

static float reported_position(const sc_acl_t *a, uint8_t i)
{
    const sc_joint_t *j = &a->core->joint[i];
    return (j->cal.invert ? -1.0f : 1.0f) * j->position;
}

static float vcpr(const sc_acl_t *a, uint8_t i)
{
    float c = a->cfg.virtual_counts_per_rad[i];
    if (c <= 0.0f) {
        const scorbot_v1_JointCalibration *cal = &a->core->joint[i].cal;
        c = (float)cal->counts_per_motor_rev * cal->gear_ratio / (2.0f * PI_F);
    }
    return c > 0.0f ? c : 1.0f;
}

static float vsign(const sc_acl_t *a, uint8_t i) { return a->cfg.virtual_sign[i] < 0.0f ? -1.0f : 1.0f; }

int32_t sc_acl_virtual_counts(const sc_acl_t *a, uint8_t joint)
{
    if (joint >= a->core->joint_count) {
        return 0;
    }
    return (int32_t)lrintf(reported_position(a, joint) * vsign(a, joint) * vcpr(a, joint));
}

static float counts_to_rad(const sc_acl_t *a, uint8_t i, int32_t counts)
{
    return (float)counts / (vsign(a, i) * vcpr(a, i));
}

int sc_acl_joint_for_axis(const sc_acl_t *a, int axis)
{
    if (axis >= 1 && axis <= 5 && axis <= a->core->joint_count) {
        return axis - 1;
    }
    if (axis == 6 && a->core->joint_count >= 7) {
        return 5; /* the gripper, when the board carries all seven channels (rev 0.4) */
    }
    if (axis == 7 && a->core->joint_count >= 6) {
        return a->core->joint_count - 1; /* the slide: the Eshed's peripheral axis, always the last channel */
    }
    return -1;
}

static float vmax(const sc_acl_t *a, uint8_t i)
{
    const float v = a->core->joint[i].cal.max_velocity_rad_s;
    return v > 0.0f ? v : 1.0f;
}

/* ---- the core, driven ---------------------------------------------------------------- */

static scorbot_v1_Result request(sc_acl_t *a, scorbot_v1_Envelope *e, const char **msg)
{
    static char message[64];
    e->which_payload = scorbot_v1_Envelope_request_tag;
    e->payload.request.request_id = ++a->next_request_id;
    sc_core_handle(a->core, e);
    /* The response is queued at once; drain everything, keep the one for this id. */
    scorbot_v1_Envelope out;
    scorbot_v1_Result result = scorbot_v1_Result_RESULT_INTERNAL_ERROR;
    message[0] = '\0';
    while (sc_core_pop(a->core, &out)) {
        if (out.which_payload == scorbot_v1_Envelope_response_tag &&
            out.payload.response.request_id == e->payload.request.request_id) {
            result = out.payload.response.result;
            strncpy(message, out.payload.response.message, sizeof(message) - 1);
        }
    }
    if (msg) {
        *msg = message;
    }
    return result;
}

static scorbot_v1_Result req_simple(sc_acl_t *a, pb_size_t tag, const char **msg)
{
    scorbot_v1_Envelope e;
    memset(&e, 0, sizeof(e));
    e.payload.request.which_body = tag;
    return request(a, &e, msg);
}

static scorbot_v1_Result req_enable(sc_acl_t *a, bool allow_unhomed, const char **msg)
{
    scorbot_v1_Envelope e;
    memset(&e, 0, sizeof(e));
    e.payload.request.which_body = scorbot_v1_Request_enable_tag;
    e.payload.request.body.enable.allow_unhomed = allow_unhomed;
    return request(a, &e, msg);
}

static scorbot_v1_Result req_home(sc_acl_t *a, const char **msg)
{
    scorbot_v1_Envelope e;
    memset(&e, 0, sizeof(e));
    e.payload.request.which_body = scorbot_v1_Request_home_tag;
    e.payload.request.body.home.joint_mask = 0; /* every joint */
    e.payload.request.body.home.timeout_ms = a->cfg.home_timeout_ms;
    return request(a, &e, msg);
}

static void stop_motion(sc_acl_t *a)
{
    a->moving = false;
    a->moved_pending = false;
    a->jog_joint = -1;
    a->jog_hold_s = 0.0f;
    for (uint8_t i = 0; i < a->core->joint_count; ++i) {
        a->hold[i] = false;
    }
}

static bool core_active(const sc_acl_t *a) { return a->core->state == scorbot_v1_SystemState_SYSTEM_STATE_ACTIVE; }

/* The e-stop: the rail is off (the core saw it), or the core faulted on losing it. */
static bool power_off(const sc_acl_t *a) { return a->core->motor_power_seen; }

/// Bring the core to ACTIVE for a move or a jog. False (with the line already written)
/// if that is not possible: the MOTORS ON chain is open, or the core refused.
static bool make_active(sc_acl_t *a)
{
    if (core_active(a)) {
        return true;
    }
    if (a->core->state == scorbot_v1_SystemState_SYSTEM_STATE_FAULT) {
        line_out(a, "CONTROL DISABLED (fault latched: CON to clear)");
        return false;
    }
    if (power_off(a)) {
        line_out(a, "CONTROL DISABLED (motor power off: press MOTORS ON)");
        return false;
    }
    const char *msg = "";
    /* ASSUMED: the box moves unhomed too (its counts are just wrong then); the operator
       always homes first, and the core has no soft limits while unhomed. */
    if (req_enable(a, true, &msg) != scorbot_v1_Result_RESULT_OK) {
        char text[96];
        snprintf(text, sizeof(text), "CONTROL DISABLED (%s)", msg);
        line_out(a, text);
        return false;
    }
    return true;
}

/* ---- commands ------------------------------------------------------------------------ */

static void capture_target(sc_acl_t *a)
{
    for (uint8_t i = 0; i < a->core->joint_count; ++i) {
        a->target[i] = sc_acl_virtual_counts(a, i);
    }
    a->target_defined = true;
}

static void cmd_home(sc_acl_t *a)
{
    stop_motion(a);
    if (a->core->state == scorbot_v1_SystemState_SYSTEM_STATE_FAULT) {
        req_simple(a, scorbot_v1_Request_clear_fault_tag, NULL);
        a->last_fault = scorbot_v1_FaultCode_FAULT_NONE;
    }
    if (core_active(a)) {
        req_simple(a, scorbot_v1_Request_disable_tag, NULL);
    }
    if (power_off(a)) {
        line_out(a, "CONTROL DISABLED (motor power off: press MOTORS ON)");
        prompt(a);
        return;
    }
    const char *msg = "";
    if (req_home(a, &msg) != scorbot_v1_Result_RESULT_OK) {
        char text[96];
        snprintf(text, sizeof(text), "CONTROL DISABLED (%s)", msg);
        line_out(a, text);
        prompt(a);
        return;
    }
    /* ASSUMED: HOME blocks on the real box (the operator queues HERE 200 ms after the
       prompt, which only makes sense if the prompt comes after the cycle). */
    a->homing_pending = true;
}

static void cmd_move(sc_acl_t *a, bool wait)
{
    if (!a->target_defined) {
        capture_target(a);
    }
    if (!make_active(a)) {
        prompt(a);
        return;
    }
    a->jog_joint = -1;
    a->jog_hold_s = 0.0f;
    a->moving = true;
    a->moved_pending = wait;
    if (!wait) {
        prompt(a);
    }
}

static void cmd_con(sc_acl_t *a)
{
    if (a->core->state == scorbot_v1_SystemState_SYSTEM_STATE_FAULT) {
        req_simple(a, scorbot_v1_Request_clear_fault_tag, NULL);
        a->last_fault = scorbot_v1_FaultCode_FAULT_NONE;
        stop_motion(a);
    }
    if (power_off(a)) {
        /* Decided: software never restarts the motors after an e-stop. */
        line_out(a, "CONTROL DISABLED (motor power off: press MOTORS ON)");
        prompt(a);
        return;
    }
    if (!core_active(a)) {
        const char *msg = "";
        if (req_enable(a, true, &msg) != scorbot_v1_Result_RESULT_OK) {
            char text[96];
            snprintf(text, sizeof(text), "CONTROL DISABLED (%s)", msg);
            line_out(a, text);
        }
    }
    prompt(a);
}

static void cmd_listpv(sc_acl_t *a)
{
    /* The operator parses " %*d:%ld" five times from the start of a line, so the pairs
       lead the line; the gripper (axis 6, when fitted) and the slide (axis 7) follow and are
       ignored by it. */
    char text[128];
    int n = 0;
    const uint8_t joints = a->core->joint_count < 5 ? a->core->joint_count : 5;
    for (uint8_t i = 0; i < joints; ++i) {
        n += snprintf(text + n, sizeof(text) - (size_t)n, "%s%u:%ld", i ? " " : "", (unsigned)(i + 1),
                      (long)sc_acl_virtual_counts(a, i));
    }
    if (a->core->joint_count >= 7) {
        n += snprintf(text + n, sizeof(text) - (size_t)n, " 6:%ld", (long)sc_acl_virtual_counts(a, 5)); /* the gripper */
    }
    if (a->core->joint_count >= 6) {
        n += snprintf(text + n, sizeof(text) - (size_t)n, " 7:%ld", (long)sc_acl_virtual_counts(a, (uint8_t)(a->core->joint_count - 1)));
    }
    line_out(a, text);
}

static void toggle_manual(sc_acl_t *a)
{
    if (a->manual) {
        a->manual = false;
        stop_motion(a);
        prompt(a); /* the operator waits for this before its next queued command */
        return;
    }
    a->moving = false;
    a->moved_pending = false;
    if (!make_active(a)) {
        prompt(a);
        return; /* stays in direct mode: the keys would do nothing anyway */
    }
    a->manual = true;
    a->jog_joint = -1;
    prompt(a); /* ASSUMED harmless: the operator ignores it while manual */
}

static void execute(sc_acl_t *a, char *line)
{
    ++a->lines;
    /* tokens, upper-cased: ACL is case-insensitive (the operator sends "clrbuf") */
    char *tok[6];
    int ntok = 0;
    for (char *p = strtok(line, " \t"); p && ntok < 6; p = strtok(NULL, " \t")) {
        for (char *q = p; *q; ++q) {
            *q = (char)toupper((unsigned char)*q);
        }
        tok[ntok++] = p;
    }
    if (ntok == 0) {
        prompt(a); /* a bare CR: the operator's flush, wants a prompt */
        return;
    }
    const char *c = tok[0];
    if (strcmp(c, "DEFP") == 0) {
        capture_target(a);
    } else if (strcmp(c, "HERE") == 0) {
        capture_target(a);
    } else if (strcmp(c, "SHIFT") == 0 && ntok >= 5 && strcmp(tok[2], "BY") == 0) {
        const int joint = sc_acl_joint_for_axis(a, atoi(tok[3]));
        if (joint < 0) {
            line_out(a, "ACL - unrecognized request (axis)");
        } else {
            if (!a->target_defined) {
                capture_target(a);
            }
            a->target[joint] += (int32_t)atol(tok[4]);
        }
    } else if (strcmp(c, "HOME") == 0) {
        cmd_home(a);
        return; /* the prompt follows the cycle */
    } else if (strcmp(c, "CLRBUF") == 0) {
        stop_motion(a);
    } else if (strcmp(c, "MOVE") == 0) {
        cmd_move(a, false);
        return; /* prompted inside */
    } else if (strcmp(c, "MOVED") == 0) {
        cmd_move(a, true);
        return;
    } else if (strcmp(c, "SPEED") == 0 && ntok >= 2) {
        const int pct = atoi(tok[1]);
        if (pct >= 1 && pct <= 100) {
            a->speed_percent = (uint8_t)pct;
        } else {
            line_out(a, "ACL - unrecognized request (speed)");
        }
    } else if (strcmp(c, "CON") == 0) {
        cmd_con(a);
        return;
    } else if (strcmp(c, "COFF") == 0) {
        stop_motion(a);
        req_simple(a, scorbot_v1_Request_disable_tag, NULL);
    } else if (strcmp(c, "LISTPV") == 0) {
        cmd_listpv(a);
    } else {
        ++a->unknown_lines;
        line_out(a, "ACL - unrecognized request");
    }
    prompt(a);
}

/* ---- input ---------------------------------------------------------------------------- */

static bool jog_key(sc_acl_t *a, char c)
{
    /* The pendant's keys, as acl_private.hpp names them. */
    static const struct { char key; int axis; int8_t dir; } keys[] = {
        {'1', 1, -1}, {'q', 1, +1}, {'w', 2, -1}, {'2', 2, +1}, {'e', 3, -1},
        {'3', 3, +1}, {'r', 4, -1}, {'4', 4, +1}, {'5', 5, -1}, {'t', 5, +1},
    };
    for (size_t k = 0; k < sizeof(keys) / sizeof(keys[0]); ++k) {
        if (keys[k].key == c) {
            const int joint = sc_acl_joint_for_axis(a, keys[k].axis);
            if (joint < 0) {
                return true;
            }
            a->jog_joint = (int8_t)joint;
            a->jog_dir = keys[k].dir;
            a->jog_hold_s = (float)a->cfg.jog_hold_ms / 1000.0f;
            ++a->jog_bytes;
            return true;
        }
    }
    return false;
}

void sc_acl_feed(sc_acl_t *a, const uint8_t *data, size_t n)
{
    for (size_t k = 0; k < n; ++k) {
        const char c = (char)data[k];
        if (c == 0x03) { /* Ctrl-C: whatever was pending is cancelled, give a prompt */
            a->moved_pending = false;
            prompt(a);
            continue;
        }
        if (c == '~') {
            a->line_len = 0;
            toggle_manual(a);
            continue;
        }
        if (a->manual) {
            jog_key(a, c); /* anything else in manual mode is ignored */
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (c == '\n' && a->line_len == 0) {
                continue; /* the LF of a CR LF pair */
            }
            a->line[a->line_len] = '\0';
            if (a->line_overflow) {
                ++a->unknown_lines;
                line_out(a, "ACL - unrecognized request");
                prompt(a);
            } else {
                execute(a, a->line);
            }
            a->line_len = 0;
            a->line_overflow = false;
            continue;
        }
        if (a->line_len < SC_ACL_LINE_MAX - 1) {
            a->line[a->line_len++] = c;
        } else {
            a->line_overflow = true;
        }
    }
}

/* ---- per tick ------------------------------------------------------------------------- */

static const char *fault_text(scorbot_v1_FaultCode f, int joint, char *buf, size_t cap)
{
    const char *kind;
    switch (f) {
    case scorbot_v1_FaultCode_FAULT_OVERCURRENT:
    case scorbot_v1_FaultCode_FAULT_STALL:
    case scorbot_v1_FaultCode_FAULT_SOFT_LIMIT:
    case scorbot_v1_FaultCode_FAULT_LIMIT_SWITCH:
        kind = "IMPACT"; /* the box's collision trip: something stopped the arm */
        break;
    case scorbot_v1_FaultCode_FAULT_DRIVER:
    case scorbot_v1_FaultCode_FAULT_WATCHDOG:
        kind = "CONTROL DISABLED"; /* the drivers are off: e-stop, a driver fault, a lost link */
        break;
    default:
        kind = "FAULT";
        break;
    }
    if (joint >= 0) {
        snprintf(buf, cap, "%s (%s on axis %d)", kind, sc_fault_name(f), joint + 1);
    } else {
        snprintf(buf, cap, "%s (%s)", kind, sc_fault_name(f));
    }
    return buf;
}

static void send_command(sc_acl_t *a, float dt)
{
    scorbot_v1_Envelope e;
    memset(&e, 0, sizeof(e));
    e.which_payload = scorbot_v1_Envelope_joint_command_tag;
    scorbot_v1_JointCommand *cmd = &e.payload.joint_command;
    const uint8_t n = a->core->joint_count;
    cmd->mode_count = cmd->position_count = cmd->velocity_count = n;
    bool all_reached = true;
    const float vcap_fraction = (float)a->speed_percent / 100.0f;

    if (a->jog_hold_s > 0.0f) {
        a->jog_hold_s -= dt;
        if (a->jog_hold_s <= 0.0f) {
            a->jog_hold_s = 0.0f;
            a->jog_joint = -1; /* the key stopped coming: stop */
        }
    }

    for (uint8_t i = 0; i < n; ++i) {
        cmd->mode[i] = scorbot_v1_ControlMode_CONTROL_MODE_NONE;
        if (a->jog_joint == (int8_t)i && a->jog_hold_s > 0.0f) {
            cmd->mode[i] = scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY;
            cmd->velocity[i] = (float)a->jog_dir * vsign(a, i) * a->cfg.manual_speed_fraction * vmax(a, i);
            a->hold[i] = false;
            continue;
        }
        if (a->moving) {
            const float target_rad = counts_to_rad(a, i, a->target[i]);
            const float err = target_rad - reported_position(a, i);
            if (fabsf(err) * vcpr(a, i) <= a->cfg.move_tolerance_counts) {
                a->hold[i] = true;
                a->hold_position[i] = target_rad;
            } else {
                all_reached = false;
                const float cap = vcap_fraction * vmax(a, i);
                float v = a->cfg.position_gain * err;
                if (v > cap) v = cap;
                if (v < -cap) v = -cap;
                cmd->mode[i] = scorbot_v1_ControlMode_CONTROL_MODE_VELOCITY;
                cmd->velocity[i] = v;
                continue;
            }
        }
        if (a->hold[i]) {
            cmd->mode[i] = scorbot_v1_ControlMode_CONTROL_MODE_POSITION;
            cmd->position[i] = a->hold_position[i];
        }
    }
    if (a->moving && all_reached) {
        a->moving = false;
        if (a->moved_pending) {
            a->moved_pending = false;
            prompt(a);
        }
    }
    sc_core_handle(a->core, &e);
}

void sc_acl_tick(sc_acl_t *a, float dt)
{
    sc_core_t *core = a->core;

    /* The core's own stream (JointState, events) has no reader in this mode. */
    scorbot_v1_Envelope e;
    while (sc_core_pop(core, &e)) {
    }

    /* A fault, once. */
    if (core->state == scorbot_v1_SystemState_SYSTEM_STATE_FAULT && core->fault != a->last_fault) {
        a->last_fault = core->fault;
        int joint = -1;
        for (uint8_t i = 0; i < core->joint_count; ++i) {
            if (core->joint[i].fault) {
                joint = i;
                break;
            }
        }
        char text[96];
        line_out(a, fault_text(core->fault, joint, text, sizeof(text)));
        const bool was_pending = a->homing_pending || a->moved_pending;
        a->homing_pending = false;
        stop_motion(a);
        a->manual = false;
        (void)was_pending;
        prompt(a); /* ends whatever the operator was waiting on */
    }

    /* The e-stop while idle: the rail went away without a fault (nothing was moving). */
    const bool off = power_off(a);
    if (off && !a->last_power_off && core->state != scorbot_v1_SystemState_SYSTEM_STATE_FAULT) {
        line_out(a, "CONTROL DISABLED (motor power off: press MOTORS ON)");
        prompt(a);
    }
    a->last_power_off = off;

    /* HOME: the prompt when the cycle ends. */
    if (a->homing_pending && core->state != scorbot_v1_SystemState_SYSTEM_STATE_HOMING) {
        a->homing_pending = false;
        if (core->state != scorbot_v1_SystemState_SYSTEM_STATE_FAULT) {
            capture_target(a); /* DELTA reads the home pose until HERE says otherwise */
            prompt(a);
        }
    }

    if (core_active(a)) {
        send_command(a, dt);
    } else if (a->moving || a->jog_joint >= 0) {
        stop_motion(a); /* the core dropped out of ACTIVE under us (a disable, a fault) */
        a->manual = false;
    }
}

/* ---- setup ----------------------------------------------------------------------------- */

void sc_acl_config_default(sc_acl_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    /* The ER-V's factors from the operator (counts per degree), per axis: base, shoulder,
       elbow (the shoulder's), wrist pitch, wrist roll (the pitch's); the slide uses its own
       real counts until measured. */
    const float per_deg[SC_CORE_MAX_JOINTS] = {42.5666f, 33.2121f, 33.2121f, 8.3555f, 8.3555f, 0.0f, 0.0f, 0.0f};
    for (uint8_t i = 0; i < SC_CORE_MAX_JOINTS; ++i) {
        cfg->virtual_counts_per_rad[i] = per_deg[i] > 0.0f ? per_deg[i] / DEG : 0.0f;
        cfg->virtual_sign[i] = 1.0f;
    }
    cfg->manual_speed_fraction = 0.3f;
    cfg->jog_hold_ms = 50;
    cfg->home_timeout_ms = 120000; /* a full-range search at a quarter speed on the elbow is about 45 s */
    cfg->move_tolerance_counts = 2.0f;
    cfg->position_gain = 10.0f;
}

void sc_acl_init(sc_acl_t *a, sc_core_t *core, const sc_acl_config_t *cfg)
{
    memset(a, 0, sizeof(*a));
    a->core = core;
    a->cfg = *cfg;
    if (a->cfg.manual_speed_fraction <= 0.0f) a->cfg.manual_speed_fraction = 0.3f;
    if (a->cfg.jog_hold_ms == 0) a->cfg.jog_hold_ms = 50;
    if (a->cfg.move_tolerance_counts <= 0.0f) a->cfg.move_tolerance_counts = 2.0f;
    if (a->cfg.position_gain <= 0.0f) a->cfg.position_gain = 10.0f;
    a->jog_joint = -1;
    a->speed_percent = 100;
    a->last_fault = scorbot_v1_FaultCode_FAULT_NONE;
}
