// scorbot_core: the controller behaviour, as portable C with no ESP-IDF calls.
//
// Everything the ROS side depends on lives here: the state machine, homing, the
// watchdog, soft limits, fault latching, calibration handling, the per-joint control
// law and the messages on the wire. It takes encoder counts, switch states and currents
// in, and gives motor duties out, on a fixed tick. The device build wraps it in the
// ESP-IDF glue (main/, components/link_uart, motor, encoder, limit_switch, current,
// nvs_store); the host build wraps it in test/host/plant.c so the same core runs
// against scorbot_esp_sim's behaviour spec (scorbot_esp_sim/test/test_controller_sim.cpp)
// before it ever touches a motor.
//
// Threading: none. One owner calls sc_core_tick(), sc_core_handle() and sc_core_pop()
// from one context (the 1 kHz control task on the device). Inbound envelopes and
// outbound frames are queued by the glue around it.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "scorbot_proto/envelope.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SC_CORE_MAX_JOINTS SC_MAX_JOINTS
#define SC_CORE_OUTBOUND 12    /* envelopes queued between ticks */
#define SC_CORE_VEL_WINDOW 16  /* ticks of encoder history for the velocity estimate */
#define SC_CORE_ROBOT_TYPE_LEN 16
#define SC_CORE_JOINT_NAME_LEN 24

/// Per-joint inner-loop gains, in duty per (rad/s). Zero = derive from the calibration
/// (full duty assumed to give 1.5 x max_velocity_rad_s); tune on the bench per axis.
typedef struct {
    float kff; /* feedforward: duty per rad/s of reference velocity */
    float kp;  /* proportional on velocity error */
    float ki;  /* integral on velocity error, per second */
} sc_gains_t;

/// Callbacks the core uses to persist what must survive a reboot. NULL = not persisted.
typedef struct {
    void *ctx;
    void (*calibration)(void *ctx, const scorbot_v1_JointCalibration *cal);
    void (*robot_type)(void *ctx, const char *robot_type);
} sc_persist_t;

typedef struct {
    /* identity */
    const char *firmware_version;                                  /* "0.1.0" */
    const char *joint_names[SC_CORE_MAX_JOINTS];                   /* GetInfo order */
    uint8_t joint_count;

    /* persisted values, loaded by the glue before init */
    char robot_type[SC_CORE_ROBOT_TYPE_LEN];                       /* "unset" if none */
    uint32_t boot_count;                                           /* already incremented for this boot */
    bool has_calibration[SC_CORE_MAX_JOINTS];
    scorbot_v1_JointCalibration calibration[SC_CORE_MAX_JOINTS];
    sc_persist_t persist;

    /* tunables (0 = default noted) */
    uint32_t watchdog_ms;                /* 200 */
    uint32_t home_timeout_ms;            /* 30000 */
    float homing_speed_fraction;         /* 0.25 of max_velocity_rad_s */
    float backoff_margin_rad;            /* 0.02 past the release point before re-approaching */
    float state_period_s;                /* 0.010: JointState emission */
    float boot_delay_s;                  /* 0.050: BOOT -> UNHOMED */
    float hard_stop_current_fraction;    /* 0.9 of current_limit_a: homing hard-stop detector */
    uint32_t hard_stop_ms;               /* 50: how long the detector must hold */
    uint32_t overcurrent_ms;             /* 30: current above the limit this long -> FAULT_OVERCURRENT */
    uint32_t stall_ms;                   /* 300: full duty with no motion this long -> FAULT_STALL; 0 = off */
    float soft_limit_overrun_rad;        /* 0.05 past a soft limit -> FAULT_SOFT_LIMIT */
    bool limit_switch_faults_when_active;/* false: Scorbot switches sit mid-range */
    bool require_calibration;            /* true on the device: Enable refused until every joint has one */
    sc_gains_t gains[SC_CORE_MAX_JOINTS];
} sc_core_config_t;

/// What the glue measured for one joint since the last tick.
typedef struct {
    int32_t counts;          /* raw quadrature count, signed, accumulated (PCNT + overflow) */
    bool limit_switch;       /* debounced switch level, true = pressed */
    bool switch_edge;        /* the switch changed since the last tick */
    int32_t latched_counts;  /* raw count captured in the switch-edge interrupt (valid with switch_edge) */
    float current_a;         /* driver current sense, magnitude */
    bool driver_fault;       /* the driver's FLT line is asserted */
} sc_joint_input_t;

typedef struct {
    sc_joint_input_t joint[SC_CORE_MAX_JOINTS];
    bool motor_power_valid;  /* the rail is measured (VSENSE present) */
    bool motor_power;        /* the 12 V motor rail is up (e-stop closed) */
} sc_input_t;

typedef struct {
    float duty[SC_CORE_MAX_JOINTS];   /* -1..+1, motor frame (invert already applied) */
    bool brake[SC_CORE_MAX_JOINTS];   /* short the motor (duty is 0 when set) */
    bool drives_enabled;              /* the shared SLP line: high in HOMING and ACTIVE */
} sc_output_t;

/* ---- per-joint state, exposed read-only for the harness and the console -------------- */
typedef struct {
    scorbot_v1_JointCalibration cal;
    bool has_cal;
    /* measurement */
    int32_t raw_counts;
    int32_t corrected_counts;     /* encoder_invert applied */
    int32_t zero_counts;          /* corrected count that reads as 0 (switch edge once homed) */
    int32_t history[SC_CORE_VEL_WINDOW];
    uint8_t history_head;
    uint8_t history_filled;
    float position;               /* rad, motor frame (soft limits and home_offset live here) */
    float velocity;               /* rad/s, motor frame */
    float current;                /* A, magnitude */
    bool limit_switch;
    bool homed;
    bool at_soft_limit;
    bool fault;
    /* control */
    scorbot_v1_ControlMode mode;
    float setpoint_position;
    float setpoint_velocity;
    float v_ref;                  /* accel-limited reference the inner loop follows */
    float integral;
    float duty;
    bool brake;
    float over_current_s;
    float stall_s;
    /* homing */
    bool homing;
    int8_t homing_direction;
    bool homing_reversed;
    bool homing_passed;
    bool homing_backoff;
    bool homing_released;
    float homing_release_pos;
    float hard_stop_s;
} sc_joint_t;

typedef struct {
    sc_core_config_t cfg;
    sc_joint_t joint[SC_CORE_MAX_JOINTS];
    uint8_t joint_count;
    scorbot_v1_SystemState state;
    scorbot_v1_FaultCode fault;
    bool fault_trusted;           /* positions survive this fault (ClearFault -> READY) */
    bool unhomed_active;
    char robot_type[SC_CORE_ROBOT_TYPE_LEN];
    uint32_t watchdog_ms;
    uint64_t now_us;
    float boot_elapsed_s;
    float since_state_s;
    float since_command_s;
    float homing_elapsed_s;
    float homing_timeout_s;
    uint32_t homing_mask;
    uint32_t homed_progress_mask;
    uint32_t commands_received;
    uint32_t commands_ignored;
    uint32_t seq;
    float last_dt_s;
    bool first_tick;
    bool motor_power_seen;
    /* outbound ring */
    scorbot_v1_Envelope outbound[SC_CORE_OUTBOUND];
    uint8_t out_head, out_count;
    uint32_t out_dropped;
} sc_core_t;

/// Fill cfg with the defaults noted above (no joints, no calibration).
void sc_core_config_default(sc_core_config_t *cfg);

/// Reset the core to BOOT with this configuration. Safe to call again for a reboot.
void sc_core_init(sc_core_t *core, const sc_core_config_t *cfg);

/// Advance one control period: read the inputs, run the state machine and the loops,
/// write the outputs. dt_s is the measured period (1 ms nominal).
void sc_core_tick(sc_core_t *core, float dt_s, const sc_input_t *in, sc_output_t *out);

/// Consume one inbound Envelope (JointCommand or Request; anything else is ignored).
void sc_core_handle(sc_core_t *core, const scorbot_v1_Envelope *in);

/// Take the next outbound Envelope (JointState, Response, Event). False when empty.
bool sc_core_pop(sc_core_t *core, scorbot_v1_Envelope *out);
/// Envelopes queued right now.
uint8_t sc_core_outbound_count(const sc_core_t *core);

/// Raise a fault from outside the tick (self-test failure, I2C error, injection).
/// joint < 0 marks no joint. Faults whose positions stay valid per protocol.md keep
/// homing across ClearFault; pass trusted=true to override (e-stop power loss).
void sc_core_raise_fault(sc_core_t *core, scorbot_v1_FaultCode code, int joint, bool trusted);

/* ---- names, shared with the console ------------------------------------------------- */
const char *sc_state_name(scorbot_v1_SystemState s);
const char *sc_fault_name(scorbot_v1_FaultCode f);

#ifdef __cplusplus
}
#endif
