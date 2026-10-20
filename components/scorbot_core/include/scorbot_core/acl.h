// scorbot_core/acl: the Eshed box's serial dialect on top of the core, so the Pi runs the
// operator unchanged against this controller (decided Oct 16 2026, design doc section 7).
//
// It speaks exactly the subset of ACL the operator speaks (reference_only/operator/src/ER_V,
// acl_private.hpp and erv.cpp) and nothing more: one '>' prompt per command after its
// reply, the DEFP/HOME/HERE/SHIFT/MOVE/clrbuf/SPEED/CON vocabulary around one stored
// target called DELTA, LISTPV POSITION answered as "1:n 2:n 3:n 4:n 5:n", manual mode
// behind '~' with the pendant's jog keys, Ctrl-C, and the fault words the operator
// looks for (IMPACT, CONTROL DISABLED, FAULT, "ACL - unrecognized request").
//
// Units on the wire are virtual ER-V encoder counts: the operator's degree factors and
// its Home are the ER-V's, so this layer converts between those counts and the core's
// calibrated radians with a scale and sign per axis (configuration, never code). The
// slide, when fitted, is the Eshed's axis 7.
//
// Portable C, no ESP-IDF: the device glue and test/host/core_sim both wrap it the same
// way: feed the bytes from the link, tick it once per core tick after sc_core_tick, and
// write what sc_acl_read gives back to the link. Output is printable text, CR, LF and
// '>' only, never XON/XOFF, because the operator runs the port with software flow control.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "scorbot_core/core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SC_ACL_LINE_MAX 64
#define SC_ACL_OUT_MAX 512

typedef struct {
    /* virtual ER-V counts per radian of the calibrated joint, per core joint; 0 = the
       joint's own real counts per radian (from its calibration) */
    float virtual_counts_per_rad[SC_CORE_MAX_JOINTS];
    float virtual_sign[SC_CORE_MAX_JOINTS];  /* +1 or -1: which way the ER-V counts up */
    float manual_speed_fraction;             /* 0.3: jog speed as a fraction of max_velocity_rad_s */
    uint32_t jog_hold_ms;                    /* 50: a jog outlives the last key byte this long */
    uint32_t home_timeout_ms;                /* 120000: HOME may have to search the whole range; the operator waits */
    float move_tolerance_counts;             /* 2: a move is done within this many virtual counts */
    float position_gain;                     /* 10 /s: v = gain x error, capped by SPEED */
} sc_acl_config_t;

typedef struct {
    sc_core_t *core;
    sc_acl_config_t cfg;
    /* input */
    char line[SC_ACL_LINE_MAX];
    uint8_t line_len;
    bool line_overflow;
    bool manual;                             /* '~' pressed: key bytes jog, lines are not parsed */
    /* the DELTA variable and motion */
    bool target_defined;
    int32_t target[SC_CORE_MAX_JOINTS];      /* virtual counts */
    bool moving;
    bool moved_pending;                      /* MOVED: prompt when the move completes */
    bool hold[SC_CORE_MAX_JOINTS];           /* hold the reached target in position mode */
    float hold_position[SC_CORE_MAX_JOINTS]; /* rad, reported frame */
    int8_t jog_joint;                        /* -1 = none */
    int8_t jog_dir;
    float jog_hold_s;
    uint8_t speed_percent;                   /* SPEED, 1..100 */
    bool homing_pending;                     /* HOME: prompt when the core leaves HOMING */
    /* what was last reported */
    scorbot_v1_FaultCode last_fault;
    bool last_power_off;
    uint32_t next_request_id;
    /* output ring */
    uint8_t out[SC_ACL_OUT_MAX];
    uint16_t out_head, out_len;
    uint32_t out_dropped;
    /* counters */
    uint32_t lines, unknown_lines, prompts, jog_bytes;
} sc_acl_t;

/// Defaults: the ER-V's count factors (42.5666 counts/deg base, 33.2121 shoulder and
/// elbow, 8.3555 wrist pitch and roll), sign +1, jog hold 50 ms, SPEED 100.
void sc_acl_config_default(sc_acl_config_t *cfg);
void sc_acl_init(sc_acl_t *a, sc_core_t *core, const sc_acl_config_t *cfg);

/// Bytes from the Pi. Lines are executed as they complete; key bytes act at once.
void sc_acl_feed(sc_acl_t *a, const uint8_t *data, size_t n);
/// Once per core tick, after sc_core_tick: drives the core (the command that feeds the
/// watchdog, the move and jog references) and reports state changes (prompt after
/// HOME, fault lines).
void sc_acl_tick(sc_acl_t *a, float dt_s);
/// Bytes for the Pi (non-blocking; returns how many were copied).
size_t sc_acl_read(sc_acl_t *a, uint8_t *out, size_t cap);
size_t sc_acl_pending(const sc_acl_t *a);

/// The joint's position in virtual counts, as LISTPV reports it.
int32_t sc_acl_virtual_counts(const sc_acl_t *a, uint8_t joint);
/// Core joint index for an ACL axis number (1-5 the arm, 6 the gripper when seven channels are fitted, 7 = the slide, always the last channel), or -1.
int sc_acl_joint_for_axis(const sc_acl_t *a, int axis);

#ifdef __cplusplus
}
#endif
