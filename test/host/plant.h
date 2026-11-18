// A stand-in for the robot behind the core on the host: one motor, encoder and switch
// per joint with the same mechanics scorbot_esp_sim assumes (mid-range switch of a
// fixed width, hard stops a margin past the soft limits, a current that rises at a
// stall), plus a first-order motor so the core's inner loop has something to control.
// Also the harness that wires core and plant together the way the ESP glue will.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "scorbot_core/core.h"

#define PLANT_MAX_JOINTS SC_CORE_MAX_JOINTS

typedef struct {
    /* mechanics */
    scorbot_v1_JointCalibration cal;
    bool encoder_reversed;    /* counts fall when the motor drives positive */
    double true_position;     /* rad, calibrated frame (the switch edge is at home_offset_rad) */
    double velocity;          /* rad/s */
    double hard_margin;       /* end stop this far beyond each soft limit */
    double switch_width;      /* switch stays pressed this far past its edge */
    double motor_tau;         /* s, first-order motor response */
    double brake_tau;         /* s, decay while braked */
    /* observed */
    double boot_position;     /* where the counter was zeroed */
    int32_t counts;
    bool limit_switch;
    bool switch_edge;
    int32_t latched_counts;
    double current;
    bool stalled;
} plant_joint_t;

typedef struct {
    plant_joint_t joint[PLANT_MAX_JOINTS];
    uint8_t joint_count;
    bool motor_power;
    bool motor_power_valid;
    /* the encoder 5 V sense pins; not reported unless a test turns the sense on */
    bool enc_supply_valid;
    bool enc_supply_h;        /* 1 = present */
    bool enc_supply_l;        /* 0 = present */
} plant_t;

/// Scorbot-like default calibration for joint index (matches scorbot_esp_sim).
scorbot_v1_JointCalibration plant_default_calibration(uint32_t index);

void plant_init(plant_t *p, uint8_t joint_count, const scorbot_v1_JointCalibration *cal,
                const double *initial_positions, const bool *encoder_reversed);
/// Read what the glue would measure since the last tick.
void plant_read(plant_t *p, sc_input_t *in);
/// Apply the core's outputs and advance the mechanics by dt.
void plant_step(plant_t *p, const sc_output_t *out, double dt);
/// Zero the counters (a controller reboot); mechanics unchanged.
void plant_reset_counters(plant_t *p);
double plant_switch_edge(const plant_joint_t *j);
bool plant_switch_pressed(const plant_joint_t *j, double pos);
double plant_soft_min(const plant_joint_t *j);
double plant_soft_max(const plant_joint_t *j);
