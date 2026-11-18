#include "plant.h"

#include <math.h>
#include <string.h>

#define PI_D 3.14159265358979323846

scorbot_v1_JointCalibration plant_default_calibration(uint32_t index)
{
    scorbot_v1_JointCalibration c;
    memset(&c, 0, sizeof(c));
    c.index = index;
    c.counts_per_motor_rev = 80;
    const float d = (float)(PI_D / 180.0);
    switch (index) {
    case 0: c.gear_ratio = 635.5f; c.soft_limit_min_rad = -155 * d; c.soft_limit_max_rad = 155 * d; c.max_velocity_rad_s = 0.35f; c.max_accel_rad_s2 = 1.0f; break;
    case 1: c.gear_ratio = 457.56f; c.soft_limit_min_rad = -35 * d; c.soft_limit_max_rad = 130 * d; c.max_velocity_rad_s = 0.45f; c.max_accel_rad_s2 = 1.0f; break;
    case 2: c.gear_ratio = 457.56f; c.soft_limit_min_rad = -130 * d; c.soft_limit_max_rad = 130 * d; c.max_velocity_rad_s = 0.45f; c.max_accel_rad_s2 = 1.0f; break;
    case 3: c.gear_ratio = 125.54f; c.soft_limit_min_rad = -130 * d; c.soft_limit_max_rad = 130 * d; c.max_velocity_rad_s = 1.4f; c.max_accel_rad_s2 = 3.0f; break;
    default: c.gear_ratio = 125.54f; c.soft_limit_min_rad = -570 * d; c.soft_limit_max_rad = 570 * d; c.max_velocity_rad_s = 1.8f; c.max_accel_rad_s2 = 3.0f; break;
    }
    c.home_offset_rad = 0.0f;
    c.home_direction = -1;
    c.current_limit_a = 2.0f;
    return c;
}

static double counts_per_rad(const plant_joint_t *j)
{
    return (double)j->cal.counts_per_motor_rev * j->cal.gear_ratio / (2.0 * PI_D);
}

bool plant_switch_pressed(const plant_joint_t *j, double pos)
{
    const double s = (j->cal.home_direction >= 0 ? 1.0 : -1.0) * (pos - j->cal.home_offset_rad);
    return s >= 0.0 && s < j->switch_width;
}

double plant_switch_edge(const plant_joint_t *j) { return j->cal.home_offset_rad; }
double plant_soft_min(const plant_joint_t *j) { return j->cal.soft_limit_min_rad; }
double plant_soft_max(const plant_joint_t *j) { return j->cal.soft_limit_max_rad; }

static int32_t counts_at(const plant_joint_t *j, double pos)
{
    const double c = (pos - j->boot_position) * counts_per_rad(j);
    const double r = c < 0 ? -floor(-c + 0.5) : floor(c + 0.5);
    return (int32_t)(j->encoder_reversed ? -r : r);
}

void plant_init(plant_t *p, uint8_t joint_count, const scorbot_v1_JointCalibration *cal,
                const double *initial_positions, const bool *encoder_reversed)
{
    memset(p, 0, sizeof(*p));
    p->joint_count = joint_count;
    p->motor_power = true;
    p->motor_power_valid = true;
    p->enc_supply_valid = false;
    p->enc_supply_h = true;
    p->enc_supply_l = false;
    for (uint8_t i = 0; i < joint_count; ++i) {
        plant_joint_t *j = &p->joint[i];
        j->cal = cal ? cal[i] : plant_default_calibration(i);
        j->encoder_reversed = encoder_reversed ? encoder_reversed[i] : false;
        j->hard_margin = 0.15;
        j->switch_width = 0.02;
        j->motor_tau = 0.05;
        j->brake_tau = 0.01;
        const double away = -0.4 * (j->cal.home_direction >= 0 ? 1.0 : -1.0);
        j->true_position = initial_positions ? initial_positions[i] : j->cal.home_offset_rad + away;
        j->boot_position = j->true_position;
        j->limit_switch = plant_switch_pressed(j, j->true_position);
    }
}

void plant_reset_counters(plant_t *p)
{
    for (uint8_t i = 0; i < p->joint_count; ++i) {
        p->joint[i].boot_position = p->joint[i].true_position;
        p->joint[i].counts = 0;
        p->joint[i].switch_edge = false;
    }
}

void plant_read(plant_t *p, sc_input_t *in)
{
    memset(in, 0, sizeof(*in));
    in->motor_power = p->motor_power;
    in->motor_power_valid = p->motor_power_valid;
    in->enc_supply_valid = p->enc_supply_valid;
    in->enc_supply_h = p->enc_supply_h;
    in->enc_supply_l = p->enc_supply_l;
    for (uint8_t i = 0; i < p->joint_count; ++i) {
        plant_joint_t *j = &p->joint[i];
        in->joint[i].counts = j->counts;
        in->joint[i].limit_switch = j->limit_switch;
        in->joint[i].switch_edge = j->switch_edge;
        in->joint[i].latched_counts = j->latched_counts;
        in->joint[i].current_a = (float)j->current;
        in->joint[i].driver_fault = false;
        j->switch_edge = false; /* consumed */
    }
}

void plant_step(plant_t *p, const sc_output_t *out, double dt)
{
    for (uint8_t i = 0; i < p->joint_count; ++i) {
        plant_joint_t *j = &p->joint[i];
        const double vmax = j->cal.max_velocity_rad_s;
        const double amax = j->cal.max_accel_rad_s2;
        const double imax = j->cal.current_limit_a;
        const double v_full = 1.5 * vmax;
        const double duty = out->drives_enabled ? out->duty[i] : 0.0;
        const bool brake = !out->drives_enabled || out->brake[i] || !p->motor_power;
        const double v_target = p->motor_power ? duty * v_full : 0.0;

        if (brake) {
            j->velocity += (0.0 - j->velocity) * (1.0 - exp(-dt / j->brake_tau));
        } else {
            j->velocity += (v_target - j->velocity) * (1.0 - exp(-dt / j->motor_tau));
        }
        j->true_position += j->velocity * dt;

        const double hard_max = j->cal.soft_limit_max_rad + j->hard_margin;
        const double hard_min = j->cal.soft_limit_min_rad - j->hard_margin;
        j->stalled = false;
        if (j->true_position > hard_max) {
            j->true_position = hard_max;
            j->stalled = !brake && v_target > 0.0;
            j->velocity = 0.0;
        }
        if (j->true_position < hard_min) {
            j->true_position = hard_min;
            j->stalled = !brake && v_target < 0.0;
            j->velocity = 0.0;
        }

        /* Current: bias + speed term + the (voltage - back EMF) term; a stall pins it above
           the limit like the simulator does. */
        if (!out->drives_enabled || !p->motor_power) {
            j->current = 0.0;
        } else if (j->stalled) {
            j->current = 1.5 * imax;
        } else {
            const double slip = fabs(v_target - j->velocity) / v_full;
            j->current = 0.15 + 0.5 * imax * fabs(j->velocity) / vmax + 0.4 * imax * (slip > 1.0 ? 1.0 : slip);
        }
        (void)amax;

        const bool pressed = plant_switch_pressed(j, j->true_position);
        if (pressed != j->limit_switch) {
            /* The interrupt latches the count at the edge itself, not at this sample. */
            const double edge = pressed ? (j->cal.home_direction >= 0 ? 1.0 : -1.0) *
                                            ((j->cal.home_direction >= 0 ? 1.0 : -1.0) * (j->true_position - j->cal.home_offset_rad) < j->switch_width / 2 ? 0.0 : j->switch_width) + j->cal.home_offset_rad
                                        : j->true_position;
            j->latched_counts = counts_at(j, pressed ? edge : j->true_position);
            j->switch_edge = true;
        }
        j->limit_switch = pressed;
        j->counts = counts_at(j, j->true_position);
    }
}
