#include <inttypes.h>
#include <math.h>
#include <stdlib.h>

#include "ads1015.h"
#include "driver/gpio.h"
#include "driver_socket.h"
#include "driver_socket_api.h"
#include "driver_wifi.h"
#include "encoder.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"
#include "limit_switch.h"
#include "motor_config.h"
#include "motorhat.h"
#include "nvs_flash.h"
#include "pca9685.h"
#include "pi_link.h"
#include "signal_bus.h"

#define TAG "MAIN"

#define ENCODER_GLITCH_FILTER 100  // CONFIG_ENCODER_GLITCH_FILTER
#define ENCODER_STANDARD_RESOLUTION 20
#define POLAR_PAN_SPEED

#ifdef CONFIG_DRIVER_LINK_WIFI
// Only the Wi-Fi socket uses this; -Werror rejects it unused in Pi mode
static const driver_socket_api_motor_interface_t motor_interface = {
    .polar_pan = motorhat_polar_pan,
    .polar_pan_start = motorhat_polar_pan_start,
    .polar_pan_stop = motorhat_polar_pan_stop,
    .home = motorhat_home,
};
#endif

typedef struct {
  encoder_handle_t* encoders;
  int count;
} encoder_array_t;

void clear_all_encoders(void* ctx) {
  encoder_array_t* arr = (encoder_array_t*)ctx;
  for (int i = 0; i < arr->count; i++) {
    encoder_clear_count(&arr->encoders[i]);
  }
}

static const int axis_encoder_index[MOTORHAT_NUM_AXES] = {
    [MOTORHAT_AXIS_AZIMUTH] = 0,
    [MOTORHAT_AXIS_ALTITUDE] = 1,
    [MOTORHAT_AXIS_ALTITUDE_B] = 2,
};

static esp_err_t read_axis_count(void* ctx, motorhat_axis_t axis, int* count) {
  encoder_array_t* arr = (encoder_array_t*)ctx;
  if (arr == NULL || count == NULL || axis < MOTORHAT_AXIS_AZIMUTH ||
      axis >= MOTORHAT_NUM_AXES) {
    return ESP_ERR_INVALID_ARG;
  }

  int index = axis_encoder_index[axis];
  if (index >= arr->count) {
    return ESP_ERR_INVALID_ARG;
  }

  return encoder_get_raw_count(&arr->encoders[index], count);
}

// Each encoder slot produces 4 counts due to x4 quadrature decoding (see
// encoder_get_raw_count in encoder.h)
#define ENCODER_COUNTS_PER_SLOT 4

typedef struct {
  const char* name;
  int encoder_slots;
  const char* gearbox_ratio;
  int pinion_teeth;
  int driven_teeth;
  int range_pos_deg;
  int range_neg_deg;
} soft_limit_axis_params_t;

// Encoder counts per degree of joint travel, from the motor's gearing
static esp_err_t counts_per_degree(const soft_limit_axis_params_t* params,
                                   double* counts_per_deg) {
  if (params->encoder_slots <= 0 || params->pinion_teeth <= 0 ||
      params->driven_teeth <= 0) {
    ESP_LOGE(TAG, "Soft limits %s: slots and teeth must be positive",
             params->name);
    return ESP_ERR_INVALID_ARG;
  }

  char* end = NULL;
  double gearbox = strtod(params->gearbox_ratio, &end);
  if (end == params->gearbox_ratio || *end != '\0' || !(gearbox > 0.0)) {
    ESP_LOGE(TAG, "Soft limits %s: invalid gearbox ratio \"%s\"", params->name,
             params->gearbox_ratio);
    return ESP_ERR_INVALID_ARG;
  }

  *counts_per_deg = (double)params->encoder_slots * ENCODER_COUNTS_PER_SLOT *
                    gearbox * params->driven_teeth / params->pinion_teeth /
                    360.0;
  return ESP_OK;
}

static esp_err_t compute_soft_limit(const soft_limit_axis_params_t* params,
                                    int percent, motorhat_soft_limit_t* limit) {
  double counts_per_deg = 0.0;
  esp_err_t err = counts_per_degree(params, &counts_per_deg);
  if (err != ESP_OK) {
    return err;
  }
  double fraction = percent / 100.0;

  // round each magnitude down so rounding can only shrink the bounds
  double pos = floor(params->range_pos_deg * fraction * counts_per_deg);
  double neg = floor(params->range_neg_deg * fraction * counts_per_deg);

  if (!(pos <= INT32_MAX) || !(neg <= INT32_MAX)) {
    ESP_LOGE(TAG, "soft limits %s: bounds overflow", params->name);
    return ESP_ERR_INVALID_SIZE;
  }

  limit->max_count = (int32_t)pos;
  limit->min_count = -(int32_t)neg;

  ESP_LOGI(TAG,
           "Soft limits %s: %.2f counts/deg, allowed counts %" PRId32
           " to %" PRId32,
           params->name, counts_per_deg, limit->min_count, limit->max_count);
  return ESP_OK;
}

// One configured motor from menuconfig's Motors menu
typedef struct {
  const char* name;
  motorhat_motor_t port;  // MOTORHAT_MOTOR1-8 = HAT 1 M1 ... HAT 2 M4
  gpio_num_t encoder_a;
  gpio_num_t encoder_b;
  const char* speed;  // fraction of full PWM, as text
  soft_limit_axis_params_t limits;
  bool forward_increases_count;
} motor_setting_t;

#define MOTOR_SETTING(ID, SYM, NAME)                                    \
  [ID] = {                                                              \
      .name = NAME,                                                     \
      .port = MOTOR_CONFIG_PORT_INDEX(SYM),                             \
      .encoder_a = CONFIG_DRIVER_MOTOR_##SYM##_ENCODER_A_PIN,           \
      .encoder_b = CONFIG_DRIVER_MOTOR_##SYM##_ENCODER_B_PIN,           \
      .speed = CONFIG_DRIVER_MOTOR_##SYM##_SPEED,                       \
      .limits =                                                         \
          {                                                             \
              .name = NAME,                                             \
              .encoder_slots = CONFIG_DRIVER_MOTOR_##SYM##_ENCODER_SLOTS, \
              .gearbox_ratio = CONFIG_DRIVER_MOTOR_##SYM##_GEARBOX_RATIO, \
              .pinion_teeth = CONFIG_DRIVER_MOTOR_##SYM##_PINION_TEETH, \
              .driven_teeth = CONFIG_DRIVER_MOTOR_##SYM##_DRIVEN_TEETH, \
              .range_pos_deg = CONFIG_DRIVER_MOTOR_##SYM##_RANGE_POS_DEG, \
              .range_neg_deg = CONFIG_DRIVER_MOTOR_##SYM##_RANGE_NEG_DEG, \
          },                                                            \
      .forward_increases_count = CONFIG_DRIVER_MOTOR_##SYM##_FWD_INCREASES, \
  },

// Indexed by motor ID; [0] is "no motor"
static const motor_setting_t motor_settings[MOTOR_CONFIG_COUNT + 1] = {
    MOTOR_CONFIG_LIST(MOTOR_SETTING)};

static int motor_hat_num(int id) {
  return motor_settings[id].port / MOTORHAT_MOTORS_PER_HAT + 1;
}
static int motor_hat_port(int id) {
  return motor_settings[id].port % MOTORHAT_MOTORS_PER_HAT + 1;
}

// The motor picked for each role in the Axes menu
#ifdef CONFIG_DRIVER_ALTITUDE_B_ENABLE
#define NUM_ROLES 3
#else
#define NUM_ROLES 2
#endif
static const int role_motor_id[MOTORHAT_NUM_AXES] = {
    [MOTORHAT_AXIS_AZIMUTH] = CONFIG_DRIVER_AZIMUTH_MOTOR_ID,
    [MOTORHAT_AXIS_ALTITUDE] = CONFIG_DRIVER_ALTITUDE_MOTOR_ID,
#ifdef CONFIG_DRIVER_ALTITUDE_B_ENABLE
    [MOTORHAT_AXIS_ALTITUDE_B] = CONFIG_DRIVER_ALTITUDE_B_MOTOR_ID,
#endif
};
static const char* const role_name[MOTORHAT_NUM_AXES] = {
    [MOTORHAT_AXIS_AZIMUTH] = "azimuth",
    [MOTORHAT_AXIS_ALTITUDE] = "altitude",
    [MOTORHAT_AXIS_ALTITUDE_B] = "altitude B",
};

#ifdef CONFIG_DRIVER_CLOSED_LOOP_ENABLE
// One joint's closed-loop settings from menuconfig, in degrees of the joint
typedef struct {
  const char* speed;      // jog speed, deg/s
  const char* kv;         // deg/s per unit of duty above the deadband
  const char* kp;         // duty per degree of error
  const char* ki;         // duty per degree*s of error
  const char* deadband;   // duty the motor needs to start moving
  const char* max_error;  // degrees behind before a fault
} closed_loop_setting_t;

// Joint 0 is azimuth, joint 1 altitude (both wrist motors when paired)
static const closed_loop_setting_t closed_loop_settings[2] = {
    {CONFIG_DRIVER_CL_AZ_SPEED, CONFIG_DRIVER_CL_AZ_KV, CONFIG_DRIVER_CL_AZ_KP,
     CONFIG_DRIVER_CL_AZ_KI, CONFIG_DRIVER_CL_AZ_DEADBAND,
     CONFIG_DRIVER_CL_AZ_MAX_ERROR},
    {CONFIG_DRIVER_CL_ALT_SPEED, CONFIG_DRIVER_CL_ALT_KV,
     CONFIG_DRIVER_CL_ALT_KP, CONFIG_DRIVER_CL_ALT_KI,
     CONFIG_DRIVER_CL_ALT_DEADBAND, CONFIG_DRIVER_CL_ALT_MAX_ERROR},
};

static int role_joint(int role) {
  return role == MOTORHAT_AXIS_AZIMUTH ? 0 : 1;
}

// Fills config->servo from menuconfig. Degrees become counts with the gearing
// of the motor picked for the joint's role, and each role's Pan speed becomes
// its duty cap. Call after the soft limits are filled in: they become the
// joints' position limits.
static esp_err_t configure_closed_loop(motorhat_config_t* config) {
  motorhat_servo_config_t* servo = &config->servo;
  servo->enabled = true;
  servo->period_ms = CONFIG_DRIVER_CL_PERIOD_MS;
  servo->log_ms = CONFIG_DRIVER_CL_LOG_MS;

  for (int j = 0; j < 2; j++) {
    const closed_loop_setting_t* s = &closed_loop_settings[j];
    int role = j == 0 ? MOTORHAT_AXIS_AZIMUTH : MOTORHAT_AXIS_ALTITUDE;
    double cpd = 0.0;
    esp_err_t err =
        counts_per_degree(&motor_settings[role_motor_id[role]].limits, &cpd);
    if (err != ESP_OK) {
      return err;
    }
    double speed = atof(s->speed) * cpd;
    double kv = atof(s->kv) * cpd;
    servo->joint[j] = (servo_joint_params_t){
        .jog_speed = (float)speed,
        .accel = (float)(speed * 1000.0 / CONFIG_DRIVER_CL_ACCEL_MS),
        .kff = kv > 0.0 ? (float)(1.0 / kv) : 0.0f,
        .kp = (float)(atof(s->kp) / cpd),
        .ki = (float)(atof(s->ki) / cpd),
        .max_error = (float)(atof(s->max_error) * cpd),
        .fault_s = CONFIG_DRIVER_CL_FAULT_MS / 1000.0f,
        .stall_counts = (float)(atof(CONFIG_DRIVER_CL_STALL_DEG) * cpd),
        .hold_s = CONFIG_DRIVER_CL_HOLD_MS / 1000.0f,
        .hold_window = (float)(atof(CONFIG_DRIVER_CL_HOLD_WINDOW_DEG) * cpd),
        .max_pair = (float)(atof(CONFIG_DRIVER_CL_PAIR_DEG) * cpd),
    };
  }

  for (int r = 0; r < NUM_ROLES; r++) {
    const motor_setting_t* m = &motor_settings[role_motor_id[r]];
    // + joint motion drives the motor FORWARD (BACKWARD for a reversed
    // altitude B); the sign says which way that moves its count
    int sign = m->forward_increases_count ? 1 : -1;
#ifdef CONFIG_DRIVER_ALTITUDE_B_REVERSED
    if (r == MOTORHAT_AXIS_ALTITUDE_B) {
      sign = -sign;
    }
#endif
    config->forward_increases_count[r] = m->forward_increases_count;
    servo->motor[r] = (servo_motor_params_t){
        .sign = sign,
        .max_duty = (float)atof(m->speed),
        .deadband = (float)atof(closed_loop_settings[role_joint(r)].deadband),
    };

#ifdef CONFIG_DRIVER_SOFT_LIMITS_ENABLE
    // A joint may go only as far as every one of its motors may
    const motorhat_soft_limit_t* lim = &config->soft_limits[r];
    float lo = (float)(sign > 0 ? lim->min_count : -lim->max_count);
    float hi = (float)(sign > 0 ? lim->max_count : -lim->min_count);
    servo_joint_params_t* p = &servo->joint[role_joint(r)];
    p->min_pos = p->limited ? fmaxf(p->min_pos, lo) : lo;
    p->max_pos = p->limited ? fminf(p->max_pos, hi) : hi;
    p->limited = true;
#endif
  }
  return ESP_OK;
}
#endif

#ifdef CONFIG_DRIVER_CURRENT_SENSING_ENABLE
typedef struct {
  uint8_t addr;           // 0 ends the list
  gpio_num_t rdy;
  uint8_t pair_motor[2];  // A0-A1, A2-A3; motor ID, 0 = none
} adc_setting_t;

// Every ADC enabled in menuconfig (Talos ESP Driver Configuration)
static const adc_setting_t adc_settings[] = {
#ifdef CONFIG_DRIVER_ADC1_ENABLE
    {CONFIG_DRIVER_ADC1_ADDRESS, CONFIG_DRIVER_ADC1_RDY_PIN,
     {CONFIG_DRIVER_ADC1_PAIR_A_MOTOR_ID, CONFIG_DRIVER_ADC1_PAIR_B_MOTOR_ID}},
#endif
#ifdef CONFIG_DRIVER_ADC2_ENABLE
    {CONFIG_DRIVER_ADC2_ADDRESS, CONFIG_DRIVER_ADC2_RDY_PIN,
     {CONFIG_DRIVER_ADC2_PAIR_A_MOTOR_ID, CONFIG_DRIVER_ADC2_PAIR_B_MOTOR_ID}},
#endif
#ifdef CONFIG_DRIVER_ADC3_ENABLE
    {CONFIG_DRIVER_ADC3_ADDRESS, CONFIG_DRIVER_ADC3_RDY_PIN,
     {CONFIG_DRIVER_ADC3_PAIR_A_MOTOR_ID, CONFIG_DRIVER_ADC3_PAIR_B_MOTOR_ID}},
#endif
#ifdef CONFIG_DRIVER_ADC4_ENABLE
    {CONFIG_DRIVER_ADC4_ADDRESS, CONFIG_DRIVER_ADC4_RDY_PIN,
     {CONFIG_DRIVER_ADC4_PAIR_A_MOTOR_ID, CONFIG_DRIVER_ADC4_PAIR_B_MOTOR_ID}},
#endif
    {0},
};
#define MAX_ADCS 4

// Their tasks and RDY interrupts use these for as long as the firmware runs
static ads1015_handle_t adcs[MAX_ADCS];

// Motor port (1-8) the ADS1015 driver gets for a motor ID; 0 = not watched
static uint8_t adc_pair_port(int id) {
  return id == 0 ? 0 : motor_settings[id].port + 1;
}
#endif

// Two motors in use (roles and watched ADC pairs) must sit on different
// ports, and two roles must not share a motor or encoder pins. With the
// second HAT off, menuconfig clamps HAT 2 motors onto HAT 1, which lands them
// on another motor's port: this catches that too. Stops the boot on a clash.
static void check_motor_settings(void) {
  bool used[MOTOR_CONFIG_COUNT + 1] = {false};
  bool ok = true;

  for (int r = 0; r < NUM_ROLES; r++) {
    int id = role_motor_id[r];
    used[id] = true;
    for (int other = 0; other < r; other++) {
      const motor_setting_t* a = &motor_settings[role_motor_id[other]];
      const motor_setting_t* b = &motor_settings[id];
      if (role_motor_id[other] == id) {
        ESP_LOGE(TAG, "Roles %s and %s both use %s", role_name[other],
                 role_name[r], b->name);
        ok = false;
      } else if (a->encoder_a == b->encoder_a || a->encoder_a == b->encoder_b ||
                 a->encoder_b == b->encoder_a || a->encoder_b == b->encoder_b) {
        ESP_LOGE(TAG, "%s and %s share an encoder GPIO", a->name, b->name);
        ok = false;
      }
    }
  }
#ifdef CONFIG_DRIVER_CURRENT_SENSING_ENABLE
  for (int i = 0; adc_settings[i].addr != 0; i++) {
    used[adc_settings[i].pair_motor[0]] = true;
    used[adc_settings[i].pair_motor[1]] = true;
  }
#endif
  used[0] = false;

  for (int i = 1; i <= MOTOR_CONFIG_COUNT; i++) {
    for (int j = 1; j < i; j++) {
      if (used[i] && used[j] &&
          motor_settings[i].port == motor_settings[j].port) {
        ESP_LOGE(TAG, "%s and %s are both set to HAT %d M%d",
                 motor_settings[j].name, motor_settings[i].name,
                 motor_hat_num(i), motor_hat_port(i));
        ok = false;
      }
    }
  }
  if (!ok) {
    ESP_LOGE(TAG, "Motor settings clash, fix them in menuconfig (Motors, Axes)");
    ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
  }
  for (int r = 0; r < NUM_ROLES; r++) {
    int id = role_motor_id[r];
    ESP_LOGI(TAG, "Role %s: %s on HAT %d M%d, encoder GPIO%d/%d",
             role_name[r], motor_settings[id].name, motor_hat_num(id),
             motor_hat_port(id), motor_settings[id].encoder_a,
             motor_settings[id].encoder_b);
  }
}

#ifdef CONFIG_DRIVER_CURRENT_SENSING_ENABLE
// Checks the menuconfig settings, then starts every enabled ADC. Any mistake
// stops the boot: a wrong setting would leave a motor without overcurrent
// protection while looking fine.
static void current_sensing_init(i2c_master_bus_handle_t bus, int num_motors) {
  int count = 0;
  bool watched[CURRENT_NUM_BITS + 1] = {false};  // by port 1-8
  bool ok = true;

  for (int i = 0; adc_settings[i].addr != 0; i++) {
    const adc_setting_t* a = &adc_settings[i];
    count++;
    for (int j = 0; j < i; j++) {
      if (adc_settings[j].addr == a->addr) {
        ESP_LOGE(TAG, "Two ADCs at address 0x%02X", a->addr);
        ok = false;
      }
      if (adc_settings[j].rdy == a->rdy) {
        ESP_LOGE(TAG, "ADCs 0x%02X and 0x%02X share RDY GPIO%d", adc_settings[j].addr, a->addr, a->rdy);
        ok = false;
      }
    }
    if (a->rdy == CONFIG_DRIVER_MOTORHAT_SDA_PIN || a->rdy == CONFIG_DRIVER_MOTORHAT_SCL_PIN ||
        a->rdy == CONFIG_DRIVER_LIMIT_SWITCH_PIN) {
      ESP_LOGE(TAG, "ADC 0x%02X RDY GPIO%d is already the I2C or limit switch pin", a->addr, a->rdy);
      ok = false;
    }
    for (int p = 0; p < 2; p++) {
      int id = a->pair_motor[p];
      if (id == 0) continue;
      int port = adc_pair_port(id);
      if (port > num_motors) {
        ESP_LOGE(TAG, "ADC 0x%02X %s watches %s on HAT %d, but that HAT is off (Second MotorHAT)", a->addr,
                 p == 0 ? "A0-A1" : "A2-A3", motor_settings[id].name, motor_hat_num(id));
        ok = false;
      } else if (watched[port]) {
        ESP_LOGE(TAG, "%s is watched by two ADC pairs", motor_settings[id].name);
        ok = false;
      }
      watched[port] = true;
    }
  }
  if (!ok) {
    ESP_LOGE(TAG, "Current sensing settings are wrong, fix them in menuconfig");
    ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
  }
  if (count == 0) {
    ESP_LOGW(TAG, "Current sensing on but no ADC enabled, no overcurrent stop");
    return;
  }

  for (int i = 0; i < count; i++) {
    const adc_setting_t* a = &adc_settings[i];
    ads1015_config_t cfg = {
        .i2c_addr = a->addr,
        .i2c_speed_hz = 400000,
        .rdy_gpio = a->rdy,
        .bus_handle = bus,
        .adc_data_rate = CONFIG_DRIVER_ADS1015_DATA_RATE,
        .pair_motor = {adc_pair_port(a->pair_motor[0]), adc_pair_port(a->pair_motor[1])},
    };
    esp_err_t err = ads1015_init(&adcs[i], &cfg);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "ADC 0x%02X did not start (%s). Check its power, SDA/SCL and ADDR pin.", a->addr,
               esp_err_to_name(err));
      ESP_ERROR_CHECK(err);
    }
  }

  for (int id = 1; id <= MOTOR_CONFIG_COUNT; id++) {
    if (!watched[motor_settings[id].port + 1]) {
      ESP_LOGW(TAG, "%s (HAT %d M%d) has no current sensing", motor_settings[id].name, motor_hat_num(id),
               motor_hat_port(id));
    }
  }
  ESP_LOGI(TAG, "Current sensing: %d ADC(s) running", count);
}
#endif

// 1 = run main/axis_check.c instead of the normal firmware: robot connected,
// both HATs, motor supply on. Pulses each of the 6 axes briefly and reports
// encoder movement. Set back to 0 for the normal firmware.
#define AXIS_CHECK 0
void axis_check(void);

void app_main(void) {
#if AXIS_CHECK
  axis_check();  // never returns
#endif

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Install GPIO interrupt service
  ESP_ERROR_CHECK(gpio_install_isr_service(0));

  // Initialize signal bus
  ESP_ERROR_CHECK(signal_bus_init());

  // Initialize I2C bus
  i2c_bus_t bus;
  i2c_bus_config_t bus_config = {
      .port = I2C_NUM_0,
      .sda_io_num = CONFIG_DRIVER_MOTORHAT_SDA_PIN,
      .scl_io_num = CONFIG_DRIVER_MOTORHAT_SCL_PIN,
  };
  ESP_ERROR_CHECK(i2c_bus_init(&bus, &bus_config));

  // Initialize limit switches
  limit_switch_config_t limit_switch_config = {
      .limit_gpio = CONFIG_DRIVER_LIMIT_SWITCH_PIN,
  };
  ESP_ERROR_CHECK(limit_switch_init(&limit_switch_config));

  check_motor_settings();

  // One encoder per role, from the motor picked for it (index = role)
  encoder_handle_t encoders[MOTORHAT_NUM_AXES];
  for (int r = 0; r < NUM_ROLES; r++) {
    const motor_setting_t* m = &motor_settings[role_motor_id[r]];
    encoder_config_t encoder_config = {
        .P0_pin = m->encoder_a,
        .P1_pin = m->encoder_b,
        .resolution = CONFIG_ENCODER_0_RESOLUTION,
        .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER,
        .invert_angle = CONFIG_ENCODER_0_ANGLE_INVERT,
    };
    ESP_ERROR_CHECK(encoder_init(&encoders[r], &encoder_config));
    ESP_ERROR_CHECK(encoder_start(&encoders[r]));
  }

  encoder_array_t encoder_array = {
      .encoders = encoders,
      .count = NUM_ROLES,
  };

  // Initialize motor controller
  motorhat_handle_t motorhat;
  motorhat_config_t motorhat_config = {
      .pca9685_config =
          {
              {
                  .i2c_addr = CONFIG_DRIVER_MOTORHAT_ADDRESS,
                  .i2c_speed_hz = 400000,
                  .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
                  .bus_handle = bus.handle,
              },
#ifdef CONFIG_DRIVER_MOTORHAT2_ENABLE
              {
                  .i2c_addr = CONFIG_DRIVER_MOTORHAT2_ADDRESS,
                  .i2c_speed_hz = 400000,
                  .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
                  .bus_handle = bus.handle,
              },
#endif
          },
#ifdef CONFIG_DRIVER_MOTORHAT2_ENABLE
      .num_hats = 2,
#else
      .num_hats = 1,
#endif
      .polar_pan_speed =
          PCA9685_PWM_MAX * atof(CONFIG_DRIVER_MOTORHAT_PAN_SPEED),
      .encoder_cb = (motorhat_encoder_cb_t)clear_all_encoders,
      .encoder_ctx = &encoder_array,
      .limit_gpio = CONFIG_DRIVER_LIMIT_SWITCH_PIN,

      .axis_count_cb = read_axis_count,
      .axis_count_ctx = &encoder_array,
      .max_run_ms = CONFIG_DRIVER_MAX_RUN_MS,
  };

  for (int r = 0; r < NUM_ROLES; r++) {
    const motor_setting_t* m = &motor_settings[role_motor_id[r]];
    motorhat_config.axis_motor[r] = m->port;
    motorhat_config.axis_speed[r] = PCA9685_PWM_MAX * atof(m->speed);
  }

#ifdef CONFIG_DRIVER_ALTITUDE_B_ENABLE
  // ER-4pc wrist pitch: altitude drives both motors of the wrist differential.
  // The second motor runs at the altitude motor's speed so the wrist pitches
  // without rolling; its own speed setting is not used.
  motorhat_config.altitude_b_enabled = true;
  motorhat_config.axis_speed[MOTORHAT_AXIS_ALTITUDE_B] =
      motorhat_config.axis_speed[MOTORHAT_AXIS_ALTITUDE];
#ifdef CONFIG_DRIVER_ALTITUDE_B_REVERSED
  motorhat_config.altitude_b_reversed = true;
#endif
#endif

#ifdef CONFIG_DRIVER_HOMING_ENABLE
  motorhat_config.homing_enabled = true;
#else
  ESP_LOGW(TAG, "Homing disabled, home command will be rejected");
#endif

#ifdef CONFIG_DRIVER_SOFT_LIMITS_ENABLE
  // Each role takes the gearing, travel and direction of the motor picked
  // for it
  for (int r = 0; r < NUM_ROLES; r++) {
    const motor_setting_t* m = &motor_settings[role_motor_id[r]];
    ESP_ERROR_CHECK(compute_soft_limit(&m->limits,
                                       CONFIG_DRIVER_SOFT_LIMITS_PERCENT,
                                       &motorhat_config.soft_limits[r]));
    motorhat_config.forward_increases_count[r] = m->forward_increases_count;
  }
  motorhat_config.soft_limit_poll_ms = CONFIG_DRIVER_SOFT_LIMITS_POLL_MS;
  motorhat_config.soft_limits_enabled = true;

#else
  ESP_LOGW(TAG, "Soft limits disabled, motors are not bounded");
#endif

#ifdef CONFIG_DRIVER_CLOSED_LOOP_ENABLE
  ESP_ERROR_CHECK(configure_closed_loop(&motorhat_config));
#endif

  ESP_ERROR_CHECK(motorhat_init(&motorhat, &motorhat_config));

#ifdef CONFIG_DRIVER_CURRENT_SENSING_ENABLE
  // Started after the HATs so an overcurrent at once has motors to stop
  current_sensing_init(bus.handle, motorhat.num_motors);
#else
  ESP_LOGW(TAG, "Current sensing disabled, no overcurrent stop");
#endif

#ifdef CONFIG_DRIVER_LINK_PI_SERIAL
#if CONFIG_ESP_CONSOLE_UART_NUM == 0
#error "Pi serial link owns UART0: set console output to None or UART1"
#endif
  // Take commands from the Pi's operator over USB serial
  pi_link_config_t pi_link_config = {
      .baud_rate = CONFIG_DRIVER_PI_SERIAL_BAUD,
      .debug_reply = false,
      .motor =
          {
              .polar_pan_start = motorhat_polar_pan_start,
              .polar_pan_stop = motorhat_polar_pan_stop,
              .home = motorhat_home,
          },
  };
#ifdef CONFIG_DRIVER_PI_SERIAL_DEBUG_REPLY
  pi_link_config.debug_reply = true;
#endif
  ESP_ERROR_CHECK(pi_link_init(&pi_link_config));
#else
  // Initialize Wi-Fi and socket connection
  driver_wifi_config_t wifi_config = {
      .ssid = CONFIG_DRIVER_WIFI_SSID,
      .password = CONFIG_DRIVER_WIFI_PASSWORD,
  };

  ESP_ERROR_CHECK(wifi_init(&wifi_config));

  ESP_ERROR_CHECK(wait_for_wifi_connection());

  driver_socket_handle_t socket_handle;
  driver_socket_config_t socket_config = {
      .ip = CONFIG_DRIVER_SERVER_IP,
      .port = CONFIG_DRIVER_SERVER_PORT,
  };

  ESP_ERROR_CHECK(
      driver_socket_init(&socket_handle, &socket_config, &motor_interface));
#endif

  // Track encoder values
  int azimuth_count;
  const char* azimuth_name =
      motor_settings[role_motor_id[MOTORHAT_AXIS_AZIMUTH]].name;

  while (1) {
    esp_err_t err = encoder_get_raw_count(&encoders[MOTORHAT_AXIS_AZIMUTH],
                                          &azimuth_count);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to get encoder count: %s", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "Azimuth (%s) count: %i", azimuth_name, azimuth_count);
    int altitude_count = 0;
    encoder_get_raw_count(&encoders[MOTORHAT_AXIS_ALTITUDE], &altitude_count);
#ifdef CONFIG_DRIVER_ALTITUDE_B_ENABLE
    int altitude_b_count = 0;
    encoder_get_raw_count(&encoders[MOTORHAT_AXIS_ALTITUDE_B],
                          &altitude_b_count);
    ESP_LOGI(TAG, "Altitude count: %i   Altitude B count: %i", altitude_count,
             altitude_b_count);
#else
    ESP_LOGI(TAG, "Altitude count: %i", altitude_count);
#endif

    vTaskDelay(pdMS_TO_TICKS(500));  // Avoid busy loop

    // This loop can also be used to add periodic tasks like reading sensors, or
    // checking limit switches. However, it must not terminate or structs
    // initialized here will disappear from stack memory, breaking modules that
    // use them.
  }
}
