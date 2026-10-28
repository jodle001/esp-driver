#include <math.h>
#include <inttypes.h>
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
#include "motorhat.h"
#include "nvs_flash.h"
#include "pca9685.h"
#include "signal_bus.h"

#define TAG "MAIN"

#define ENCODER_GLITCH_FILTER 100  // CONFIG_ENCODER_GLITCH_FILTER
#define ENCODER_STANDARD_RESOLUTION 20
#define POLAR_PAN_SPEED

static const driver_socket_api_motor_interface_t motor_interface = {
    .polar_pan = motorhat_polar_pan,
    .polar_pan_start = motorhat_polar_pan_start,
    .polar_pan_stop = motorhat_polar_pan_stop,
    .home = motorhat_home,
};

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
};

static esp_err_t read_axis_count(void* ctx, motorhat_axis_t axis, int* count) {
    encoder_array_t* arr = (encoder_array_t*)ctx;
    if (arr == NULL || count == NULL || axis < MOTORHAT_AXIS_AZIMUTH || axis >= MOTORHAT_NUM_AXES) {
        return ESP_ERR_INVALID_ARG;
    }

    int index = axis_encoder_index[axis];
    if (index >= arr->count) {
        return ESP_ERR_INVALID_ARG;
    }

    return encoder_get_raw_count(&arr->encoders[index], count);
}

// Each encoder slot produces 4 counts due to x4 quadrature decoding (see encoder_get_raw_count in encoder.h)
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

static esp_err_t compute_soft_limit(const soft_limit_axis_params_t* params, int percent, motorhat_soft_limit_t* limit) {
    if (params->encoder_slots <= 0 || params->pinion_teeth <= 0 || params->driven_teeth<= 0) {
        ESP_LOGE(TAG, "Soft limits %s: slots and teeth must be positive", params->name);
        return ESP_ERR_INVALID_ARG;
    }

    char* end = NULL;
    double gearbox = strtod(params->gearbox_ratio, &end);
    if (end == params->gearbox_ratio || *end != '\0' || !(gearbox > 0.0)) {
        ESP_LOGE(TAG, "Soft limits %s: invalid gearbox ratio \"%s\"", params->name, params->gearbox_ratio);
        return ESP_ERR_INVALID_ARG;
    }

    double counts_per_deg = (double)params->encoder_slots * ENCODER_COUNTS_PER_SLOT * gearbox * params->driven_teeth / params->pinion_teeth / 360.0;
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

    ESP_LOGI(TAG, "Soft limits %s: %.2f counts/deg, allowed counts %" PRId32 " to %" PRId32, params->name, counts_per_deg, limit->min_count, limit->max_count);
    return ESP_OK;
}


void app_main(void) {
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

  // Initialize encoders
  encoder_handle_t encoders[2];
  encoder_config_t encoder_axis1_config = {
      .P0_pin = 36,
      .P1_pin = 39,
      .resolution = CONFIG_ENCODER_0_RESOLUTION,
      .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER,
      .invert_angle = CONFIG_ENCODER_0_ANGLE_INVERT,
  };
  ESP_ERROR_CHECK(encoder_init(&encoders[0], &encoder_axis1_config));
  ESP_ERROR_CHECK(encoder_start(&encoders[0]));

  encoder_config_t encoder_axis2_config = {
      .P0_pin = 34,
      .P1_pin = 35,
      .resolution = CONFIG_ENCODER_0_RESOLUTION,
      .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER,
      .invert_angle = CONFIG_ENCODER_0_ANGLE_INVERT,
  };
  ESP_ERROR_CHECK(encoder_init(&encoders[1], &encoder_axis2_config));
  ESP_ERROR_CHECK(encoder_start(&encoders[1]));

  encoder_array_t encoder_array = {
    .encoders = encoders,
    .count = 2,
};

  // Initialize motor controller
  motorhat_handle_t motorhat;
  motorhat_config_t motorhat_config = {
      .pca9685_config =
          {
              .i2c_addr = CONFIG_DRIVER_MOTORHAT_ADDRESS,
              .i2c_speed_hz = 400000,
              .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
              .bus_handle = bus.handle,
          },
      .polar_pan_speed =
          PCA9685_PWM_MAX * atof(CONFIG_DRIVER_MOTORHAT_PAN_SPEED),
      .encoder_cb = (motorhat_encoder_cb_t)clear_all_encoders,
      .encoder_ctx = &encoder_array,
      .limit_gpio = CONFIG_DRIVER_LIMIT_SWITCH_PIN,

      .axis_count_cb = read_axis_count,
      .axis_count_ctx = &encoder_array,
  };

  #ifdef CONFIG_DRIVER_SOFT_LIMITS_ENABLE
    const soft_limit_axis_params_t soft_limit_params[MOTORHAT_NUM_AXES] = {
        [MOTORHAT_AXIS_AZIMUTH] =
            {
                .name = "azimuth",
                .encoder_slots = CONFIG_DRIVER_SOFT_LIMITS_AZIMUTH_ENCODER_SLOTS,
                .gearbox_ratio = CONFIG_DRIVER_SOFT_LIMITS_AZIMUTH_GEARBOX_RATIO,
                .pinion_teeth = CONFIG_DRIVER_SOFT_LIMITS_AZIMUTH_PINION_TEETH,
                .driven_teeth = CONFIG_DRIVER_SOFT_LIMITS_AZIMUTH_DRIVEN_TEETH,
                .range_pos_deg = CONFIG_DRIVER_SOFT_LIMITS_AZIMUTH_RANGE_POS_DEG,
                .range_neg_deg = CONFIG_DRIVER_SOFT_LIMITS_AZIMUTH_RANGE_NEG_DEG,
            },
        [MOTORHAT_AXIS_ALTITUDE] =
            {
                .name = "altitude",
                .encoder_slots = CONFIG_DRIVER_SOFT_LIMITS_ALTITUDE_ENCODER_SLOTS,
                .gearbox_ratio = CONFIG_DRIVER_SOFT_LIMITS_ALTITUDE_GEARBOX_RATIO,
                .pinion_teeth = CONFIG_DRIVER_SOFT_LIMITS_ALTITUDE_PINION_TEETH,
                .driven_teeth = CONFIG_DRIVER_SOFT_LIMITS_ALTITUDE_DRIVEN_TEETH,
                .range_pos_deg = CONFIG_DRIVER_SOFT_LIMITS_ALTITUDE_RANGE_POS_DEG,
                .range_neg_deg = CONFIG_DRIVER_SOFT_LIMITS_ALTITUDE_RANGE_NEG_DEG,
            },
    };

    for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
        ESP_ERROR_CHECK(compute_soft_limit(&soft_limit_params[axis], CONFIG_DRIVER_SOFT_LIMITS_PERCENT, &motorhat_config.soft_limits[axis]));
    }
    motorhat_config.forward_increases_count[MOTORHAT_AXIS_AZIMUTH] = CONFIG_DRIVER_SOFT_LIMITS_AZIMUTH_FWD_INCREASES;
    motorhat_config.forward_increases_count[MOTORHAT_AXIS_ALTITUDE] = CONFIG_DRIVER_SOFT_LIMITS_ALTITUDE_FWD_INCREASES;
    motorhat_config.soft_limit_poll_ms = CONFIG_DRIVER_SOFT_LIMITS_POLL_MS;
    motorhat_config.soft_limits_enabled = true;

  #else
    ESP_LOGW(TAG, "Soft limits disabled, motors are not bounded");
  #endif

  ESP_ERROR_CHECK(motorhat_init(&motorhat, &motorhat_config));

  // Initialize current sensing
  ads1015_handle_t ads;
  ads1015_config_t ads_config = {
      .i2c_addr = CONFIG_DRIVER_ADS1015_ADDRESS,
      .i2c_speed_hz = 400000,
      .rdy_gpio = CONFIG_DRIVER_ADS1015_RDY_PIN,
      .bus_handle = bus.handle,
      .adc_data_rate = CONFIG_DRIVER_ADS1015_DATA_RATE,
  };
  ESP_ERROR_CHECK(ads1015_init(&ads, &ads_config));

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


  // Track encoder values
  int axis1_count;
  int axis2_count;

  while (1) {
    esp_err_t err = encoder_get_raw_count(&encoders[0], &axis1_count);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to get encoder count: %s", esp_err_to_name(err));
    }
    err = encoder_get_raw_count(&encoders[1], &axis2_count);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to get encoder count: %s", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "Axis 1 count: %i       Axis 2 count: %i", axis1_count, axis2_count);

    vTaskDelay(pdMS_TO_TICKS(500));  // Avoid busy loop

    // This loop can also be used to add periodic tasks like reading sensors, or
    // checking limit switches. However, it must not terminate or structs
    // initialized here will disappear from stack memory, breaking modules that
    // use them.
  }
}
