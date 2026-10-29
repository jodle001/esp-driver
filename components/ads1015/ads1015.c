#include "ads1015.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "signal_bus.h"

#define TAG "ADS1015"

static TaskHandle_t adc_task_handle = NULL;

static void IRAM_ATTR ads1015_isr(void* arg) {
  BaseType_t higher_priority_task_woken = pdFALSE;

  if (adc_task_handle) {
    xTaskNotifyFromISR(adc_task_handle, 0, eNoAction,
                       &higher_priority_task_woken);
  }

  if (higher_priority_task_woken) {
    portYIELD_FROM_ISR();
  }
}

void adc_task(void* arg) {
  ads1015_handle_t* handle = (ads1015_handle_t*)arg;
  uint16_t config_reg =
      handle->config_reg |
      (1 << ADS1015_OS_BIT);  // Modify so that writing will start conversions

  // Input of the conversion in progress (0 = A0-A1, 1 = A2-A3), and how many
  // samples in a row each input has been past a threshold
  int input = 0;
  int over_count[2] = {0, 0};

  ads1015_write_register(handle, ADS1015_CONFIG,
                         config_reg);  // Start conversions

  while (1) {
    // Wait until RDY interrupt fires
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Read conversion value
    uint16_t raw;
    if (ads1015_read_register(handle, ADS1015_CONVERSION, &raw) == ESP_OK) {
      int16_t value = ((int16_t)raw) >> 4;
      bool over = value >= CONFIG_ADS1015_HIGH_THRESH ||
                  value <= CONFIG_ADS1015_LOW_THRESH;
      if (!over) {
        over_count[input] = 0;
      } else if (over_count[input] < CONFIG_ADS1015_TRIP_SAMPLES) {
        over_count[input]++;
        if (over_count[input] == CONFIG_ADS1015_TRIP_SAMPLES) {
          ESP_LOGW(TAG, "Overcurrent on input %d", input);
        }
      }

      // Keeps signalling while the input stays past the threshold
      if (over_count[input] == CONFIG_ADS1015_TRIP_SAMPLES) {
        xEventGroupSetBits(g_motor_events, CURRENT_0);
      }
    }

    // Switch MUX inputs and start next conversion
    input = !input;
    config_reg &= ~(0b111 << ADS1015_MUX_SHIFT);  // clear MUX bits
    config_reg |= (input ? (ADS1015_MUX_AIN2_AIN3 << ADS1015_MUX_SHIFT)
                         : (ADS1015_MUX_AIN0_AIN1 << ADS1015_MUX_SHIFT));

    ads1015_write_register(handle, ADS1015_CONFIG, config_reg);
  }
}

esp_err_t ads1015_init(ads1015_handle_t* handle,
                       const ads1015_config_t* config) {
  if (handle == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = config->i2c_addr,
      .scl_speed_hz = config->i2c_speed_hz,
  };

  esp_err_t ret = i2c_master_bus_add_device(config->bus_handle, &dev_config,
                                            &handle->dev_handle);
  if (ret != ESP_OK) {
    return ret;
  }

  // Set comparator threshold registers for RDY mode
  ret = ads1015_write_register(handle, ADS1015_HIGH_THRESH_REG, 0x8000);
  if (ret != ESP_OK) {
    return ret;
  }
  ret = ads1015_write_register(handle, ADS1015_LOW_THRESH_REG, 0x0000);
  if (ret != ESP_OK) {
    return ret;
  }

  // Write config register
  uint16_t config_reg = ads1015_build_config(
      ADS1015_MUX_AIN0_AIN1, ADS1015_PGA_4_096V, ADS1015_MODE_SINGLESHOT,
      config->adc_data_rate, ADS1015_COMP_TRADITIONAL, ADS1015_COMP_ACTIVE_LOW,
      ADS1015_COMP_NON_LATCHING, ADS1015_COMP_ASSERT_1,
      false  // don't conversions immediately
  );

  ret = ads1015_write_register(handle, ADS1015_CONFIG, config_reg);
  if (ret != ESP_OK) {
    return ret;
  }

  handle->config_reg = config_reg;

  // Configure alert GPIO and interrupt service
  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_NEGEDGE,
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = 1ULL << config->rdy_gpio,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
  };

  ret = gpio_config(&io_conf);
  if (ret != ESP_OK) {
    return ret;
  }
  ret = gpio_isr_handler_add(config->rdy_gpio, ads1015_isr, NULL);
  if (ret != ESP_OK) {
    return ret;
  }

  // Start waiting task
  xTaskCreate(adc_task, "adc_task", 4096, handle, 10, &adc_task_handle);

  ESP_LOGI(TAG, "ADS1015 initialized");

  return ESP_OK;
}

esp_err_t ads1015_read_register(ads1015_handle_t* handle,
                                ads1015_register_t reg, uint16_t* data) {
  if (handle == NULL || data == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t rx_buf[2];

  esp_err_t ret =
      i2c_master_transmit_receive(handle->dev_handle, (uint8_t*)&reg, 1, rx_buf,
                                  2, ADS1015_I2C_MASTER_TIMEOUT_MS);

  if (ret == ESP_OK) {
    *data = (rx_buf[0] << 8) | rx_buf[1];
  }

  return ret;
}

esp_err_t ads1015_write_register(ads1015_handle_t* handle,
                                 ads1015_register_t reg, uint16_t data) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t write_buf[3];
  write_buf[0] = reg;
  write_buf[1] = (data >> 8) & 0xFF;  // MSB first
  write_buf[2] = data & 0xFF;         // LSB
  return i2c_master_transmit(handle->dev_handle, write_buf, sizeof(write_buf),
                             ADS1015_I2C_MASTER_TIMEOUT_MS);
}
