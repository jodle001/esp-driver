#include "current.h"

#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "current";
#define TIMEOUT_MS 20

/* ADS1015 config register: OS=1 start, MUX single-ended AINn = 0b100+n, PGA, MODE=1
   single shot, DR=110 (3300 SPS), comparator disabled (COMP_QUE=11). */
#define REG_CONVERSION 0x00
#define REG_CONFIG 0x01
#define PGA_256MV 0x5
#define PGA_4096V 0x1
#define DR_3300 0x6
#define VSENSE_INPUT 3 /* A3 of the second board */

static current_config_t s_cfg;
static i2c_master_dev_handle_t s_dev_a, s_dev_b;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static current_snapshot_t s_state;
static float s_duty[CURRENT_MAX]; /* magnitude of the commanded duty, from the control task */

void current_set_duty(const float *duty)
{
    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < s_cfg.count; ++i) {
        s_duty[i] = fabsf(duty[i]);
    }
    portEXIT_CRITICAL(&s_lock);
}

static float duty_now(int motor)
{
    portENTER_CRITICAL(&s_lock);
    const float d = s_duty[motor];
    portEXIT_CRITICAL(&s_lock);
    return d;
}

/// Amps from a raw +-256 mV conversion that ran while the motor was at `duty`.
static float amps(int16_t raw, float duty)
{
    const float d = duty < s_cfg.min_duty ? s_cfg.min_duty : duty;
    const float mv = raw * 0.125f / d; /* 256 mV / 2048 per LSB, undo the duty weighting */
    const float a = (mv - s_cfg.offset_mv) / s_cfg.mv_per_a;
    return a < 0.0f ? 0.0f : a;
}

static esp_err_t start(i2c_master_dev_handle_t dev, uint8_t channel, uint8_t pga)
{
    const uint16_t cfg = (uint16_t)((1u << 15) | ((0x4u + channel) << 12) | (pga << 9) | (1u << 8) | (DR_3300 << 5) | 0x3);
    const uint8_t buf[3] = {REG_CONFIG, (uint8_t)(cfg >> 8), (uint8_t)cfg};
    return i2c_master_transmit(dev, buf, sizeof(buf), TIMEOUT_MS);
}

static esp_err_t read_raw(i2c_master_dev_handle_t dev, int16_t *out)
{
    const uint8_t reg = REG_CONVERSION;
    uint8_t buf[2];
    const esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, buf, 2, TIMEOUT_MS);
    if (err == ESP_OK) {
        *out = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]) >> 4; /* 12-bit, left justified */
    }
    return err;
}

/* The second board's inputs: CS5.. on A0-A2 (those that exist), VSENSE on A3. */
static int b_inputs(uint8_t *inputs)
{
    int n = 0;
    for (int m = 4; m < s_cfg.count; ++m) {
        inputs[n++] = (uint8_t)(m - 4);
    }
    inputs[n++] = VSENSE_INPUT;
    return n;
}

static void current_task(void *arg)
{
    uint8_t b_list[4];
    const int b_count = b_inputs(b_list);
    const int a_count = s_cfg.count < 4 ? s_cfg.count : 4;
    uint8_t ch_a = 0;
    int ib = 0;
    for (;;) {
        const uint8_t ch_b = b_list[ib];
        const bool b_is_cs = ch_b != VSENSE_INPUT;
        const int motor_b = 4 + ch_b;
        /* The duty while the conversion runs: mean of the value at its start and its end
           (the control task updates it at 1 kHz, the conversion window is 0.3 ms). */
        const float d0_a = duty_now(ch_a);
        const float d0_b = b_is_cs ? duty_now(motor_b) : 0.0f;
        bool err = start(s_dev_a, ch_a, PGA_256MV) != ESP_OK;
        err |= start(s_dev_b, ch_b, b_is_cs ? PGA_256MV : PGA_4096V) != ESP_OK;
        vTaskDelay(1); /* 3300 SPS conversion takes 0.3 ms; the tick is 1 ms (CONFIG_FREERTOS_HZ=1000) */
        int16_t raw_a = 0, raw_b = 0;
        err |= read_raw(s_dev_a, &raw_a) != ESP_OK;
        err |= read_raw(s_dev_b, &raw_b) != ESP_OK;
        const float d1_a = duty_now(ch_a);
        const float d1_b = b_is_cs ? duty_now(motor_b) : 0.0f;

        const float a = amps(raw_a, 0.5f * (d0_a + d1_a));
        const float b_val = b_is_cs ? amps(raw_b, 0.5f * (d0_b + d1_b)) : raw_b * 0.002f * s_cfg.vsense_ratio; /* 4.096 V / 2048 = 2 mV per LSB */

        portENTER_CRITICAL(&s_lock);
        s_state.bus_error = err;
        if (!err) {
            s_state.current_a[ch_a] += 0.5f * (a - s_state.current_a[ch_a]);
            if (b_is_cs) {
                s_state.current_a[motor_b] += 0.5f * (b_val - s_state.current_a[motor_b]);
            } else {
                s_state.rail_v += 0.5f * (b_val - s_state.rail_v);
                s_state.rail_ok = s_state.rail_v >= s_cfg.rail_ok_v;
            }
        }
        portEXIT_CRITICAL(&s_lock);
        ch_a = (uint8_t)((ch_a + 1) % a_count);
        ib = (ib + 1) % b_count;
    }
}

esp_err_t current_init(const current_config_t *cfg)
{
    if (!cfg || cfg->count < 1 || cfg->count > 7) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    if (s_cfg.mv_per_a <= 0.0f) s_cfg.mv_per_a = 20.0f;
    if (s_cfg.vsense_ratio <= 0.0f) s_cfg.vsense_ratio = 4.9f;
    if (s_cfg.rail_ok_v <= 0.0f) s_cfg.rail_ok_v = 9.0f;
    if (s_cfg.min_duty <= 0.0f || s_cfg.min_duty > 1.0f) s_cfg.min_duty = 0.05f;
    i2c_device_config_t dev = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = cfg->addr_a, .scl_speed_hz = cfg->speed_hz};
    esp_err_t err = i2c_master_bus_add_device(cfg->bus, &dev, &s_dev_a);
    if (err != ESP_OK) {
        return err;
    }
    dev.device_address = cfg->addr_b;
    if ((err = i2c_master_bus_add_device(cfg->bus, &dev, &s_dev_b)) != ESP_OK) {
        return err;
    }
    /* Probe both with a config write before the task starts. */
    if ((err = start(s_dev_a, 0, PGA_256MV)) != ESP_OK || (err = start(s_dev_b, VSENSE_INPUT, PGA_4096V)) != ESP_OK) {
        ESP_LOGE(TAG, "ADS1015 not answering: %s", esp_err_to_name(err));
        return err;
    }
    xTaskCreatePinnedToCore(current_task, "current", 3072, NULL, 8, NULL, 0);
    ESP_LOGI(TAG, "two ADS1015 at 0x%02x/0x%02x for %d motors, %.0f mV/A, duty floor %.2f", cfg->addr_a, cfg->addr_b, cfg->count,
             (double)s_cfg.mv_per_a, (double)s_cfg.min_duty);
    return ESP_OK;
}

void current_collect(current_snapshot_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    portEXIT_CRITICAL(&s_lock);
}
