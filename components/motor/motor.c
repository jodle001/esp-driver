#include "motor.h"

#include <math.h>

#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "motor";

#define LEDC_RES LEDC_TIMER_10_BIT /* 1024 steps at 20 kHz from the 80 MHz APB clock */
#define SLP_BIT 0x20

esp_err_t motor_init(motor_t *m, const motor_config_t *cfg)
{
    m->cfg = *cfg;
    m->enabled = false;
    m->port_a = 0;
    m->duty_max = (1u << LEDC_RES) - 1u;
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = cfg->pwm_freq_hz ? cfg->pwm_freq_hz : 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        const ledc_channel_config_t ch = {
            .gpio_num = cfg->pwm_gpio[i],
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        if ((err = ledc_channel_config(&ch)) != ESP_OK) {
            return err;
        }
    }
    /* Drivers asleep, PWM low. */
    err = expander_write_port_a(cfg->expander, 0x00);
    ESP_LOGI(TAG, "5 x LEDC at %lu Hz, SLP low", (unsigned long)timer.freq_hz);
    return err;
}

esp_err_t motor_apply(motor_t *m, const float *duty, const bool *brake, bool drives_enabled)
{
    uint8_t port_a = drives_enabled ? SLP_BIT : 0;
    uint32_t ledc[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        float d = drives_enabled && !brake[i] ? duty[i] : 0.0f;
        if (d > 1.0f) d = 1.0f;
        if (d < -1.0f) d = -1.0f;
        if (d < 0.0f) {
            port_a |= (uint8_t)(1u << i); /* DIR */
            d = -d;
        }
        ledc[i] = (uint32_t)lroundf(d * (float)m->duty_max);
    }
    /* Direction and sleep first, so a sign change never drives the old way with the new
       duty for a whole I2C transaction; when waking the drivers, PWM is still 0. */
    esp_err_t err = ESP_OK;
    if (port_a != m->port_a) {
        err = expander_write_port_a(m->cfg.expander, port_a);
        if (err != ESP_OK) {
            return err;
        }
        m->port_a = port_a;
    }
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)i, ledc[i]);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)i);
    }
    m->enabled = drives_enabled;
    return err;
}

uint8_t motor_faults_from_ports(uint8_t gpio_a, uint8_t gpio_b)
{
    /* FLT1-2 on GPA6-7, FLT3-5 on GPB0-2, active low. */
    uint8_t f = 0;
    if (!(gpio_a & 0x40)) f |= 1u << 0;
    if (!(gpio_a & 0x80)) f |= 1u << 1;
    if (!(gpio_b & 0x01)) f |= 1u << 2;
    if (!(gpio_b & 0x02)) f |= 1u << 3;
    if (!(gpio_b & 0x04)) f |= 1u << 4;
    return f;
}
