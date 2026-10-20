#include "motor.h"

#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "soc/soc_caps.h"

static const char *TAG = "motor";

#define LEDC_RES LEDC_TIMER_10_BIT /* 1024 steps at 20 kHz from the 80 MHz APB clock */
#if SOC_LEDC_SUPPORT_HS_MODE
#define PWM_SPEED_MODE LEDC_HIGH_SPEED_MODE
#else
#define PWM_SPEED_MODE LEDC_LOW_SPEED_MODE /* the ESP32-S3 has one mode, eight channels */
#endif

static esp_err_t slp_init(int gpio)
{
    /* Level first, then direction: the pin goes from high-Z (pulled down by the board)
       to a driven low, never through a high. */
    esp_err_t err = gpio_set_level(gpio, 0);
    if (err != ESP_OK) {
        return err;
    }
    const gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << gpio,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    err = gpio_config(&io);
    return err == ESP_OK ? gpio_set_level(gpio, 0) : err;
}

esp_err_t motor_init(motor_t *m, const motor_config_t *cfg)
{
    if (!cfg || cfg->count < 1 || cfg->count > MOTOR_MAX - 1 || !cfg->dir_expander) {
        return ESP_ERR_INVALID_ARG;
    }
    m->cfg = *cfg;
    m->enabled = false;
    m->port_a = 0;
    m->duty_max = (1u << LEDC_RES) - 1u;
    esp_err_t err = slp_init(cfg->slp_gpio); /* drivers asleep before anything else */
    if (err != ESP_OK) {
        return err;
    }
    const ledc_timer_config_t timer = {
        .speed_mode = PWM_SPEED_MODE,
        .duty_resolution = LEDC_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = cfg->pwm_freq_hz ? cfg->pwm_freq_hz : 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < cfg->count; ++i) {
        const ledc_channel_config_t ch = {
            .gpio_num = cfg->pwm_gpio[i],
            .speed_mode = PWM_SPEED_MODE,
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
    /* DIR low, PWM low, drivers asleep. */
    err = expander_write_port_a(cfg->dir_expander, 0x00);
    ESP_LOGI(TAG, "%d x LEDC at %lu Hz, SLP low on GPIO %d", cfg->count, (unsigned long)timer.freq_hz, cfg->slp_gpio);
    return err;
}

esp_err_t motor_apply(motor_t *m, const float *duty, const bool *brake, bool drives_enabled)
{
    uint8_t port_a = 0;
    uint32_t ledc[MOTOR_MAX];
    for (int i = 0; i < m->cfg.count; ++i) {
        float d = drives_enabled && !brake[i] ? duty[i] : 0.0f;
        if (d > 1.0f) d = 1.0f;
        if (d < -1.0f) d = -1.0f;
        if (d < 0.0f) {
            port_a |= expander_dir_bit(i);
            d = -d;
        }
        ledc[i] = (uint32_t)lroundf(d * (float)m->duty_max);
    }
    /* Sleep first when dropping the drives: no I2C transaction in that path. */
    if (!drives_enabled && m->enabled) {
        gpio_set_level(m->cfg.slp_gpio, 0);
        m->enabled = false;
    }
    /* Direction before duty, so a sign change never drives the old way with the new
       duty for a whole I2C transaction; when waking the drivers, PWM is still 0. */
    esp_err_t err = ESP_OK;
    if (port_a != m->port_a) {
        err = expander_write_port_a(m->cfg.dir_expander, port_a);
        if (err != ESP_OK) {
            return err;
        }
        m->port_a = port_a;
    }
    for (int i = 0; i < m->cfg.count; ++i) {
        ledc_set_duty(PWM_SPEED_MODE, (ledc_channel_t)i, ledc[i]);
        ledc_update_duty(PWM_SPEED_MODE, (ledc_channel_t)i);
    }
    if (drives_enabled && !m->enabled) {
        gpio_set_level(m->cfg.slp_gpio, 1);
        m->enabled = true;
    }
    return err;
}

uint8_t motor_faults_from_port(uint8_t sense_a, int count)
{
    uint8_t f = 0;
    for (int i = 0; i < count; ++i) {
        if (expander_fault(i, sense_a)) {
            f |= (uint8_t)(1u << i);
        }
    }
    return f;
}
