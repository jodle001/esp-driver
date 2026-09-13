#include "limit_switch.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"

static const char *TAG = "limit_switch";

static limit_switch_config_t s_cfg;
static TaskHandle_t s_task;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* Written by the ISR, consumed by the task. */
static volatile int32_t s_isr_counts[LIMIT_SWITCH_COUNT];
static volatile bool s_isr_pending;

/* Shared with the control loop under s_lock. */
static limit_switch_snapshot_t s_state;
static int64_t s_last_edge_us[LIMIT_SWITCH_COUNT];

static void IRAM_ATTR on_inta(void *arg)
{
    /* The latch: every count, right now, before anything else runs. */
    for (int i = 0; i < LIMIT_SWITCH_COUNT; ++i) {
        int c = 0;
        pcnt_unit_get_count(s_cfg.encoders[i]->pcnt_unit, &c);
        s_isr_counts[i] = c;
    }
    s_isr_pending = true;
    BaseType_t woken = pdFALSE;
    if (s_task) {
        vTaskNotifyGiveFromISR(s_task, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

static void apply_ports(uint8_t gpio_a, uint8_t gpio_b, const int32_t *latched, bool from_edge)
{
    const int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    s_state.fault_mask = motor_faults_from_ports(gpio_a, gpio_b);
    s_state.bus_error = false;
    for (int i = 0; i < LIMIT_SWITCH_COUNT; ++i) {
        const bool level = (gpio_b >> (3 + i)) & 1u;
        const bool pressed = s_cfg.active_low ? !level : level;
        if (pressed != s_state.joint[i].pressed) {
            if (now - s_last_edge_us[i] < (int64_t)s_cfg.debounce_us && !from_edge) {
                continue; /* bounce: keep the latched edge, ignore the re-read */
            }
            s_state.joint[i].pressed = pressed;
            /* `edge` promises a valid latch. A change seen only by the periodic re-read
               (an interrupt that was missed) has none; the core then falls back to the
               count of the tick it sees the level change in. */
            s_state.joint[i].edge = latched != NULL;
            if (latched) {
                s_state.joint[i].latched_count = latched[i];
            }
            s_last_edge_us[i] = now;
        }
    }
    portEXIT_CRITICAL(&s_lock);
}

static void limit_switch_task(void *arg)
{
    uint8_t a = 0, b = 0;
    if (expander_read(s_cfg.expander, &a, &b) == ESP_OK) {
        apply_ports(a, b, NULL, false);
    }
    for (;;) {
        /* Wake on an edge, or every 2 ms to re-read the levels (debounce settle, and a
           safety net if an interrupt is ever missed). */
        const uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
        int32_t latched[LIMIT_SWITCH_COUNT];
        bool edge = false;
        if (notified && s_isr_pending) {
            portENTER_CRITICAL(&s_lock);
            for (int i = 0; i < LIMIT_SWITCH_COUNT; ++i) {
                latched[i] = s_isr_counts[i];
            }
            s_isr_pending = false;
            portEXIT_CRITICAL(&s_lock);
            edge = true;
            uint8_t intf_a, intf_b, cap_a, cap_b;
            if (expander_read_interrupt(s_cfg.expander, &intf_a, &intf_b, &cap_a, &cap_b) == ESP_OK) {
                apply_ports(cap_a, cap_b, latched, true);
                continue;
            }
        }
        if (expander_read(s_cfg.expander, &a, &b) == ESP_OK) {
            apply_ports(a, b, edge ? latched : NULL, edge);
        } else {
            portENTER_CRITICAL(&s_lock);
            s_state.bus_error = true;
            portEXIT_CRITICAL(&s_lock);
        }
    }
}

esp_err_t limit_switch_init(const limit_switch_config_t *cfg)
{
    if (!cfg || !cfg->expander) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    if (s_cfg.debounce_us == 0) {
        s_cfg.debounce_us = 2000;
    }
    const gpio_config_t io = {
        .intr_type = GPIO_INTR_NEGEDGE, /* INTA active low */
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << cfg->int_gpio,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }
    xTaskCreatePinnedToCore(limit_switch_task, "limit_switch", 3072, NULL, 12, &s_task, 0);
    err = gpio_isr_handler_add(cfg->int_gpio, on_inta, NULL);
    ESP_LOGI(TAG, "INTA on GPIO %d, %s switches", cfg->int_gpio, cfg->active_low ? "active-low" : "active-high");
    return err;
}

void limit_switch_collect(limit_switch_snapshot_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    for (int i = 0; i < LIMIT_SWITCH_COUNT; ++i) {
        s_state.joint[i].edge = false;
    }
    portEXIT_CRITICAL(&s_lock);
}
