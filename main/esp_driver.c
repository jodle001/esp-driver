// Scorbot ESP32 controller firmware: wires the portable core (components/scorbot_core)
// to the rev 0.3 board. Pins, addresses and scales come from Kconfig ("Talos ESP Driver
// Configuration"); nothing robot-specific is compiled in, the calibration lives in NVS
// (pushed once over the protobuf link by scorbot_protocol_cli).
//
// The link to the Pi is UART0 (the DevKit's USB). It speaks the Eshed box's dialect
// (scorbot_core/acl.h, the operator unchanged) or the protobuf framing (the ROS 2
// stack), chosen in Kconfig or, in auto mode, from the first bytes (scorbot_core/link_mode.h).
//
// Tasks:
//   control (1 kHz, core 1, from an esp_timer): read encoders, switches, currents;
//           sc_core_tick; write PWM/DIR/SLP; move the link's bytes both ways
//   link_rx (core 0): UART bytes -> stream buffer for the control task
//   limit_switch (core 0): INTA -> which switch moved, with the encoder latch from the ISR
//   current (core 0): ADS1015 round-robin
#include <string.h>

#include "current.h"
#include "driver/gpio.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "expander.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "limit_switch.h"
#include "link_uart.h"
#include "motor.h"
#include "nvs_store.h"
#include "scorbot_core/acl.h"
#include "scorbot_core/core.h"
#include "scorbot_core/link_mode.h"
#include "scorbot_proto/framing.h"
#include "sdkconfig.h"

static const char *TAG = "scorbot";

#define JOINTS CONFIG_SC_JOINT_COUNT
#define FIRMWARE_VERSION "0.2.0"

#if JOINTS > 5
#error "The slide channel (joint 6) is provided for on the rev 0.3 board, but its device glue (motor, limit_switch, current, the second expander at 0x21) is not written yet"
#endif

static const char *kJointNames[6] = {"base_joint", "shoulder_joint", "elbow_joint", "wrist_pitch_joint", "wrist_roll_joint", "slide_joint"};

static sc_core_t s_core;
static sc_core_config_t s_cfg;
static sc_acl_t s_acl;
static sc_link_mode_t s_mode;
static sc_link_sniffer_t s_sniffer;
static sc_stream_decoder_t s_decoder;
static encoder_handle_t s_encoders[JOINTS];
static expander_t s_expander;
static motor_t s_motor;
static TaskHandle_t s_control_task;
static int64_t s_last_tick_us;

/* ---- persistence hooks --------------------------------------------------------- */

static void persist_calibration(void *ctx, const scorbot_v1_JointCalibration *cal)
{
    const esp_err_t err = nvs_store_save_calibration(cal);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "saving calibration %lu: %s", (unsigned long)cal->index, esp_err_to_name(err));
    }
}

static void persist_robot_type(void *ctx, const char *robot_type)
{
    const esp_err_t err = nvs_store_save_robot_type(robot_type);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "saving robot type: %s", esp_err_to_name(err));
    }
}

/* ---- the link: bytes in, to the dialect or the protobuf decoder ------------------ */

static void on_payload(void *ctx, const uint8_t *payload, size_t n)
{
    scorbot_v1_Envelope e;
    if (sc_envelope_decode(payload, n, &e)) {
        sc_core_handle(&s_core, &e);
    }
}

static void on_link_bytes(const uint8_t *buf, size_t n)
{
    size_t k = 0;
    if (s_mode == SC_LINK_UNDECIDED) {
        /* Auto mode. The first bytes decide; a 921600 stream read at 9600 shows up as
           framing errors long before it shows a delimiter. */
        link_uart_stats_t st;
        link_uart_stats(&st);
        if (st.frame_errors >= 3) {
            s_mode = SC_LINK_PROTOBUF;
            link_uart_set_baud(CONFIG_SC_LINK_PROTOBUF_BAUD);
            sc_link_sniffer_init(&s_sniffer);
            ESP_LOGI(TAG, "link: protobuf (framing errors at %d baud)", CONFIG_SC_LINK_ACL_BAUD);
            return; /* what was read at the wrong rate is garbage */
        }
        size_t used = 0;
        s_mode = sc_link_sniffer_feed(&s_sniffer, buf, n, &used);
        if (s_mode == SC_LINK_UNDECIDED) {
            return;
        }
        uint8_t held[SC_LINK_SNIFF_MAX];
        const size_t h = sc_link_sniffer_take(&s_sniffer, held, sizeof(held));
        ESP_LOGI(TAG, "link: %s", s_mode == SC_LINK_ACL ? "Eshed dialect" : "protobuf");
        if (s_mode == SC_LINK_ACL) {
            sc_acl_feed(&s_acl, held, h);
        } else {
            link_uart_set_baud(CONFIG_SC_LINK_PROTOBUF_BAUD);
            sc_stream_decoder_feed(&s_decoder, held, h, on_payload, NULL);
        }
        k = used;
    }
    if (s_mode == SC_LINK_ACL) {
        sc_acl_feed(&s_acl, buf + k, n - k);
    } else {
        sc_stream_decoder_feed(&s_decoder, buf + k, n - k, on_payload, NULL);
    }
}

/* ---- control loop -------------------------------------------------------------- */

static void IRAM_ATTR on_timer(void *arg)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_control_task, &woken);
    portYIELD_FROM_ISR(woken);
}

static void control_task(void *arg)
{
    sc_input_t in;
    sc_output_t out;
    limit_switch_snapshot_t sw;
    current_snapshot_t cur;
    scorbot_v1_Envelope e;
    uint32_t bus_error_ticks = 0;

    s_last_tick_us = esp_timer_get_time();
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        const int64_t now = esp_timer_get_time();
        float dt = (float)(now - s_last_tick_us) * 1e-6f;
        s_last_tick_us = now;
        if (dt > 0.05f) {
            dt = 0.05f; /* a stall of the task (flash write, log burst): don't integrate it */
        }

        /* Inbound: everything that arrived since the last tick. */
        uint8_t rx[256];
        size_t rn;
        while ((rn = link_uart_read(rx, sizeof(rx))) > 0) {
            on_link_bytes(rx, rn);
        }

        /* Inputs. */
        memset(&in, 0, sizeof(in));
        limit_switch_collect(&sw);
        current_collect(&cur);
        for (int i = 0; i < JOINTS; ++i) {
            int c = 0;
            encoder_get_raw_count(&s_encoders[i], &c);
            in.joint[i].counts = c;
            in.joint[i].limit_switch = sw.joint[i].pressed;
            in.joint[i].switch_edge = sw.joint[i].edge;
            in.joint[i].latched_counts = sw.joint[i].latched_count;
            in.joint[i].current_a = cur.current_a[i];
            in.joint[i].driver_fault = (sw.fault_mask >> i) & 1u;
        }
        in.motor_power_valid = !cur.bus_error;
        in.motor_power = cur.rail_ok;
        if (sw.bus_error || cur.bus_error) {
            if (++bus_error_ticks == 50) { /* 50 ms of a dead I2C bus: the drivers are blind */
                sc_core_raise_fault(&s_core, scorbot_v1_FaultCode_FAULT_DRIVER, -1, false);
            }
        } else {
            bus_error_ticks = 0;
        }

        sc_core_tick(&s_core, dt, &in, &out);

        /* Outputs. */
        if (motor_apply(&s_motor, out.duty, out.brake, out.drives_enabled) != ESP_OK) {
            sc_core_raise_fault(&s_core, scorbot_v1_FaultCode_FAULT_DRIVER, -1, false);
        }
        current_set_duty(out.duty); /* the current task divides its next conversions by these */

        /* Outbound. The dialect drives the core itself (its command feeds the watchdog)
           and eats the core's stream; protobuf forwards it: JointState at 100 Hz,
           responses, events. */
        if (s_mode == SC_LINK_ACL) {
            sc_acl_tick(&s_acl, dt);
            uint8_t text[128];
            size_t tn;
            while ((tn = sc_acl_read(&s_acl, text, sizeof(text))) > 0) {
                link_uart_write(text, tn);
            }
        } else {
            while (sc_core_pop(&s_core, &e)) {
                if (s_mode == SC_LINK_PROTOBUF) {
                    link_uart_send(&e);
                }
            }
        }
    }
}

/* ---- boot -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    /* Persisted state first: boot count, robot type, calibration. */
    uint32_t boot_count = 0;
    ESP_ERROR_CHECK(nvs_store_init(&boot_count));
    sc_core_config_default(&s_cfg);
    s_cfg.firmware_version = FIRMWARE_VERSION;
    s_cfg.joint_count = JOINTS;
    for (int i = 0; i < JOINTS; ++i) {
        s_cfg.joint_names[i] = kJointNames[i];
        s_cfg.has_calibration[i] = nvs_store_load_calibration((uint32_t)i, &s_cfg.calibration[i]);
    }
    if (!nvs_store_load_robot_type(s_cfg.robot_type, sizeof(s_cfg.robot_type))) {
        strncpy(s_cfg.robot_type, "unset", sizeof(s_cfg.robot_type) - 1);
    }
    s_cfg.boot_count = boot_count;
    s_cfg.persist.calibration = persist_calibration;
    s_cfg.persist.robot_type = persist_robot_type;
    s_cfg.watchdog_ms = CONFIG_SC_WATCHDOG_MS;
    s_cfg.require_calibration = true;
    s_cfg.total_current_limit_a = CONFIG_SC_TOTAL_CURRENT_LIMIT_DA / 10.0f;
    sc_core_init(&s_core, &s_cfg);

    /* The Eshed dialect on top of the core: virtual ER-V counts, sign per joint from Kconfig. */
    sc_acl_config_t ac;
    sc_acl_config_default(&ac);
    for (int i = 0; i < JOINTS; ++i) {
        if ((CONFIG_SC_ACL_SIGN_MASK >> i) & 1) {
            ac.virtual_sign[i] = -1.0f;
        }
    }
    sc_acl_init(&s_acl, &s_core, &ac);
    sc_link_sniffer_init(&s_sniffer);
    sc_stream_decoder_init(&s_decoder);
#if CONFIG_SC_LINK_MODE_PROTOBUF
    s_mode = SC_LINK_PROTOBUF;
    const int link_baud = CONFIG_SC_LINK_PROTOBUF_BAUD;
#elif CONFIG_SC_LINK_MODE_AUTO
    s_mode = SC_LINK_UNDECIDED;
    const int link_baud = CONFIG_SC_LINK_ACL_BAUD;
#else
    s_mode = SC_LINK_ACL;
    const int link_baud = CONFIG_SC_LINK_ACL_BAUD;
#endif

    /* The link comes up before the hardware so a failed self-test is still reported. */
    const link_uart_config_t link = {.uart_num = CONFIG_SC_LINK_UART_NUM, .baud = link_baud,
                                     .tx_gpio = CONFIG_SC_LINK_TX_GPIO, .rx_gpio = CONFIG_SC_LINK_RX_GPIO};
    ESP_ERROR_CHECK(link_uart_init(&link));

    /* I2C bus: expander, two ADCs. */
    i2c_bus_t bus;
    const i2c_bus_config_t bus_cfg = {.port = I2C_NUM_0, .sda_io_num = CONFIG_SC_I2C_SDA_GPIO, .scl_io_num = CONFIG_SC_I2C_SCL_GPIO};
    ESP_ERROR_CHECK(i2c_bus_init(&bus, &bus_cfg));

    bool hardware_ok = true;
    esp_err_t err = expander_init(&s_expander, bus.handle, CONFIG_SC_EXPANDER_ADDRESS, CONFIG_SC_I2C_SPEED_HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MCP23017: %s", esp_err_to_name(err));
        hardware_ok = false;
    }

    /* Encoders on the pulse counters. */
    const int enc_pins[JOINTS][2] = {
        {CONFIG_SC_ENC1_P0_GPIO, CONFIG_SC_ENC1_P1_GPIO}, {CONFIG_SC_ENC2_P0_GPIO, CONFIG_SC_ENC2_P1_GPIO},
        {CONFIG_SC_ENC3_P0_GPIO, CONFIG_SC_ENC3_P1_GPIO}, {CONFIG_SC_ENC4_P0_GPIO, CONFIG_SC_ENC4_P1_GPIO},
        {CONFIG_SC_ENC5_P0_GPIO, CONFIG_SC_ENC5_P1_GPIO},
    };
    for (int i = 0; i < JOINTS; ++i) {
        encoder_config_t ec = {
            .P0_pin = enc_pins[i][0],
            .P1_pin = enc_pins[i][1],
            .glitch_filter_ns = CONFIG_SC_ENC_GLITCH_NS,
            .resolution = 20,
            .gear_ratio = 1.0f,
            .limb_default = 0.0f,
            .invert_angle = 0, /* encoder_invert is applied in the core from the calibration */
        };
        ESP_ERROR_CHECK(encoder_init(&s_encoders[i], &ec));
        ESP_ERROR_CHECK(encoder_start(&s_encoders[i]));
    }

    /* Motors: LEDC PWM, DIR through the expander, the shared SLP on its own GPIO (low
       until the core says so). */
    motor_config_t mc = {
        .pwm_gpio = {CONFIG_SC_PWM1_GPIO, CONFIG_SC_PWM2_GPIO, CONFIG_SC_PWM3_GPIO, CONFIG_SC_PWM4_GPIO, CONFIG_SC_PWM5_GPIO},
        .pwm_freq_hz = CONFIG_SC_PWM_FREQ_HZ,
        .slp_gpio = CONFIG_SC_SLP_GPIO,
        .expander = &s_expander,
    };
    err = motor_init(&s_motor, &mc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor: %s", esp_err_to_name(err));
        hardware_ok = false;
    }

    /* Switches and driver faults through the expander, with the encoder latch. */
    limit_switch_config_t lc = {
        .int_gpio = CONFIG_SC_EXPANDER_INT_GPIO,
#ifdef CONFIG_SC_SWITCH_ACTIVE_LOW
        .active_low = true,
#else
        .active_low = false,
#endif
        .expander = &s_expander,
        .debounce_us = 2000,
    };
    for (int i = 0; i < JOINTS; ++i) {
        lc.encoders[i] = &s_encoders[i];
    }
    err = limit_switch_init(&lc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "limit_switch: %s", esp_err_to_name(err));
        hardware_ok = false;
    }

    /* Currents and the motor rail. */
    const current_config_t cc = {
        .bus = bus.handle,
        .addr_a = CONFIG_SC_ADC_A_ADDRESS,
        .addr_b = CONFIG_SC_ADC_B_ADDRESS,
        .speed_hz = CONFIG_SC_I2C_SPEED_HZ,
        .mv_per_a = CONFIG_SC_CS_MV_PER_A,
        .offset_mv = CONFIG_SC_CS_OFFSET_MV,
        .vsense_ratio = CONFIG_SC_VSENSE_RATIO_X100 / 100.0f,
        .rail_ok_v = CONFIG_SC_RAIL_OK_DV / 10.0f,
        .min_duty = CONFIG_SC_CS_MIN_DUTY_PCT / 100.0f,
    };
    err = current_init(&cc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADS1015: %s", esp_err_to_name(err));
        hardware_ok = false;
    }

    if (!hardware_ok) {
        /* Self-test failed: BOOT -> FAULT, reported on the link, drivers never enabled. */
        sc_core_raise_fault(&s_core, scorbot_v1_FaultCode_FAULT_DRIVER, -1, false);
    }

    xTaskCreatePinnedToCore(control_task, "control", 8192, NULL, 20, &s_control_task, 1);
    const esp_timer_create_args_t targs = {.callback = on_timer, .name = "control", .dispatch_method = ESP_TIMER_ISR};
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&targs, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000));

    ESP_LOGI(TAG, "firmware %s, boot %lu, robot %s, link %s, %s", FIRMWARE_VERSION, (unsigned long)boot_count, s_cfg.robot_type,
             s_mode == SC_LINK_ACL ? "Eshed dialect" : s_mode == SC_LINK_PROTOBUF ? "protobuf" : "auto",
             hardware_ok ? "hardware ok" : "HARDWARE FAULT");
}
