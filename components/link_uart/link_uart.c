#include "link_uart.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "scorbot_proto/framing.h"

static const char *TAG = "link_uart";

#define RX_BUF 2048
#define TX_BUF 4096
#define STREAM_BUF 2048
#define EVENT_QUEUE 16

static link_uart_config_t s_cfg;
static QueueHandle_t s_events;
static StreamBufferHandle_t s_rx;
static link_uart_stats_t s_stats;

static void rx_task(void *arg)
{
    uart_event_t ev;
    uint8_t buf[256];
    for (;;) {
        if (xQueueReceive(s_events, &ev, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }
        switch (ev.type) {
        case UART_DATA: {
            int n;
            while ((n = uart_read_bytes(s_cfg.uart_num, buf, sizeof(buf), 0)) > 0) {
                const size_t sent = xStreamBufferSend(s_rx, buf, (size_t)n, 0);
                s_stats.rx_bytes += (uint32_t)sent;
                s_stats.rx_dropped += (uint32_t)n - (uint32_t)sent;
            }
            break;
        }
        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
            ++s_stats.frame_errors;
            break;
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            ++s_stats.overflows;
            uart_flush_input(s_cfg.uart_num);
            xQueueReset(s_events);
            break;
        default:
            break;
        }
    }
}

esp_err_t link_uart_init(const link_uart_config_t *cfg)
{
    s_cfg = *cfg;
    s_rx = xStreamBufferCreate(STREAM_BUF, 1);
    if (!s_rx) {
        return ESP_ERR_NO_MEM;
    }
    const uart_config_t uc = {
        .baud_rate = cfg->baud ? cfg->baud : 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, /* the operator uses XON/XOFF, which the dialect never emits */
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(cfg->uart_num, RX_BUF, TX_BUF, EVENT_QUEUE, &s_events, 0);
    if (err != ESP_OK) {
        return err;
    }
    if ((err = uart_param_config(cfg->uart_num, &uc)) != ESP_OK) {
        return err;
    }
    if (cfg->tx_gpio >= 0 && cfg->rx_gpio >= 0) {
        if ((err = uart_set_pin(cfg->uart_num, cfg->tx_gpio, cfg->rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) != ESP_OK) {
            return err;
        }
    }
    xTaskCreatePinnedToCore(rx_task, "link_rx", 4096, NULL, 10, NULL, 0);
    ESP_LOGI(TAG, "UART%d at %d baud", cfg->uart_num, uc.baud_rate);
    return ESP_OK;
}

size_t link_uart_read(uint8_t *out, size_t cap) { return xStreamBufferReceive(s_rx, out, cap, 0); }

bool link_uart_write(const uint8_t *data, size_t n)
{
    size_t free_bytes = 0;
    if (n == 0) {
        return true;
    }
    if (uart_get_tx_buffer_free_size(s_cfg.uart_num, &free_bytes) != ESP_OK || free_bytes < n) {
        s_stats.tx_dropped += (uint32_t)n;
        return false;
    }
    if (uart_write_bytes(s_cfg.uart_num, data, n) != (int)n) {
        s_stats.tx_dropped += (uint32_t)n;
        return false;
    }
    s_stats.tx_bytes += (uint32_t)n;
    return true;
}

bool link_uart_send(const scorbot_v1_Envelope *e)
{
    uint8_t frame[SC_MAX_FRAME];
    const size_t len = sc_envelope_to_frame(e, frame, sizeof(frame));
    if (len == 0) {
        ++s_stats.tx_dropped;
        return false;
    }
    return link_uart_write(frame, len);
}

esp_err_t link_uart_set_baud(int baud)
{
    const esp_err_t err = uart_set_baudrate(s_cfg.uart_num, (uint32_t)baud);
    if (err == ESP_OK) {
        s_cfg.baud = baud;
        ESP_LOGI(TAG, "UART%d now at %d baud", s_cfg.uart_num, baud);
    }
    return err;
}

void link_uart_stats(link_uart_stats_t *out) { *out = s_stats; }
