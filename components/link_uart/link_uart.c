#include "link_uart.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "scorbot_proto/framing.h"

static const char *TAG = "link_uart";

#define RX_BUF 2048
#define TX_BUF 4096
#define INBOUND_DEPTH 8

static link_uart_config_t s_cfg;
static QueueHandle_t s_inbound;
static link_uart_stats_t s_stats;
static sc_stream_decoder_t s_decoder;

static void on_payload(void *ctx, const uint8_t *payload, size_t n)
{
    scorbot_v1_Envelope e;
    if (!sc_envelope_decode(payload, n, &e)) {
        ++s_stats.rx_decode_errors;
        return;
    }
    ++s_stats.rx_envelopes;
    if (xQueueSend(s_inbound, &e, 0) != pdTRUE) {
        ++s_stats.rx_dropped; /* the control loop is not keeping up; newest command lost */
    }
}

static void rx_task(void *arg)
{
    uint8_t buf[256];
    for (;;) {
        const int n = uart_read_bytes(s_cfg.uart_num, buf, sizeof(buf), pdMS_TO_TICKS(5));
        if (n > 0) {
            sc_stream_decoder_feed(&s_decoder, buf, (size_t)n, on_payload, NULL);
            s_stats.crc_errors = s_decoder.stats.crc_errors;
            s_stats.cobs_errors = s_decoder.stats.cobs_errors;
        }
    }
}

esp_err_t link_uart_init(const link_uart_config_t *cfg)
{
    s_cfg = *cfg;
    sc_stream_decoder_init(&s_decoder);
    s_inbound = xQueueCreate(INBOUND_DEPTH, sizeof(scorbot_v1_Envelope));
    if (!s_inbound) {
        return ESP_ERR_NO_MEM;
    }
    const uart_config_t uc = {
        .baud_rate = cfg->baud ? cfg->baud : 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(cfg->uart_num, RX_BUF, TX_BUF, 0, NULL, 0);
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

bool link_uart_receive(scorbot_v1_Envelope *out)
{
    return xQueueReceive(s_inbound, out, 0) == pdTRUE;
}

bool link_uart_send(const scorbot_v1_Envelope *e)
{
    uint8_t frame[SC_MAX_FRAME];
    const size_t len = sc_envelope_to_frame(e, frame, sizeof(frame));
    size_t free_bytes = 0;
    if (len == 0 || uart_get_tx_buffer_free_size(s_cfg.uart_num, &free_bytes) != ESP_OK || free_bytes < len) {
        ++s_stats.tx_dropped;
        return false;
    }
    if (uart_write_bytes(s_cfg.uart_num, frame, len) != (int)len) {
        ++s_stats.tx_dropped;
        return false;
    }
    ++s_stats.tx_envelopes;
    return true;
}

void link_uart_stats(link_uart_stats_t *out) { *out = s_stats; }
