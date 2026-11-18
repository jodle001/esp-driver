#include "pixel_strip.h"

#include <string.h>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "pixel_strip";

#define RMT_RESOLUTION_HZ 20000000 /* 50 ns per tick */
#define NS_PER_TICK 50
#define FRAME_BYTES (PIXEL_STRIP_MAX * 3)

typedef struct {
    uint8_t rgb[FRAME_BYTES];
} frame_t;

static pixel_strip_config_t s_cfg;
static rmt_channel_handle_t s_channel;
static rmt_encoder_handle_t s_encoder;
static QueueHandle_t s_queue; /* one slot: the newest frame wins */
static uint8_t s_wire[FRAME_BYTES]; /* must stay valid until the transmission is done */

static void strip_task(void *arg)
{
    const rmt_transmit_config_t tx = {.loop_count = 0};
    frame_t f;
    for (;;) {
        if (xQueueReceive(s_queue, &f, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        for (int i = 0; i < s_cfg.count; ++i) {
            const uint8_t r = f.rgb[3 * i], g = f.rgb[3 * i + 1], b = f.rgb[3 * i + 2];
            s_wire[3 * i] = s_cfg.grb ? g : r;
            s_wire[3 * i + 1] = s_cfg.grb ? r : g;
            s_wire[3 * i + 2] = b;
        }
        if (rmt_transmit(s_channel, s_encoder, s_wire, (size_t)s_cfg.count * 3, &tx) == ESP_OK) {
            rmt_tx_wait_all_done(s_channel, 100);
        }
        /* The line now idles low until the next frame, tens of milliseconds later: far
           longer than the reset time of any of these pixels. */
    }
}

esp_err_t pixel_strip_init(const pixel_strip_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->count >= 1 && cfg->count <= PIXEL_STRIP_MAX, ESP_ERR_INVALID_ARG, TAG, "count");
    ESP_RETURN_ON_FALSE(cfg->t0h_ns && cfg->t0l_ns && cfg->t1h_ns && cfg->t1l_ns, ESP_ERR_INVALID_ARG, TAG, "timing");
    s_cfg = *cfg;

    const rmt_tx_channel_config_t ch = {
        .gpio_num = cfg->gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&ch, &s_channel), TAG, "rmt channel");

    const rmt_bytes_encoder_config_t enc = {
        .bit0 = {.level0 = 1, .duration0 = cfg->t0h_ns / NS_PER_TICK, .level1 = 0, .duration1 = cfg->t0l_ns / NS_PER_TICK},
        .bit1 = {.level0 = 1, .duration0 = cfg->t1h_ns / NS_PER_TICK, .level1 = 0, .duration1 = cfg->t1l_ns / NS_PER_TICK},
        .flags.msb_first = 1,
    };
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&enc, &s_encoder), TAG, "rmt encoder");
    ESP_RETURN_ON_ERROR(rmt_enable(s_channel), TAG, "rmt enable");

    s_queue = xQueueCreate(1, sizeof(frame_t));
    ESP_RETURN_ON_FALSE(s_queue, ESP_ERR_NO_MEM, TAG, "queue");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(strip_task, "pixels", 3072, NULL, 3, NULL, 0) == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "task");

    /* The first frame after boot is dark. */
    const frame_t dark = {0};
    xQueueOverwrite(s_queue, &dark);
    return ESP_OK;
}

void pixel_strip_show(const uint8_t *rgb)
{
    if (!s_queue) {
        return;
    }
    frame_t f = {0};
    memcpy(f.rgb, rgb, (size_t)s_cfg.count * 3);
    xQueueOverwrite(s_queue, &f);
}
