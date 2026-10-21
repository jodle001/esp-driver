#include "encoder_ls7366r.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "soc/soc_caps.h"

static const char *TAG = "ls7366r";

#define OP_CLR 0x00
#define OP_RD 0x40
#define OP_WR 0x80
#define REG_MDR0 (1u << 3)
#define REG_MDR1 (2u << 3)
#define REG_CNTR (4u << 3)
#define REG_OTR (5u << 3)
#define MDR0_X4_FREE_RUNNING 0x03 /* bits 1:0 = 11 x4 quadrature, 3:2 = 00 free running, 6 = 0 asynchronous index, 7 = 0 filter /1 */
#define MDR0_INDEX_LOAD_OTR 0x30  /* bits 5:4 = 11: the index edge transfers CNTR into OTR */
#define MDR1_4_BYTES_ENABLED 0x00 /* bits 1:0 = 00 4-byte counter, 2 = 0 counting enabled, no flags */

#if SOC_LEDC_SUPPORT_HS_MODE
#define CLK_SPEED_MODE LEDC_HIGH_SPEED_MODE
#else
#define CLK_SPEED_MODE LEDC_LOW_SPEED_MODE
#endif

static ls7366r_config_t s_cfg;
static spi_device_handle_t s_dev;
static SemaphoreHandle_t s_mutex;

static esp_err_t xfer(int chip, const uint8_t *tx, uint8_t *rx, size_t n)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = n * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    gpio_set_level(s_cfg.cs_gpio[chip], 0);
    const esp_err_t err = spi_device_polling_transmit(s_dev, &t);
    gpio_set_level(s_cfg.cs_gpio[chip], 1);
    return err;
}

static esp_err_t write_reg(int chip, uint8_t reg, uint8_t value)
{
    const uint8_t tx[2] = {(uint8_t)(OP_WR | reg), value};
    uint8_t rx[2];
    return xfer(chip, tx, rx, sizeof(tx));
}

static esp_err_t read_reg8(int chip, uint8_t reg, uint8_t *value)
{
    const uint8_t tx[2] = {(uint8_t)(OP_RD | reg), 0};
    uint8_t rx[2] = {0, 0};
    const esp_err_t err = xfer(chip, tx, rx, sizeof(tx));
    *value = rx[1];
    return err;
}

static esp_err_t read_reg32(int chip, uint8_t reg, int32_t *count)
{
    const uint8_t tx[5] = {(uint8_t)(OP_RD | reg), 0, 0, 0, 0};
    uint8_t rx[5] = {0};
    const esp_err_t err = xfer(chip, tx, rx, sizeof(tx));
    if (err == ESP_OK) {
        *count = (int32_t)(((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 8) | rx[4]);
    }
    return err;
}

static esp_err_t filter_clock_init(const ls7366r_config_t *cfg)
{
    if (cfg->clk_gpio < 0) {
        return ESP_OK;
    }
    const ledc_timer_config_t timer = {
        .speed_mode = CLK_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_2_BIT, /* 4 steps: 80 MHz / (4 MHz x 4) = divider 5 */
        .timer_num = LEDC_TIMER_1,
        .freq_hz = cfg->clk_hz ? cfg->clk_hz : 4000000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        return err;
    }
    const ledc_channel_config_t ch = {
        .gpio_num = cfg->clk_gpio,
        .speed_mode = CLK_SPEED_MODE,
        .channel = (ledc_channel_t)cfg->ledc_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 2, /* 50 % */
        .hpoint = 0,
    };
    return ledc_channel_config(&ch);
}

esp_err_t ls7366r_init(const ls7366r_config_t *cfg)
{
    if (!cfg || cfg->count < 1 || cfg->count > LS7366R_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;
    if (s_cfg.spi_hz == 0) {
        s_cfg.spi_hz = 4000000; /* 120 ns SCK high and low at 3.3 V: 4.16 MHz is the limit */
    }
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        return ESP_ERR_NO_MEM;
    }
    /* Chip selects high (deselected) before the bus comes up. */
    uint64_t mask = 0;
    for (int i = 0; i < cfg->count; ++i) {
        mask |= 1ULL << cfg->cs_gpio[i];
        gpio_set_level(cfg->cs_gpio[i], 1);
    }
    const gpio_config_t io = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = mask,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < cfg->count; ++i) {
        gpio_set_level(cfg->cs_gpio[i], 1);
    }
    if ((err = filter_clock_init(cfg)) != ESP_OK) {
        return err;
    }
    const spi_bus_config_t bus = {
        .mosi_io_num = cfg->mosi_gpio,
        .miso_io_num = cfg->miso_gpio,
        .sclk_io_num = cfg->sclk_gpio,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16,
    };
    if ((err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED)) != ESP_OK) {
        return err;
    }
    const spi_device_interface_config_t dev = {
        .clock_speed_hz = (int)s_cfg.spi_hz,
        .mode = 0,
        .spics_io_num = -1, /* the chip selects are plain GPIOs: seven of them, more than the host has CS lines */
        .queue_size = 1,
    };
    if ((err = spi_bus_add_device(SPI2_HOST, &dev, &s_dev)) != ESP_OK) {
        return err;
    }
    /* Configure and verify every chip: x4, free running, 4-byte counter, count cleared,
       the index latch into OTR if asked for. */
    const uint8_t mdr0_want = MDR0_X4_FREE_RUNNING | (s_cfg.index_latch ? MDR0_INDEX_LOAD_OTR : 0);
    for (int i = 0; i < cfg->count; ++i) {
        uint8_t mdr0 = 0xFF;
        if ((err = write_reg(i, REG_MDR0, mdr0_want)) != ESP_OK) return err;
        if ((err = write_reg(i, REG_MDR1, MDR1_4_BYTES_ENABLED)) != ESP_OK) return err;
        if ((err = read_reg8(i, REG_MDR0, &mdr0)) != ESP_OK) return err;
        if (mdr0 != mdr0_want) {
            ESP_LOGE(TAG, "counter %d (CS GPIO %d) does not answer: MDR0 read 0x%02x", i + 1, cfg->cs_gpio[i], mdr0);
            return ESP_ERR_NOT_FOUND;
        }
        const uint8_t clr = OP_CLR | REG_CNTR;
        uint8_t rx;
        if ((err = xfer(i, &clr, &rx, 1)) != ESP_OK) return err;
    }
    ESP_LOGI(TAG, "%d counters on SPI2 at %lu Hz, filter clock %lu Hz on GPIO %d, index latch %s", cfg->count,
             (unsigned long)s_cfg.spi_hz, (unsigned long)(cfg->clk_hz ? cfg->clk_hz : 4000000), cfg->clk_gpio,
             s_cfg.index_latch ? "in the counters (OTR)" : "in software");
    return ESP_OK;
}

esp_err_t ls7366r_read_all(int32_t *counts)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = ESP_OK;
    for (int i = 0; i < s_cfg.count && err == ESP_OK; ++i) {
        err = read_reg32(i, REG_CNTR, &counts[i]);
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t ls7366r_read_latch_all(int32_t *counts)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = ESP_OK;
    const uint8_t reg = s_cfg.index_latch ? REG_OTR : REG_CNTR;
    for (int i = 0; i < s_cfg.count && err == ESP_OK; ++i) {
        err = read_reg32(i, reg, &counts[i]);
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t ls7366r_read(int chip, int32_t *count)
{
    if (chip < 0 || chip >= s_cfg.count) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const esp_err_t err = read_reg32(chip, REG_CNTR, count);
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t ls7366r_clear(int chip)
{
    if (chip < 0 || chip >= s_cfg.count) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const uint8_t clr = OP_CLR | REG_CNTR;
    uint8_t rx;
    const esp_err_t err = xfer(chip, &clr, &rx, 1);
    xSemaphoreGive(s_mutex);
    return err;
}
