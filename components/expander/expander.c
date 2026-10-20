#include "expander.h"

#define TIMEOUT_MS 20

/* BANK = 0 register map */
#define REG_IODIRA 0x00
#define REG_IODIRB 0x01
#define REG_GPINTENA 0x04
#define REG_GPINTENB 0x05
#define REG_INTCONA 0x08
#define REG_INTCONB 0x09
#define REG_IOCON 0x0A
#define REG_GPPUA 0x0C
#define REG_GPPUB 0x0D
#define REG_INTFA 0x0E
#define REG_INTCAPA 0x10
#define REG_GPIOA 0x12
#define REG_OLATA 0x14
#define REG_OLATB 0x15

#define IOCON_MIRROR 0x40 /* INTA and INTB are internally connected */
#define IOCON_ODR 0x04    /* INT pin open-drain: the two chips share one line */

static esp_err_t wr(expander_t *x, uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(x->dev, buf, sizeof(buf), TIMEOUT_MS);
}

static esp_err_t rd(expander_t *x, uint8_t reg, uint8_t *out, size_t n)
{
    return i2c_master_transmit_receive(x->dev, &reg, 1, out, n, TIMEOUT_MS);
}

esp_err_t expander_init(expander_t *x, i2c_master_bus_handle_t bus, uint8_t address, uint32_t speed_hz, expander_role_t role)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = speed_hz,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &x->dev);
    if (err != ESP_OK) {
        return err;
    }
    x->role = role;
    x->olat_a = 0;
    /* Outputs low before they become outputs: DIR don't-care (the drivers are asleep on
       the SLP line), the output-only GPA7/GPB7 driven low on both chips. */
    if ((err = wr(x, REG_OLATA, 0x00)) != ESP_OK) return err;
    if ((err = wr(x, REG_OLATB, 0x00)) != ESP_OK) return err;
    if (role == EXPANDER_ROLE_DIR) {
        if ((err = wr(x, REG_IODIRA, 0x00)) != ESP_OK) return err;  /* DIR1-7 out, GPA7 out (low) */
        if ((err = wr(x, REG_IODIRB, 0x7F)) != ESP_OK) return err;  /* GPB0 ESTOP_SENSE and spares in, GPB7 out */
        if ((err = wr(x, REG_GPPUA, 0x00)) != ESP_OK) return err;
        if ((err = wr(x, REG_GPPUB, 0x7E)) != ESP_OK) return err;   /* spares pulled up; ESTOP_SENSE has its divider */
        if ((err = wr(x, REG_GPINTENA, 0x00)) != ESP_OK) return err;
        if ((err = wr(x, REG_GPINTENB, EXPANDER_DIR_ESTOP_SENSE)) != ESP_OK) return err;
    } else {
        if ((err = wr(x, REG_IODIRA, 0x7F)) != ESP_OK) return err;  /* FLT1-7 in, GPA7 out */
        if ((err = wr(x, REG_IODIRB, 0x7F)) != ESP_OK) return err;  /* SW1-7 in, GPB7 out */
        if ((err = wr(x, REG_GPPUA, EXPANDER_CHANNEL_MASK)) != ESP_OK) return err; /* FLT: open-drain outputs on the modules */
        if ((err = wr(x, REG_GPPUB, 0x00)) != ESP_OK) return err;  /* the switches have the board's pull-ups */
        if ((err = wr(x, REG_GPINTENA, EXPANDER_CHANNEL_MASK)) != ESP_OK) return err;
        if ((err = wr(x, REG_GPINTENB, EXPANDER_CHANNEL_MASK)) != ESP_OK) return err;
    }
    if ((err = wr(x, REG_IOCON, IOCON_MIRROR | IOCON_ODR)) != ESP_OK) return err;
    if ((err = wr(x, REG_INTCONA, 0x00)) != ESP_OK) return err; /* interrupt on change against the previous value */
    if ((err = wr(x, REG_INTCONB, 0x00)) != ESP_OK) return err;
    uint8_t dummy[2];
    return rd(x, REG_INTCAPA, dummy, 2); /* clear any pending interrupt */
}

esp_err_t expander_write_port_a(expander_t *x, uint8_t value)
{
    if (value == x->olat_a) {
        return ESP_OK;
    }
    const esp_err_t err = wr(x, REG_OLATA, value);
    if (err == ESP_OK) {
        x->olat_a = value;
    }
    return err;
}

esp_err_t expander_read(expander_t *x, uint8_t *a, uint8_t *b)
{
    uint8_t buf[2];
    const esp_err_t err = rd(x, REG_GPIOA, buf, 2);
    if (err == ESP_OK) {
        *a = buf[0];
        *b = buf[1];
    }
    return err;
}

esp_err_t expander_read_interrupt(expander_t *x, uint8_t *intf_a, uint8_t *intf_b, uint8_t *cap_a, uint8_t *cap_b)
{
    uint8_t buf[4]; /* INTFA, INTFB, INTCAPA, INTCAPB are consecutive */
    const esp_err_t err = rd(x, REG_INTFA, buf, 4);
    if (err == ESP_OK) {
        *intf_a = buf[0];
        *intf_b = buf[1];
        *cap_a = buf[2];
        *cap_b = buf[3];
    }
    return err;
}
