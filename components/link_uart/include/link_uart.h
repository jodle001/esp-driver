// The serial link to the Pi: UART0, the DevKit's USB (CP2102), since Oct 16 2026 the
// rev 0.3 board's only connection to the Pi. It carries either the Eshed dialect (ACL
// text at 9600, the operator) or the protobuf framing (921600, the ROS 2 stack); which
// one is decided in main/esp_driver.c from Kconfig or from the first bytes, so this
// component moves raw bytes: an RX task drains the UART driver into a stream buffer the
// control loop reads without blocking, and TX goes straight into the driver's ring
// buffer, dropped and counted when that is full, so the control loop never blocks on
// the link. Framing errors are counted (a 921600 stream read at 9600 is all of them).
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "scorbot_proto/envelope.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int uart_num;      /* 0 = the DevKit's USB */
    int baud;          /* the starting rate: 9600 for the dialect, 921600 for protobuf */
    int tx_gpio;       /* -1 = keep the UART's default pins (1/3 on UART0) */
    int rx_gpio;
} link_uart_config_t;

typedef struct {
    uint32_t rx_bytes, rx_dropped;   /* dropped: the control loop was not keeping up */
    uint32_t tx_bytes, tx_dropped;
    uint32_t frame_errors, overflows;
} link_uart_stats_t;

esp_err_t link_uart_init(const link_uart_config_t *cfg);
/// Bytes received since the last call (non-blocking; returns how many were copied).
size_t link_uart_read(uint8_t *out, size_t cap);
/// Queue raw bytes for transmission (non-blocking; false if dropped for lack of room).
bool link_uart_write(const uint8_t *data, size_t n);
/// Frame and queue a protobuf Envelope (non-blocking; false if dropped).
bool link_uart_send(const scorbot_v1_Envelope *e);
/// Change the rate (the auto mode's switch from 9600 to 921600).
esp_err_t link_uart_set_baud(int baud);
void link_uart_stats(link_uart_stats_t *out);

#ifdef __cplusplus
}
#endif
