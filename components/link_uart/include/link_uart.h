// The serial link to the Pi on UART0 (the DevKit's USB): 921600 8N1, COBS frames.
// An RX task feeds the stream decoder and queues decoded Envelopes for the control
// loop; TX goes straight into the UART driver's ring buffer and is dropped, counted,
// if that is full, so the control loop never blocks on the link.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "scorbot_proto/envelope.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int uart_num;      /* 0 */
    int baud;          /* 921600 */
    int tx_gpio;       /* -1 = keep the default pins (1/3 on UART0) */
    int rx_gpio;
} link_uart_config_t;

typedef struct {
    uint32_t rx_envelopes, rx_decode_errors, rx_dropped;
    uint32_t tx_envelopes, tx_dropped;
    uint32_t crc_errors, cobs_errors;
} link_uart_stats_t;

esp_err_t link_uart_init(const link_uart_config_t *cfg);
/// Next decoded inbound Envelope, if any (non-blocking).
bool link_uart_receive(scorbot_v1_Envelope *out);
/// Queue an Envelope for transmission (non-blocking; false if dropped).
bool link_uart_send(const scorbot_v1_Envelope *e);
void link_uart_stats(link_uart_stats_t *out);

#ifdef __cplusplus
}
#endif
