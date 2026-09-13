#include "scorbot_proto/framing.h"

#include <string.h>

#include "scorbot_proto/crc16.h"

size_t sc_encode_frame(const uint8_t *payload, size_t n, uint8_t *out, size_t cap)
{
    if (n > SC_MAX_PAYLOAD || cap == 0) {
        return 0;
    }
    uint8_t with_crc[SC_MAX_PAYLOAD + 2];
    if (n) {
        memcpy(with_crc, payload, n);
    }
    const uint16_t crc = sc_crc16(payload, n);
    with_crc[n] = (uint8_t)(crc >> 8);
    with_crc[n + 1] = (uint8_t)(crc & 0xFF);

    const size_t encoded = sc_cobs_encode(with_crc, n + 2, out, cap - 1);
    if (encoded == 0) {
        return 0;
    }
    out[encoded] = SC_DELIMITER;
    return encoded + 1;
}

void sc_stream_decoder_init(sc_stream_decoder_t *d)
{
    memset(d, 0, sizeof(*d));
}

static void finish_frame(sc_stream_decoder_t *d, sc_payload_fn on_payload, void *ctx)
{
    const size_t decoded = sc_cobs_decode(d->buf, d->len, d->payload, sizeof(d->payload));
    if (decoded == SC_COBS_NPOS) {
        ++d->stats.cobs_errors;
        return;
    }
    if (decoded < 2) {
        ++d->stats.short_frames;
        return;
    }
    const size_t n = decoded - 2;
    const uint16_t expected = (uint16_t)(((uint16_t)d->payload[n] << 8) | d->payload[n + 1]);
    if (sc_crc16(d->payload, n) != expected) {
        ++d->stats.crc_errors;
        return;
    }
    ++d->stats.frames;
    on_payload(ctx, d->payload, n);
}

void sc_stream_decoder_feed(sc_stream_decoder_t *d, const uint8_t *data, size_t n, sc_payload_fn on_payload,
                            void *ctx)
{
    d->stats.bytes += (uint32_t)n;
    for (size_t i = 0; i < n; ++i) {
        const uint8_t b = data[i];
        if (b != SC_DELIMITER) {
            if (d->len < sizeof(d->buf)) {
                d->buf[d->len++] = b;
            } else {
                d->overflow = true;
            }
            continue;
        }
        if (d->overflow) {
            ++d->stats.overflows;
        } else if (d->len > 0) {
            finish_frame(d, on_payload, ctx);
        }
        /* len == 0: back-to-back delimiters, idle line; nothing to do. */
        d->len = 0;
        d->overflow = false;
    }
}
