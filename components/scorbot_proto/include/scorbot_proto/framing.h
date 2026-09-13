// Serial framing, C port of scorbot_protocol/framing.hpp:
//
//     frame := COBS(payload || crc16_be) || 0x00
//
// The payload is one encoded scorbot.v1.Envelope. sc_stream_decoder turns an arbitrary
// byte stream (split anywhere, with garbage or partial frames in it) back into
// validated payloads without allocating; the receiver rules are protocol.md section 2.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "scorbot_proto/cobs.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Largest payload (encoded Envelope) the framing accepts; scorbot_v1_Envelope_size
/// (906) must fit, checked in envelope.c.
#define SC_MAX_PAYLOAD 1024u
/// Payload plus CRC after COBS plus the delimiter.
#define SC_MAX_FRAME (SC_MAX_PAYLOAD + 2u + (SC_MAX_PAYLOAD + 2u) / 254u + 1u + 1u)
#define SC_DELIMITER 0x00u

/// Build a wire frame for payload into out (capacity cap), delimiter included.
/// Returns the frame length, or 0 if the payload is too large or out too small.
size_t sc_encode_frame(const uint8_t *payload, size_t n, uint8_t *out, size_t cap);

typedef struct {
    uint32_t frames;       ///< valid payloads delivered
    uint32_t bytes;        ///< raw bytes consumed
    uint32_t crc_errors;
    uint32_t cobs_errors;
    uint32_t short_frames; ///< decoded to fewer than 2 bytes (no room for a CRC)
    uint32_t overflows;    ///< frame longer than SC_MAX_FRAME, discarded up to the next delimiter
} sc_decoder_stats_t;

/// Called once per valid payload; the buffer is only valid during the call.
typedef void (*sc_payload_fn)(void *ctx, const uint8_t *payload, size_t n);

typedef struct {
    uint8_t buf[SC_MAX_FRAME];
    uint8_t payload[SC_MAX_PAYLOAD + 2];
    size_t len;
    bool overflow;
    sc_decoder_stats_t stats;
} sc_stream_decoder_t;

void sc_stream_decoder_init(sc_stream_decoder_t *d);
/// Feed n bytes; on_payload(ctx, ...) is called for each valid frame found.
void sc_stream_decoder_feed(sc_stream_decoder_t *d, const uint8_t *data, size_t n, sc_payload_fn on_payload,
                            void *ctx);

#ifdef __cplusplus
}
#endif
