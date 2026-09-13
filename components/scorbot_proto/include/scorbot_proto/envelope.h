// Envelope encode/decode over nanopb, plus the protocol version this firmware speaks.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "scorbot/v1/scorbot.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SC_PROTOCOL_MAJOR 1u
#define SC_PROTOCOL_MINOR 0u
#define SC_MAX_JOINTS 8u /* the nanopb max_count of every repeated joint field */

/// Serialize an Envelope into out (capacity cap). Returns the length, 0 on failure.
size_t sc_envelope_encode(const scorbot_v1_Envelope *e, uint8_t *out, size_t cap);

/// Parse an Envelope. Returns false on malformed input; *e is zeroed first.
bool sc_envelope_decode(const uint8_t *data, size_t len, scorbot_v1_Envelope *e);

/// Serialize an Envelope straight into a wire frame (COBS + CRC + delimiter).
/// Returns the frame length, 0 on failure.
size_t sc_envelope_to_frame(const scorbot_v1_Envelope *e, uint8_t *out, size_t cap);

#ifdef __cplusplus
}
#endif
