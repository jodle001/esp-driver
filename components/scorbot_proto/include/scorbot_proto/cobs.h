// Consistent Overhead Byte Stuffing (COBS), C port of scorbot_protocol/cobs.hpp.
//
// Encodes a byte string so that it contains no 0x00 bytes, at a cost of one extra byte
// per 254 input bytes; a 0x00 is then the frame delimiter on the serial line. No
// allocation, no state. The test vectors in scorbot_protocol/test/test_cobs.cpp are
// the acceptance test (test/host/test_proto.c).
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SC_COBS_NPOS ((size_t)-1)

/// Upper bound on the encoded size of n input bytes (without a delimiter).
static inline size_t sc_cobs_max_encoded_size(size_t n) { return n + n / 254 + 1; }

/// Encode n bytes from in into out (capacity cap). No delimiter is appended.
/// Returns the encoded length, or 0 if out is too small.
size_t sc_cobs_encode(const uint8_t *in, size_t n, uint8_t *out, size_t cap);

/// Decode n COBS bytes (no delimiter inside) from in into out (capacity cap).
/// Returns the decoded length, or SC_COBS_NPOS if the input is malformed or out too small.
size_t sc_cobs_decode(const uint8_t *in, size_t n, uint8_t *out, size_t cap);

#ifdef __cplusplus
}
#endif
