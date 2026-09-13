#include "scorbot_proto/cobs.h"

size_t sc_cobs_encode(const uint8_t *in, size_t n, uint8_t *out, size_t cap)
{
    if (cap == 0) {
        return 0;
    }
    size_t code_idx = 0; /* where the current block's length byte goes */
    size_t o = 1;        /* next output position */
    uint8_t code = 1;    /* current block length (data bytes + 1) */
    int open = 1;        /* a code slot is reserved at code_idx */

    for (size_t i = 0; i < n; ++i) {
        if (!open) {
            /* Previous block was a full 254 bytes and more input follows: new block. */
            code_idx = o++;
            if (o > cap) {
                return 0;
            }
            code = 1;
            open = 1;
        }
        const uint8_t b = in[i];
        if (b == 0) {
            out[code_idx] = code;
            code_idx = o++;
            if (o > cap) {
                return 0;
            }
            code = 1;
        } else {
            if (o >= cap) {
                return 0;
            }
            out[o++] = b;
            if (++code == 0xFF) {
                out[code_idx] = code;
                open = 0;
            }
        }
    }
    if (open) {
        out[code_idx] = code;
    }
    return o;
}

size_t sc_cobs_decode(const uint8_t *in, size_t n, uint8_t *out, size_t cap)
{
    size_t i = 0, o = 0;
    int first = 1; /* no implied zero before the first block or after a full block */
    while (i < n) {
        const uint8_t code = in[i++];
        if (code == 0) {
            return SC_COBS_NPOS;
        }
        if (!first) {
            if (o >= cap) {
                return SC_COBS_NPOS;
            }
            out[o++] = 0;
        }
        const size_t len = (size_t)code - 1;
        if (i + len > n || o + len > cap) {
            return SC_COBS_NPOS;
        }
        for (size_t k = 0; k < len; ++k) {
            const uint8_t b = in[i++];
            if (b == 0) {
                return SC_COBS_NPOS;
            }
            out[o++] = b;
        }
        first = (code == 0xFF);
    }
    return o;
}
