#include "scorbot_proto/envelope.h"

#include <string.h>

#include "pb_decode.h"
#include "pb_encode.h"
#include "scorbot_proto/framing.h"

#if scorbot_v1_Envelope_size > SC_MAX_PAYLOAD
#error "SC_MAX_PAYLOAD must hold the largest possible Envelope"
#endif

size_t sc_envelope_encode(const scorbot_v1_Envelope *e, uint8_t *out, size_t cap)
{
    pb_ostream_t stream = pb_ostream_from_buffer(out, cap);
    if (!pb_encode(&stream, scorbot_v1_Envelope_fields, e)) {
        return 0;
    }
    return stream.bytes_written;
}

bool sc_envelope_decode(const uint8_t *data, size_t len, scorbot_v1_Envelope *e)
{
    memset(e, 0, sizeof(*e));
    pb_istream_t stream = pb_istream_from_buffer(data, len);
    return pb_decode(&stream, scorbot_v1_Envelope_fields, e);
}

size_t sc_envelope_to_frame(const scorbot_v1_Envelope *e, uint8_t *out, size_t cap)
{
    uint8_t payload[scorbot_v1_Envelope_size];
    const size_t n = sc_envelope_encode(e, payload, sizeof(payload));
    if (n == 0 && e->which_payload != 0) {
        return 0;
    }
    return sc_encode_frame(payload, n, out, cap);
}
