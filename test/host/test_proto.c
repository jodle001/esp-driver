// Acceptance test for the C framing against the reference implementation's vectors:
// scorbot_protocol/test/test_cobs.cpp, test_framing.cpp and the worked examples in
// scorbot_protocol/docs/protocol.md section 2.
#include <stdlib.h>

#include "minitest.h"
#include "scorbot_proto/cobs.h"
#include "scorbot_proto/crc16.h"
#include "scorbot_proto/envelope.h"
#include "scorbot_proto/framing.h"

static void check_cobs(const uint8_t *in, size_t n, const uint8_t *expect, size_t en)
{
    uint8_t out[600];
    const size_t len = sc_cobs_encode(in, n, out, sizeof(out));
    MT_EQ(len, en);
    if (len == en) {
        MT_MEMEQ(out, expect, en);
    }
    uint8_t back[600];
    const size_t dn = sc_cobs_decode(out, len, back, sizeof(back));
    MT_EQ(dn, n);
    if (dn == n && n) {
        MT_MEMEQ(back, in, n);
    }
}

static void test_cobs_known_vectors(void)
{
    check_cobs((const uint8_t[]){0}, 0, (const uint8_t[]){0x01}, 1);
    check_cobs((const uint8_t[]){0x00}, 1, (const uint8_t[]){0x01, 0x01}, 2);
    check_cobs((const uint8_t[]){0x00, 0x00}, 2, (const uint8_t[]){0x01, 0x01, 0x01}, 3);
    check_cobs((const uint8_t[]){0x00, 0x11, 0x00}, 3, (const uint8_t[]){0x01, 0x02, 0x11, 0x01}, 4);
    check_cobs((const uint8_t[]){0x11, 0x22, 0x00, 0x33}, 4, (const uint8_t[]){0x03, 0x11, 0x22, 0x02, 0x33}, 5);
    check_cobs((const uint8_t[]){0x11, 0x22, 0x33, 0x44}, 4, (const uint8_t[]){0x05, 0x11, 0x22, 0x33, 0x44}, 5);
    check_cobs((const uint8_t[]){0x11, 0x00, 0x00, 0x00}, 4, (const uint8_t[]){0x02, 0x11, 0x01, 0x01, 0x01}, 5);

    /* 1..254: one full block, 0xFF then the bytes. */
    uint8_t in[255], expect[258];
    for (int i = 0; i < 254; ++i) {
        in[i] = (uint8_t)(i + 1);
    }
    expect[0] = 0xFF;
    memcpy(expect + 1, in, 254);
    check_cobs(in, 254, expect, 255);
    /* 0, 1..254: 0x01, then 0xFF + 254 bytes. */
    uint8_t in2[255];
    in2[0] = 0;
    memcpy(in2 + 1, in, 254);
    uint8_t expect2[257];
    expect2[0] = 0x01;
    expect2[1] = 0xFF;
    memcpy(expect2 + 2, in, 254);
    check_cobs(in2, 255, expect2, 256);
    /* 1..255: 0xFF + 254 bytes, then 0x02 0xFF. */
    uint8_t in3[255];
    for (int i = 0; i < 255; ++i) {
        in3[i] = (uint8_t)(i + 1);
    }
    uint8_t expect3[257];
    expect3[0] = 0xFF;
    memcpy(expect3 + 1, in3, 254);
    expect3[255] = 0x02;
    expect3[256] = 0xFF;
    check_cobs(in3, 255, expect3, 257);
}

static void test_cobs_random_round_trip(void)
{
    srand(12345);
    for (int trial = 0; trial < 200; ++trial) {
        const size_t len = (size_t)(rand() % 600);
        uint8_t in[600], enc[700], dec[700];
        for (size_t i = 0; i < len; ++i) {
            in[i] = (uint8_t)(rand() % 4 == 0 ? 0 : rand() & 0xFF);
        }
        const size_t e = sc_cobs_encode(in, len, enc, sizeof(enc));
        MT_TRUE(e > 0 && e <= sc_cobs_max_encoded_size(len));
        for (size_t i = 0; i < e; ++i) {
            MT_TRUE(enc[i] != 0);
        }
        const size_t d = sc_cobs_decode(enc, e, dec, sizeof(dec));
        MT_EQ(d, len);
        if (d == len && len) {
            MT_MEMEQ(dec, in, len);
        }
    }
}

static void test_cobs_rejects_malformed(void)
{
    uint8_t out[16];
    MT_EQ(sc_cobs_decode((const uint8_t[]){0x02, 0x00}, 2, out, sizeof(out)), SC_COBS_NPOS);
    MT_EQ(sc_cobs_decode((const uint8_t[]){0x05, 0x11}, 2, out, sizeof(out)), SC_COBS_NPOS);
    MT_EQ(sc_cobs_decode((const uint8_t[]){0x00}, 1, out, sizeof(out)), SC_COBS_NPOS);
    const uint8_t in[] = {0x11, 0x22, 0x00, 0x33};
    uint8_t small[4];
    MT_EQ(sc_cobs_encode(in, 4, small, sizeof(small)), 0u);
    uint8_t five[5];
    MT_EQ(sc_cobs_encode(in, 4, five, sizeof(five)), 5u);
    uint8_t tiny[3];
    MT_EQ(sc_cobs_decode(five, 5, tiny, sizeof(tiny)), SC_COBS_NPOS);
}

static void test_crc16_check_value(void)
{
    MT_EQ(sc_crc16((const uint8_t *)"123456789", 9), 0x29B1);
    MT_EQ(sc_crc16(NULL, 0), 0xFFFF);
}

typedef struct {
    uint8_t payloads[8][64];
    size_t lens[8];
    int count;
} collector_t;

static void collect(void *ctx, const uint8_t *p, size_t n)
{
    collector_t *c = (collector_t *)ctx;
    if (c->count < 8) {
        memcpy(c->payloads[c->count], p, n < 64 ? n : 64);
        c->lens[c->count] = n;
    }
    ++c->count;
}

static void test_frame_layout_and_split_feeding(void)
{
    const uint8_t p0[] = {1, 2, 3}, p1[] = {0, 0, 0, 0}, p3[] = {0xFF, 0x00, 0x7F};
    const uint8_t *payloads[4] = {p0, p1, NULL, p3};
    const size_t lens[4] = {3, 4, 0, 3};
    uint8_t stream[256];
    size_t total = 0;
    for (int i = 0; i < 4; ++i) {
        const size_t n = sc_encode_frame(payloads[i], lens[i], stream + total, sizeof(stream) - total);
        MT_TRUE(n > 0);
        MT_EQ(stream[total + n - 1], SC_DELIMITER);
        for (size_t k = 0; k + 1 < n; ++k) {
            MT_TRUE(stream[total + k] != 0);
        }
        total += n;
    }
    /* byte by byte */
    sc_stream_decoder_t d;
    sc_stream_decoder_init(&d);
    collector_t c = {0};
    for (size_t i = 0; i < total; ++i) {
        sc_stream_decoder_feed(&d, stream + i, 1, collect, &c);
    }
    MT_EQ(c.count, 4);
    MT_EQ(d.stats.frames, 4u);
    for (int i = 0; i < 4 && i < c.count; ++i) {
        MT_EQ(c.lens[i], lens[i]);
        if (lens[i]) {
            MT_MEMEQ(c.payloads[i], payloads[i], lens[i]);
        }
    }
    /* all at once */
    sc_stream_decoder_init(&d);
    memset(&c, 0, sizeof(c));
    sc_stream_decoder_feed(&d, stream, total, collect, &c);
    MT_EQ(c.count, 4);
}

static void test_resync_after_garbage(void)
{
    const uint8_t good[] = {0x08, 0x01, 0x10, 0x02};
    uint8_t f[32];
    const size_t fn = sc_encode_frame(good, sizeof(good), f, sizeof(f));
    uint8_t stream[128];
    size_t n = 0;
    const uint8_t garbage[] = {0x55, 0x66, 0x77};
    memcpy(stream + n, garbage, 3); n += 3;          /* garbage fused with the next frame: CRC fails */
    memcpy(stream + n, f, fn); n += fn;
    memcpy(stream + n, f, fn); stream[n + 1] ^= 0x40; n += fn; /* corrupted byte: CRC fails */
    memcpy(stream + n, f, fn / 2); n += fn / 2;       /* partial frame, then a delimiter */
    stream[n++] = 0x00;
    memcpy(stream + n, f, fn); n += fn;               /* the good one */
    sc_stream_decoder_t d;
    sc_stream_decoder_init(&d);
    collector_t c = {0};
    sc_stream_decoder_feed(&d, stream, n, collect, &c);
    MT_EQ(c.count, 1);
    MT_EQ(d.stats.frames, 1u);
    MT_TRUE(d.stats.crc_errors + d.stats.cobs_errors + d.stats.short_frames >= 3);
}

static void test_overflow_discarded_to_next_delimiter(void)
{
    sc_stream_decoder_t d;
    sc_stream_decoder_init(&d);
    collector_t c = {0};
    uint8_t junk[SC_MAX_FRAME + 100];
    memset(junk, 0x11, sizeof(junk));
    sc_stream_decoder_feed(&d, junk, sizeof(junk), collect, &c);
    const uint8_t delim = 0;
    sc_stream_decoder_feed(&d, &delim, 1, collect, &c);
    MT_EQ(c.count, 0);
    MT_EQ(d.stats.overflows, 1u);
    const uint8_t good[] = {0x08, 0x01};
    uint8_t f[16];
    const size_t fn = sc_encode_frame(good, 2, f, sizeof(f));
    sc_stream_decoder_feed(&d, f, fn, collect, &c);
    MT_EQ(c.count, 1);
}

/* protocol.md worked examples */
static void test_worked_examples(void)
{
    scorbot_v1_Envelope e = scorbot_v1_Envelope_init_zero;
    e.seq = 1;
    e.timestamp_us = 1000;
    e.which_payload = scorbot_v1_Envelope_request_tag;
    e.payload.request.request_id = 1;
    e.payload.request.which_body = scorbot_v1_Request_ping_tag;
    e.payload.request.body.ping.nonce = 0x42;
    uint8_t payload[64], frame[64];
    size_t n = sc_envelope_encode(&e, payload, sizeof(payload));
    const uint8_t ping_payload[] = {0x08, 0x01, 0x10, 0xe8, 0x07, 0x62, 0x07, 0x08, 0x01, 0x92, 0x01, 0x02, 0x08, 0x42};
    MT_EQ(n, sizeof(ping_payload));
    MT_MEMEQ(payload, ping_payload, sizeof(ping_payload));
    MT_EQ(sc_crc16(payload, n), 0x5de6);
    n = sc_envelope_to_frame(&e, frame, sizeof(frame));
    const uint8_t ping_frame[] = {0x11, 0x08, 0x01, 0x10, 0xe8, 0x07, 0x62, 0x07, 0x08, 0x01, 0x92, 0x01, 0x02, 0x08, 0x42, 0x5d, 0xe6, 0x00};
    MT_EQ(n, sizeof(ping_frame));
    MT_MEMEQ(frame, ping_frame, sizeof(ping_frame));

    /* Pong response, seq=1, request_id=1, result=OK, pong=0x42, timestamp 1500 */
    memset(&e, 0, sizeof(e));
    e.seq = 1;
    e.timestamp_us = 1500;
    e.which_payload = scorbot_v1_Envelope_response_tag;
    e.payload.response.request_id = 1;
    e.payload.response.result = scorbot_v1_Result_RESULT_OK;
    e.payload.response.which_body = scorbot_v1_Response_pong_tag;
    e.payload.response.body.pong = 0x42;
    n = sc_envelope_to_frame(&e, frame, sizeof(frame));
    const uint8_t pong_frame[] = {0x0f, 0x08, 0x01, 0x10, 0xdc, 0x0b, 0x6a, 0x05, 0x08, 0x01, 0x90, 0x01, 0x42, 0x57, 0xe6, 0x00};
    MT_EQ(n, sizeof(pong_frame));
    MT_MEMEQ(frame, pong_frame, sizeof(pong_frame));

    /* GetInfo request: 0b 08 03 10 a0 1f 62 04 08 02 52 03 71 eb 00 (seq 3, ts 4000, id 2) */
    memset(&e, 0, sizeof(e));
    e.seq = 3;
    e.timestamp_us = 4000;
    e.which_payload = scorbot_v1_Envelope_request_tag;
    e.payload.request.request_id = 2;
    e.payload.request.which_body = scorbot_v1_Request_get_info_tag;
    n = sc_envelope_to_frame(&e, frame, sizeof(frame));
    const uint8_t info_frame[] = {0x0b, 0x08, 0x03, 0x10, 0xa0, 0x1f, 0x62, 0x04, 0x08, 0x02, 0x52, 0x03, 0x71, 0xeb, 0x00};
    MT_EQ(n, sizeof(info_frame));
    MT_MEMEQ(frame, info_frame, sizeof(info_frame));

    /* Event fault raised = WATCHDOG: 0c 08 09 10 f0 2e 72 02 08 01 51 aa 00 (seq 9, ts 6000) */
    memset(&e, 0, sizeof(e));
    e.seq = 9;
    e.timestamp_us = 6000;
    e.which_payload = scorbot_v1_Envelope_event_tag;
    e.payload.event.which_body = scorbot_v1_Event_fault_raised_tag;
    e.payload.event.body.fault_raised = scorbot_v1_FaultCode_FAULT_WATCHDOG;
    n = sc_envelope_to_frame(&e, frame, sizeof(frame));
    const uint8_t ev_frame[] = {0x0c, 0x08, 0x09, 0x10, 0xf0, 0x2e, 0x72, 0x02, 0x08, 0x01, 0x51, 0xaa, 0x00};
    MT_EQ(n, sizeof(ev_frame));
    MT_MEMEQ(frame, ev_frame, sizeof(ev_frame));

    /* And back: decode the ping frame's payload. */
    scorbot_v1_Envelope back;
    MT_TRUE(sc_envelope_decode(ping_payload, sizeof(ping_payload), &back));
    MT_EQ(back.which_payload, scorbot_v1_Envelope_request_tag);
    MT_EQ(back.payload.request.body.ping.nonce, 0x42u);
    MT_FALSE(sc_envelope_decode((const uint8_t[]){0xFF, 0xFF, 0xFF}, 3, &back));
}

int main(void)
{
    MT_RUN(test_cobs_known_vectors);
    MT_RUN(test_cobs_random_round_trip);
    MT_RUN(test_cobs_rejects_malformed);
    MT_RUN(test_crc16_check_value);
    MT_RUN(test_frame_layout_and_split_feeding);
    MT_RUN(test_resync_after_garbage);
    MT_RUN(test_overflow_discarded_to_next_delimiter);
    MT_RUN(test_worked_examples);
    return MT_SUMMARY();
}
