// Which protocol is on the link: the Eshed dialect (text, the operator) or the protobuf
// framing (the ROS 2 stack). Decided from the first bytes after boot: a 0x00 byte is a
// COBS frame delimiter and never occurs in ACL text; a CR ends an ACL line and never
// occurs... well, it can occur inside a COBS frame, so the rule is: the first of the two
// to arrive decides. Bytes seen before the decision are kept and handed to the winner.
// The baud rates differ (9600 against 921600), which this cannot see; the device glue
// starts at the ACL rate and switches on framing errors, see main/esp_driver.c.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { SC_LINK_UNDECIDED = 0, SC_LINK_ACL, SC_LINK_PROTOBUF } sc_link_mode_t;

#define SC_LINK_SNIFF_MAX 64

typedef struct {
    sc_link_mode_t mode;
    uint8_t held[SC_LINK_SNIFF_MAX];
    size_t held_len;
} sc_link_sniffer_t;

void sc_link_sniffer_init(sc_link_sniffer_t *s);
/// Feed bytes while undecided. Returns the mode once decided (the caller then takes
/// the held bytes with sc_link_sniffer_take and feeds them to that protocol).
sc_link_mode_t sc_link_sniffer_feed(sc_link_sniffer_t *s, const uint8_t *data, size_t n, size_t *consumed);
/// The bytes received before the decision, including the deciding one. Clears them.
size_t sc_link_sniffer_take(sc_link_sniffer_t *s, uint8_t *out, size_t cap);

#ifdef __cplusplus
}
#endif
