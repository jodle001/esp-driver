#include "scorbot_core/link_mode.h"

#include <string.h>

void sc_link_sniffer_init(sc_link_sniffer_t *s) { memset(s, 0, sizeof(*s)); }

sc_link_mode_t sc_link_sniffer_feed(sc_link_sniffer_t *s, const uint8_t *data, size_t n, size_t *consumed)
{
    size_t k = 0;
    while (s->mode == SC_LINK_UNDECIDED && k < n) {
        const uint8_t c = data[k++];
        if (s->held_len < SC_LINK_SNIFF_MAX) {
            s->held[s->held_len++] = c;
        }
        if (c == 0x00) {
            s->mode = SC_LINK_PROTOBUF;
        } else if (c == '\r' || c == '~' || c == 0x03) {
            s->mode = SC_LINK_ACL; /* a line end, the manual-mode toggle or Ctrl-C: the operator */
        } else if (s->held_len == SC_LINK_SNIFF_MAX) {
            /* Nothing decisive in 64 bytes: text would have had a CR by now. */
            s->mode = SC_LINK_PROTOBUF;
        }
    }
    if (consumed) {
        *consumed = k;
    }
    return s->mode;
}

size_t sc_link_sniffer_take(sc_link_sniffer_t *s, uint8_t *out, size_t cap)
{
    const size_t n = s->held_len < cap ? s->held_len : cap;
    memcpy(out, s->held, n);
    s->held_len = 0;
    return n;
}
