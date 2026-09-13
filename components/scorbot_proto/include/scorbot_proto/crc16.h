// CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, no reflection, no final xor.
// Check value for the ASCII string "123456789" is 0x29B1. Bitwise on purpose (no
// table to keep in sync with the Pi side); a 100-byte frame at 100 Hz does not need one.
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline uint16_t sc_crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)((uint16_t)byte << 8);
    for (int bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

static inline uint16_t sc_crc16(const uint8_t *data, size_t n)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < n; ++i) {
        crc = sc_crc16_update(crc, data[i]);
    }
    return crc;
}

#ifdef __cplusplus
}
#endif
