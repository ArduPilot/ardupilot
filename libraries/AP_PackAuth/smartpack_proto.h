// SmartPack I2C protocol - shared by the pack firmware (Arduino/MCU) and the
// ArduPilot battery backend. Keep this file byte-identical on both sides.
//
// Telemetry uses standard Smart Battery (SBS) registers, so ArduPilot's
// stock SMBus code can read it. Only the three registers below are custom.
//
//   0x70  NONCE     write  : [0x70, n0..n7]   FC sends an 8-byte random challenge
//   0x71  STATUS    read 1 : 0=idle 1=busy 2=ready
//   0x72  RESPONSE  read 8 : SipHash-2-4(key, nonce || serial_le16), little endian
//
// The pack must NOT stretch the clock while computing: it sets STATUS=busy
// on receiving the nonce and the FC polls until STATUS=ready.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define SMARTPACK_REG_NONCE      0x70
#define SMARTPACK_REG_STATUS     0x71
#define SMARTPACK_REG_RESPONSE   0x72

#define SMARTPACK_NONCE_LEN      8
#define SMARTPACK_RESPONSE_LEN   8

#define SMARTPACK_ST_IDLE        0
#define SMARTPACK_ST_BUSY        1
#define SMARTPACK_ST_READY       2

// ---- SipHash-2-4 (public domain algorithm, Aumasson & Bernstein) ----
// A keyed MAC designed for short messages: no library needed on either end.

#define SMARTPACK_ROTL64(x, b) ((uint64_t)(((x) << (b)) | ((x) >> (64 - (b)))))

static inline uint64_t smartpack_load64le(const uint8_t *p)
{
    uint64_t v = 0;
    for (uint8_t i = 0; i < 8; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}

#define SMARTPACK_SIPROUND                                         \
    do {                                                           \
        v0 += v1; v1 = SMARTPACK_ROTL64(v1, 13); v1 ^= v0;         \
        v0 = SMARTPACK_ROTL64(v0, 32);                             \
        v2 += v3; v3 = SMARTPACK_ROTL64(v3, 16); v3 ^= v2;         \
        v0 += v3; v3 = SMARTPACK_ROTL64(v3, 21); v3 ^= v0;         \
        v2 += v1; v1 = SMARTPACK_ROTL64(v1, 17); v1 ^= v2;         \
        v2 = SMARTPACK_ROTL64(v2, 32);                             \
    } while (0)

static inline uint64_t smartpack_siphash24(const uint8_t key[16], const uint8_t *in, size_t inlen)
{
    const uint64_t k0 = smartpack_load64le(key);
    const uint64_t k1 = smartpack_load64le(key + 8);
    uint64_t v0 = 0x736f6d6570736575ULL ^ k0;
    uint64_t v1 = 0x646f72616e646f6dULL ^ k1;
    uint64_t v2 = 0x6c7967656e657261ULL ^ k0;
    uint64_t v3 = 0x7465646279746573ULL ^ k1;

    const uint8_t *end = in + (inlen - (inlen % 8));
    for (; in != end; in += 8) {
        const uint64_t m = smartpack_load64le(in);
        v3 ^= m;
        SMARTPACK_SIPROUND;
        SMARTPACK_SIPROUND;
        v0 ^= m;
    }

    uint64_t b = ((uint64_t)inlen) << 56;
    for (uint8_t i = 0; i < (inlen & 7); i++) {
        b |= (uint64_t)in[i] << (8 * i);
    }
    v3 ^= b;
    SMARTPACK_SIPROUND;
    SMARTPACK_SIPROUND;
    v0 ^= b;

    v2 ^= 0xff;
    SMARTPACK_SIPROUND;
    SMARTPACK_SIPROUND;
    SMARTPACK_SIPROUND;
    SMARTPACK_SIPROUND;
    return v0 ^ v1 ^ v2 ^ v3;
}

// response = SipHash-2-4(key, nonce[8] || serial_le16), written little endian.
// Binding the pack serial into the MAC ties the reply to this specific pack.
static inline void smartpack_compute_response(const uint8_t key[16],
                                              const uint8_t nonce[SMARTPACK_NONCE_LEN],
                                              uint16_t serial,
                                              uint8_t out[SMARTPACK_RESPONSE_LEN])
{
    uint8_t msg[SMARTPACK_NONCE_LEN + 2];
    memcpy(msg, nonce, SMARTPACK_NONCE_LEN);
    msg[SMARTPACK_NONCE_LEN]     = (uint8_t)(serial & 0xFF);
    msg[SMARTPACK_NONCE_LEN + 1] = (uint8_t)(serial >> 8);
    const uint64_t t = smartpack_siphash24(key, msg, sizeof(msg));
    for (uint8_t i = 0; i < SMARTPACK_RESPONSE_LEN; i++) {
        out[i] = (uint8_t)(t >> (8 * i));
    }
}
