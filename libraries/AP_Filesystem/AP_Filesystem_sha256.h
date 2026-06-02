/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
  SHA-256 hash computation.  FIPS 180-4 compliant.
  Header-only implementation; include in exactly one translation unit.
*/
#pragma once

#include <stdint.h>
#include <string.h>

struct AP_SHA256_CTX {
    uint32_t state[8];
    uint64_t byte_count;
    uint8_t  buf[64];
    uint8_t  buf_len;
};

static inline uint32_t _sha256_rotr(uint32_t x, uint32_t n)
{
    return (x >> n) | (x << (32 - n));
}

static const uint32_t _sha256_k[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

static void _sha256_process_block(AP_SHA256_CTX *ctx)
{
    uint32_t w[64];
    for (uint8_t i = 0; i < 16; i++) {
        const uint8_t *p = &ctx->buf[i * 4];
        w[i] = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
               ((uint32_t)p[2] <<  8) |  (uint32_t)p[3];
    }
    for (uint8_t i = 16; i < 64; i++) {
        const uint32_t s0 = _sha256_rotr(w[i-15],  7) ^ _sha256_rotr(w[i-15], 18) ^ (w[i-15] >>  3);
        const uint32_t s1 = _sha256_rotr(w[i-2],  17) ^ _sha256_rotr(w[i-2],  19) ^ (w[i-2]  >> 10);
        w[i] = w[i-16] + s0 + w[i-7] + s1;
    }

    uint32_t a = ctx->state[0], b = ctx->state[1], c = ctx->state[2], d = ctx->state[3];
    uint32_t e = ctx->state[4], f = ctx->state[5], g = ctx->state[6], h = ctx->state[7];

    for (uint8_t i = 0; i < 64; i++) {
        const uint32_t S1  = _sha256_rotr(e, 6) ^ _sha256_rotr(e, 11) ^ _sha256_rotr(e, 25);
        const uint32_t ch  = (e & f) ^ (~e & g);
        const uint32_t t1  = h + S1 + ch + _sha256_k[i] + w[i];
        const uint32_t S0  = _sha256_rotr(a, 2) ^ _sha256_rotr(a, 13) ^ _sha256_rotr(a, 22);
        const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        const uint32_t t2  = S0 + maj;
        h = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }

    ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c; ctx->state[3] += d;
    ctx->state[4] += e; ctx->state[5] += f; ctx->state[6] += g; ctx->state[7] += h;
}

static void AP_SHA256_Init(AP_SHA256_CTX *ctx)
{
    ctx->state[0] = 0x6a09e667; ctx->state[1] = 0xbb67ae85;
    ctx->state[2] = 0x3c6ef372; ctx->state[3] = 0xa54ff53a;
    ctx->state[4] = 0x510e527f; ctx->state[5] = 0x9b05688c;
    ctx->state[6] = 0x1f83d9ab; ctx->state[7] = 0x5be0cd19;
    ctx->byte_count = 0;
    ctx->buf_len = 0;
}

static void AP_SHA256_Update(AP_SHA256_CTX *ctx, const uint8_t *data, uint32_t len)
{
    while (len > 0) {
        const uint8_t room = 64 - ctx->buf_len;
        const uint8_t take = (len < room) ? (uint8_t)len : room;
        memcpy(&ctx->buf[ctx->buf_len], data, take);
        ctx->buf_len += take;
        ctx->byte_count += take;
        data += take;
        len  -= take;
        if (ctx->buf_len == 64) {
            _sha256_process_block(ctx);
            ctx->buf_len = 0;
        }
    }
}

static void AP_SHA256_Final(AP_SHA256_CTX *ctx, uint8_t digest[32])
{
    ctx->buf[ctx->buf_len++] = 0x80;

    if (ctx->buf_len > 56) {
        memset(&ctx->buf[ctx->buf_len], 0, 64 - ctx->buf_len);
        _sha256_process_block(ctx);
        ctx->buf_len = 0;
    }
    memset(&ctx->buf[ctx->buf_len], 0, 56 - ctx->buf_len);

    const uint64_t bit_count = ctx->byte_count * 8;
    ctx->buf[56] = (uint8_t)(bit_count >> 56);
    ctx->buf[57] = (uint8_t)(bit_count >> 48);
    ctx->buf[58] = (uint8_t)(bit_count >> 40);
    ctx->buf[59] = (uint8_t)(bit_count >> 32);
    ctx->buf[60] = (uint8_t)(bit_count >> 24);
    ctx->buf[61] = (uint8_t)(bit_count >> 16);
    ctx->buf[62] = (uint8_t)(bit_count >>  8);
    ctx->buf[63] = (uint8_t)(bit_count      );
    _sha256_process_block(ctx);

    for (uint8_t i = 0; i < 8; i++) {
        digest[i*4  ] = (uint8_t)(ctx->state[i] >> 24);
        digest[i*4+1] = (uint8_t)(ctx->state[i] >> 16);
        digest[i*4+2] = (uint8_t)(ctx->state[i] >>  8);
        digest[i*4+3] = (uint8_t)(ctx->state[i]      );
    }
}
