#pragma once

/*
  sha-256 implementation for MAVLink based on Heimdal sources, with
  modifications to suit mavlink headers and optimizations for small systems

  WARNING: this implementation cannot hash more than 512MB into the same
           context! this applies to the total of all update calls.
 */
/*
 * Copyright (c) 1995 - 2001 Kungliga Tekniska Högskolan
 * (Royal Institute of Technology, Stockholm, Sweden).
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
  allow implementation to provide their own sha256 with the same API
*/
#ifndef HAVE_MAVLINK_SHA256

#ifdef MAVLINK_USE_CXX_NAMESPACE
namespace mavlink {
#endif

#ifndef MAVLINK_HELPER
#define MAVLINK_HELPER
#endif

typedef struct {
    union {
        unsigned char save_bytes[64];
        uint32_t save_u32[16];
    } u;
    uint32_t counter[8];
    uint32_t sz; // accumulated size in bytes (not bits!)
} mavlink_sha256_ctx;

#define Ch(x,y,z) (((x) & (y)) ^ ((~(x)) & (z)))
#define Maj(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))

#define ROTR(x,n)   (((x)>>(n)) | ((x) << (32 - (n))))

#define Sigma0(x)       (ROTR(x,2)  ^ ROTR(x,13) ^ ROTR(x,22))
#define Sigma1(x)       (ROTR(x,6)  ^ ROTR(x,11) ^ ROTR(x,25))
#define sigma0(x)       (ROTR(x,7)  ^ ROTR(x,18) ^ ((x)>>3))
#define sigma1(x)       (ROTR(x,17) ^ ROTR(x,19) ^ ((x)>>10))

static const uint32_t mavlink_sha256_constant_256[64] = {
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

MAVLINK_HELPER void mavlink_sha256_init(mavlink_sha256_ctx *m)
{
    m->counter[0] = 0x6a09e667;
    m->counter[1] = 0xbb67ae85;
    m->counter[2] = 0x3c6ef372;
    m->counter[3] = 0xa54ff53a;
    m->counter[4] = 0x510e527f;
    m->counter[5] = 0x9b05688c;
    m->counter[6] = 0x1f83d9ab;
    m->counter[7] = 0x5be0cd19;
    m->sz = 0;
}

static void mavlink_sha256_calc(mavlink_sha256_ctx *m)
{
    uint32_t AA, BB, CC, DD, EE, FF, GG, HH;
    int i;

    // use the 16-word input as a ring buffer to compute the message schedule
    // alongside the compression function as we only need the last 16 words
    uint32_t *s = m->u.save_u32;
    // first we pack the 64 input bytes into 16 words over the top of the bytes;
    // these are the first 16 message schedule entries
    unsigned char *b = (unsigned char*)s; // cast s to not violate strict aliasing
    for (i = 0; i < 16; i++) {
        uint32_t w = 0;
        w |= (((uint32_t)*b++)<<24); // bytes are always packed big-endian
        w |= (((uint32_t)*b++)<<16);
        w |= (((uint32_t)*b++)<<8);
        w |= (((uint32_t)*b++)<<0);
        *s++ = w;
    }
    s = m->u.save_u32; // restart schedule at first word for compression loop

    AA = m->counter[0];
    BB = m->counter[1];
    CC = m->counter[2];
    DD = m->counter[3];
    EE = m->counter[4];
    FF = m->counter[5];
    GG = m->counter[6];
    HH = m->counter[7];

    for (i = 0; i < 64; i++) {
        uint32_t T1, T2, data;

        if (i < 16) {
            data = s[i];
        } else {
            data = sigma1(s[(i-2)&15]) + s[(i-7)&15] +
                   sigma0(s[(i-15)&15]) + s[(i-16)&15];
            s[i&15] = data; // note that (i-16)&15 is the same entry as i&15
        }

        T1 = HH + Sigma1(EE) + Ch(EE, FF, GG) + mavlink_sha256_constant_256[i] + data;
        T2 = Sigma0(AA) + Maj(AA,BB,CC);
                             
        HH = GG;
        GG = FF;
        FF = EE;
        EE = DD + T1;
        DD = CC;
        CC = BB;
        BB = AA;
        AA = T1 + T2;
    }

    m->counter[0] += AA;
    m->counter[1] += BB;
    m->counter[2] += CC;
    m->counter[3] += DD;
    m->counter[4] += EE;
    m->counter[5] += FF;
    m->counter[6] += GG;
    m->counter[7] += HH;
}

MAVLINK_HELPER void mavlink_sha256_update(mavlink_sha256_ctx *m, const void *v, uint32_t len)
{
    const unsigned char *p = (const unsigned char *)v;
    uint32_t offset;

    offset = m->sz % 64;
    m->sz += len;
    while(len > 0){
        uint32_t l = 64 - offset;
        if (len < l) {
            l = len;
        }
        memcpy(m->u.save_bytes + offset, p, l);
        offset += l;
        p += l;
        len -= l;
        if(offset == 64){
            mavlink_sha256_calc(m);
            offset = 0;
        }
    }
}

/*
  get first 48 bits of final sha256 hash
 */
MAVLINK_HELPER void mavlink_sha256_final_48(mavlink_sha256_ctx *m, uint8_t result[6])
{
    uint32_t c0, c1, bit_length;
    unsigned offset = m->sz % 64;
    bit_length = m->sz * 8;

    // to finalize the hash, we append to the current 64-byte block a 0x80,
    // enough zeros to align to the 56th byte of a block (possibly the next
    // one!) then the 8 byte bit length counter
    m->u.save_bytes[offset++] = 0x80;
    if (offset > 56) { // not enough space for length
        memset(&m->u.save_bytes[offset], 0, 64-offset); // zero remainder
        mavlink_sha256_calc(m); // process the block
        offset = 0; // start at the beginning of the next one
    }
    memset(&m->u.save_bytes[offset], 0, 60-offset); // zero up to length
    // unpack bit length into bytes
    m->u.save_bytes[60] = (bit_length >> 24) & 0xFF;
    m->u.save_bytes[61] = (bit_length >> 16) & 0xFF;
    m->u.save_bytes[62] = (bit_length >> 8) & 0xFF;
    m->u.save_bytes[63] = (bit_length >> 0) & 0xFF;
    mavlink_sha256_calc(m); // process last block

    // take first six bytes of hash result in big endian counter variables
    c0 = m->counter[0];
    c1 = m->counter[1];
    result[0] = (c0 >> 24) & 0xFF;
    result[1] = (c0 >> 16) & 0xFF;
    result[2] = (c0 >> 8) & 0xFF;
    result[3] = (c0 >> 0) & 0xFF;
    result[4] = (c1 >> 24) & 0xFF;
    result[5] = (c1 >> 16) & 0xFF;
}

// prevent conflicts with users of the header
#undef Ch
#undef ROTR
#undef Sigma0
#undef Sigma1
#undef sigma0
#undef sigma1

#ifdef MAVLINK_USE_CXX_NAMESPACE
} // namespace mavlink
#endif

#endif // HAVE_MAVLINK_SHA256
