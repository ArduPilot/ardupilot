/* Copyright (c) 2012 Josh Triplett <josh@joshtriplett.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

// Reference: https://linux.die.net/man/3/be16toh

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <byteswap.h>
#include <endian.h>
#include <stdint.h>

#ifdef __CHECKER__
#define __ap_bitwise __attribute__((bitwise))
#define __ap_force __attribute__((force))
#else
#define __ap_bitwise
#define __ap_force
#endif

typedef uint16_t __ap_bitwise le16_t;
typedef uint16_t __ap_bitwise be16_t;
typedef uint32_t __ap_bitwise le32_t;
typedef uint32_t __ap_bitwise be32_t;
typedef uint64_t __ap_bitwise le64_t;
typedef uint64_t __ap_bitwise be64_t;

#undef htobe16
#undef htole16
#undef be16toh
#undef le16toh
#undef htobe32
#undef htole32
#undef be32toh
#undef le32toh
#undef htobe64
#undef htole64
#undef be64toh
#undef le64toh

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define bswap_16_on_le(x) __bswap_16(x)
#define bswap_32_on_le(x) __bswap_32(x)
#define bswap_64_on_le(x) __bswap_64(x)
#define bswap_16_on_be(x) (x)
#define bswap_32_on_be(x) (x)
#define bswap_64_on_be(x) (x)
#elif __BYTE_ORDER == __BIG_ENDIAN
#define bswap_16_on_le(x) (x)
#define bswap_32_on_le(x) (x)
#define bswap_64_on_le(x) (x)
#define bswap_16_on_be(x) __bswap_16(x)
#define bswap_32_on_be(x) __bswap_32(x)
#define bswap_64_on_be(x) __bswap_64(x)
#endif

static inline le16_t htole16(uint16_t value) { return (le16_t __ap_force) bswap_16_on_be(value); }
static inline le32_t htole32(uint32_t value) { return (le32_t __ap_force) bswap_32_on_be(value); }
static inline le64_t htole64(uint64_t value) { return (le64_t __ap_force) bswap_64_on_be(value); }

static inline be16_t htobe16(uint16_t value) { return (be16_t __ap_force) bswap_16_on_le(value); }
static inline be32_t htobe32(uint32_t value) { return (be32_t __ap_force) bswap_32_on_le(value); }
static inline be64_t htobe64(uint64_t value) { return (be64_t __ap_force) bswap_64_on_le(value); }

static inline uint16_t le16toh(le16_t value) { return bswap_16_on_be((uint16_t __ap_force)value); }
static inline uint32_t le32toh(le32_t value) { return bswap_32_on_be((uint32_t __ap_force)value); }
static inline uint64_t le64toh(le64_t value) { return bswap_64_on_be((uint64_t __ap_force)value); }

static inline uint16_t be16toh(be16_t value) { return bswap_16_on_le((uint16_t __ap_force)value); }
static inline uint32_t be32toh(be32_t value) { return bswap_32_on_le((uint32_t __ap_force)value); }
static inline uint64_t be64toh(be64_t value) { return bswap_64_on_le((uint64_t __ap_force)value); }

// methods for misaligned buffers
static inline uint16_t le16toh_ptr(const uint8_t *p) { return p[0] | (p[1]<<8U); }
static inline uint32_t le24toh_ptr(const uint8_t *p) { return p[0] | (p[1]<<8U) | (p[2]<<16U); }
static inline uint32_t le32toh_ptr(const uint8_t *p) { return p[0] | (p[1]<<8U) | (p[2]<<16U) | (p[3]<<24U); }
static inline uint64_t le64toh_ptr(const uint8_t *p) { return (uint64_t) p[0] | ((uint64_t) p[1]<<8ULL) | ((uint64_t) p[2]<<16U) | ((uint64_t) p[3]<<24U) | ((uint64_t) p[4]<<32U) | ((uint64_t) p[5]<<40U) | ((uint64_t) p[6]<<48U) | ((uint64_t) p[7]<<56U); }
static inline uint16_t be16toh_ptr(const uint8_t *p) { return p[1] | (p[0]<<8U); }
static inline uint32_t be24toh_ptr(const uint8_t *p) { return p[2] | (p[1]<<8U) | (p[0]<<16U); }
static inline uint32_t be32toh_ptr(const uint8_t *p) { return p[3] | (p[2]<<8U) | (p[1]<<16U) | (p[0]<<24U); }
static inline uint64_t be64toh_ptr(const uint8_t *p) { return (uint64_t) p[7] | ((uint64_t) p[6]<<8ULL) | ((uint64_t) p[5]<<16U) | ((uint64_t) p[4]<<24U) | ((uint64_t) p[3]<<32U) | ((uint64_t) p[2]<<40U) | ((uint64_t) p[1]<<48U) | ((uint64_t) p[0]<<56U); }

static inline float be32tofloat_ptr(const uint8_t *p) { return int32_to_float_le(be32toh_ptr(p)); }
static inline float be32tofloat_ptr(const uint8_t *p, const uint8_t offset) { return be32tofloat_ptr(&p[offset]); }
#ifdef ALLOW_DOUBLE_MATH_FUNCTIONS
static inline float be64todouble_ptr(const uint8_t *p) { return uint64_to_double_le(be64toh_ptr(p)); }
static inline float be64todouble_ptr(const uint8_t *p, const uint8_t offset) { return be64todouble_ptr(&p[offset]); }
#endif

static inline void put_le16_ptr(uint8_t *p, uint16_t v) { p[0] = (v&0xFF); p[1] = (v>>8); }
static inline void put_le24_ptr(uint8_t *p, uint32_t v) { p[0] = (v&0xFF); p[1] = (v>>8)&0xFF; p[2] = (v>>16)&0xFF; }
static inline void put_le32_ptr(uint8_t *p, uint32_t v) { p[0] = (v&0xFF); p[1] = (v>>8)&0xFF; p[2] = (v>>16)&0xFF; p[3] = (v>>24)&0xFF; }
static inline void put_le64_ptr(uint8_t *p, uint64_t v) { p[0] = (v&0xFF); p[1] = (v>>8)&0xFF; p[2] = (v>>16)&0xFF; p[3] = (v>>24)&0xFF; p[4] = (v>>24)&0xFF;p[5] = (v>>24)&0xFF; p[6] = (v>>24)&0xFF;p[7] = (v>>24)&0xFF;}
static inline void put_be16_ptr(uint8_t *p, uint16_t v) { p[1] = (v&0xFF); p[0] = (v>>8); }
static inline void put_be24_ptr(uint8_t *p, uint32_t v) { p[2] = (v&0xFF); p[1] = (v>>8)&0xFF; p[0] = (v>>16)&0xFF; }
static inline void put_be32_ptr(uint8_t *p, uint32_t v) { p[3] = (v&0xFF); p[2] = (v>>8)&0xFF; p[1] = (v>>16)&0xFF; p[0] = (v>>24)&0xFF; }
static inline void put_be64_ptr(uint8_t *p, uint64_t v) { p[7] = (v&0xFF); p[6] = (v>>8)&0xFF; p[5] = (v>>16)&0xFF; p[4] = (v>>24)&0xFF; p[3] = (v>>32)&0xFF;p[2] = (v>>40)&0xFF; p[1] = (v>>48)&0xFF;p[0] = (v>>56)&0xFF;}

#undef __ap_bitwise
#undef __ap_force
