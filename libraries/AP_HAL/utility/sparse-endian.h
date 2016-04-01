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
#ifndef SPARSE_ENDIAN_H
#define SPARSE_ENDIAN_H

#if defined __unix__

#if !defined(HAVE_BYTESWAP_H) || HAVE_BYTESWAP_H
#include <byteswap.h>
#endif
#include <endian.h>
#include <stdint.h>

#ifdef __CHECKER__
#define __bitwise __attribute__((bitwise))
#define __force __attribute__((force))
#else
#define __bitwise
#define __force
#endif

typedef uint16_t __bitwise le16_t;
typedef uint16_t __bitwise be16_t;
typedef uint32_t __bitwise le32_t;
typedef uint32_t __bitwise be32_t;
typedef uint64_t __bitwise le64_t;
typedef uint64_t __bitwise be64_t;

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

static inline le16_t htole16(uint16_t value) { return (le16_t __force) bswap_16_on_be(value); }
static inline le32_t htole32(uint32_t value) { return (le32_t __force) bswap_32_on_be(value); }
static inline le64_t htole64(uint64_t value) { return (le64_t __force) bswap_64_on_be(value); }

static inline be16_t htobe16(uint16_t value) { return (be16_t __force) bswap_16_on_le(value); }
static inline be32_t htobe32(uint32_t value) { return (be32_t __force) bswap_32_on_le(value); }
static inline be64_t htobe64(uint64_t value) { return (be64_t __force) bswap_64_on_le(value); }

static inline uint16_t le16toh(le16_t value) { return bswap_16_on_be((uint16_t __force)value); }
static inline uint32_t le32toh(le32_t value) { return bswap_32_on_be((uint32_t __force)value); }
static inline uint64_t le64toh(le64_t value) { return bswap_64_on_be((uint64_t __force)value); }

static inline uint16_t be16toh(be16_t value) { return bswap_16_on_le((uint16_t __force)value); }
static inline uint32_t be32toh(be32_t value) { return bswap_32_on_le((uint32_t __force)value); }
static inline uint64_t be64toh(be64_t value) { return bswap_64_on_le((uint64_t __force)value); }

#else // not __unix__
   #pragma message "Assuming byteswap.h endian.h not defined for this platform"
   #if ! (defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && defined(__ORDER_BIG_ENDIAN__))
      #error  byte order macros are not defined for this compiler.
   #endif
   // some sort of type safe structure
   struct be16_t{
      uint8_t ar[2];
   };

   inline int16_t be16toh(be16_t in)
   {
   #if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
      #pragma message "BEWARE! big endian be16toh function has not been tested."
      return static_cast<int16_t>(static_cast<uint16_t>(in.ar[0]) * 256  +  static_cast<uint16_t>(in.ar[1])) ;
   #else
      #if  (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
         #pragma message "little endian platform."
         return static_cast<int16_t>(static_cast<uint16_t>(in.ar[1]) | (static_cast<uint16_t>(in.ar[0]) << 8U) );
      #else  
         #error BYTE_ORDER macros dont work as expected.
      #endif
   #endif // __BYTE_ORDER__
   }
#endif // not __unix__

#endif /* SPARSE_ENDIAN_H */
