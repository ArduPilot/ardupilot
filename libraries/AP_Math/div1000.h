#pragma once
#include <stdint.h>

static inline uint64_t uint64_mulhi_32b(uint64_t x, uint64_t y)
{
    uint64_t a_lo = (uint32_t)x;
    uint64_t a_hi = x >> 32;
    uint64_t b_lo = (uint32_t)y;
    uint64_t b_hi = y >> 32;

    uint64_t a_x_b_hi = a_hi * b_hi;
    uint64_t a_x_b_mid = a_hi * b_lo;
    uint64_t b_x_a_mid = b_hi * a_lo;
    uint32_t a_x_b_lo = (a_lo * b_lo)>>32;

    // 64-bit product + two 32-bit values
    uint64_t middle = a_x_b_mid + a_x_b_lo + (uint32_t)b_x_a_mid;

    // 64-bit product + two 32-bit values
    return a_x_b_hi + (middle >> 32) + (b_x_a_mid >> 32);
}

/*
  return 64 bit x / 1000
  faster than the normal gcc implementation using by about 3x
  With thanks to https://0x414b.com/2021/04/16/arm-division.html
  and https://stackoverflow.com/questions/74765410/multiply-two-uint64-ts-and-store-result-to-uint64-t-doesnt-seem-to-work
*/
static inline uint64_t uint64_div1000_32b(uint64_t x)
{
    return uint64_mulhi_32b(x >> 3U, 0x20c49ba5e353f7cfULL) >> 4U;
}

/*
  For testability purposes both implementations are built on 64-bit targets.
  Most callers should use uint64_div1000().
*/

#if defined(__SIZEOF_INT128__)
static inline uint64_t uint64_mulhi_64b(uint64_t x, uint64_t y)
{
    return ((unsigned __int128)x * y) >> 64;
}

static inline uint64_t uint64_div1000_64b(uint64_t x)
{
    return uint64_mulhi_64b(x >> 3U, 0x20c49ba5e353f7cfULL) >> 4U;
}
#define uint64_mulhi(x,y) uint64_mulhi_64b((x), (y))
#define uint64_div1000(x) uint64_div1000_64b((x))
#else
#define uint64_mulhi(x,y) uint64_mulhi_32b((x), (y))
#define uint64_div1000(x) uint64_div1000_32b((x))
#endif
