/*
  return 64 bit x / 1000
  faster than the normal gcc implementation using by about 3x
  With thanks to https://0x414b.com/2021/04/16/arm-division.html
  and https://stackoverflow.com/questions/74765410/multiply-two-uint64-ts-and-store-result-to-uint64-t-doesnt-seem-to-work
*/
static inline uint64_t uint64_div1000(uint64_t x)
{
    x >>= 3U;
    uint64_t a_lo = (uint32_t)x;
    uint64_t a_hi = x >> 32;
    const uint64_t b_lo = 0xe353f7cfU;
    const uint64_t b_hi = 0x20c49ba5U;

    uint64_t a_x_b_hi = a_hi * b_hi;
    uint64_t a_x_b_mid = a_hi * b_lo;
    uint64_t b_x_a_mid = b_hi * a_lo;
    uint32_t a_x_b_lo = (a_lo * b_lo)>>32;

    // 64-bit product + two 32-bit values
    uint64_t middle = a_x_b_mid + a_x_b_lo + (uint32_t)b_x_a_mid;

    // 64-bit product + two 32-bit values
    uint64_t r = a_x_b_hi + (middle >> 32) + (b_x_a_mid >> 32);
    return r >> 4U;
}
