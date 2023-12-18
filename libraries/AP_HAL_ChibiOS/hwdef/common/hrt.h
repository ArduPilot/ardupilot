#pragma once


#ifdef __cplusplus
extern "C" {
#endif

void hrt_init(void);
uint64_t hrt_micros64(void);
uint64_t hrt_micros64I(void); // from locked context
uint64_t hrt_micros64_from_ISR(void); // from an ISR
uint32_t hrt_micros32(void);
uint32_t hrt_millis32(void);
uint32_t hrt_millis32I(void); // from locked context
uint32_t hrt_millis32_from_ISR(void); // from an ISR
uint32_t hrt_millis64(void);

/*
  thanks to:
  https://0x414b.com/2021/04/16/arm-division.html
 */
static inline uint64_t _hrt_umul64x64hi(uint32_t b, uint32_t a, uint32_t d, uint32_t c)
{
    __asm__ ("\n\
umull   r4, r5, %[b], %[d]   \n\
umull   %[d], r4, %[a], %[d]    \n\
adds    r5, %[d]         \n\
umull   %[d], %[a], %[a], %[c]  \n\
adcs    r4, %[d]        \n\
adc     %[a], #0        \n\
umull   %[c], %[b], %[b], %[c]  \n\
adds    r5, %[c]        \n\
adcs    %[b], r4        \n\
adc     %[a], #0        \n\
"   : [a] "+r" (a), [b] "+r" (b), [c] "+r" (c), [d] "+r" (d) : : "r4", "r5");
    return (uint64_t) a << 32 | b;
}

/*
  return x / 1000
  faster than the gcc implementation using _hrt_umul64x64hi() by about 3x
*/
static inline uint64_t _hrt_div1000(uint64_t x)
{
    x >>= 3U;
    return _hrt_umul64x64hi((uint32_t)x, x >> 32U, 0xe353f7cfU, 0x20c49ba5U) >> 4U;
}

#ifdef __cplusplus
}
#endif
