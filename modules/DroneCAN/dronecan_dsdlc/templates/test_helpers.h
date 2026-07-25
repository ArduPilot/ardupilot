#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

float random_float16_val();
float random_float_val();
uint32_t random_range_unsigned_val(uint32_t min, uint32_t max);
int64_t random_bitlen_signed_val(uint8_t bitlen);
uint64_t random_bitlen_unsigned_val(uint8_t bitlen);

#ifdef __cplusplus
}
#endif
