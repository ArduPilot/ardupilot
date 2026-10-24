// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#define HUGE_VAL (__builtin_huge_val())
#define HUGE_VALF (__builtin_huge_valf())
#define fabsf(x) __builtin_fabsf(x)
#ifdef __cplusplus
extern "C" {
#endif
float acosf(float);
float asinf(float);
float atan2f(float, float);
float ceilf(float);
float cosf(float);
float expf(float);
float floorf(float);
float fmodf(float, float);
float logf(float);
float log2f(float);
float log10f(float);
float powf(float, float);
float sinf(float);
float sqrtf(float);
float tanf(float);
float frexpf(float, int *);
float ldexpf(float, int);
#ifdef __cplusplus
}
#endif
