// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#pragma once
#include <stdint.h>

#define XSADD_FLOAT_MUL (1.0f / 16777216.0f)
#define MAX_GAUSS_SAMPLES 1000
#ifdef BOX_MULLER
#define gaussian(y1,y2) trig_box_muller(y1,y2)
#else
#define gaussian(y1,y2) polar_box_muller(y1,y2)
#endif
void polar_box_muller(float* y1, float* y2);
void trig_box_muller(float* y1, float* y2);
uint32_t xorshift128(void);
void cholesky44(float (&A)[4][4], float (&L)[4][4]);
void multivariate_normal_fill(float (&samp)[MAX_GAUSS_SAMPLES][4], float (&mean)[4], float (&cov)[4][4], const int size, const int offset = 0);
void multivariate_normal(float (&samp)[4], float (&mean)[4], float (&cov)[4][4]);

extern uint32_t s0[];
