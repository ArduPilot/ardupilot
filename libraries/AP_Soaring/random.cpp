// Copyright (c) Microsoft Corporation. All rights reserved. 
// Licensed under the GPLv3 license

#include <stdint.h>
#include "random.h"
#include <cmath>


// xorshift128
//
// See "Xorshift RNGs" by George Marsaglia, The Florida State University
//
// Code based on snippet from Wikipedia
// https://en.wikipedia.org/wiki/Xorshift
//

// non zero seed
uint32_t s[] = { 12793,912503,84501,73115 };

uint32_t xorshift128(void)
{
    uint32_t t = s[0];
    t ^= t << 11;
    t ^= t >> 8;
    s[0] = s[1]; s[1] = s[2]; s[2] = s[3];
    s[3] ^= s[3] >> 19;
    s[3] ^= t;
    return s[3];
}

/* Polar form of Box-Muller transform
*  (run time increases if sample point lands outside 
*  unit cicle, but avoids using trig functions)
*/
void polar_box_muller(float* y1, float* y2)
{
    float x1, x2, w;
    uint32_t t;
    int i = 0;
    
    do
    {
        t = s[0];
        t ^= s[0] << 11;
        t ^= t >> 8;
        s[0] = s[1]; s[1] = s[2]; s[2] = s[3];
        s[3] ^= s[3] >> 19;
        s[3] ^= t;
        x1 = 2.0 * ((s[3] >> 8) * XSADD_FLOAT_MUL) - 1.0;
        t = s[0];
        t ^= s[0] << 11;
        t ^= t >> 8;
        s[0] = s[1]; s[1] = s[2]; s[2] = s[3];
        s[3] ^= s[3] >> 19;
        s[3] ^= t;
        x2 = 2.0 * ((s[3] >> 8) * XSADD_FLOAT_MUL) - 1.0;
        w = x1 * x1 + x2 * x2;
        i++;
    }
    while ( w >= 1.0 );
    
    w = sqrtf( (-2.0 * logf( w ) ) / w );
    *y1 = x1 * w;
    *y2 = x2 * w;
}

/* Standard form of Box-Muller transform
 * (constant run time)
 */
void trig_box_muller(float* y1, float* y2)
{
	float x1, x2;
	uint32_t t;

	t = s[0];
	t ^= s[0] << 11;
	t ^= t >> 8;
	s[0] = s[1]; s[1] = s[2]; s[2] = s[3];
	s[3] ^= s[3] >> 19;
	s[3] ^= t;
	x1 = ((s[3] >> 8) * XSADD_FLOAT_MUL);
	t = s[0];
	t ^= s[0] << 11;
	t ^= t >> 8;
	s[0] = s[1]; s[1] = s[2]; s[2] = s[3];
	s[3] ^= s[3] >> 19;
	s[3] ^= t;
	x2 = ((s[3] >> 8) * XSADD_FLOAT_MUL);
	float r = sqrtf(-2.0 * logf(x1));
	float theta = 2.0 * M_PI * x2;
	*y1 = r*sinf(theta);
	*y2 = r*cosf(theta);
}


// cholesky decomposition of 4x4 matix A into L
// A must be positive definite (not tested for)
// L will be set to the lower triangular cholesky decomposition of A
void cholesky44(float (&A)[4][4], float (&L)[4][4])
{
    L[0][0] =  sqrtf(A[0][0]);
    L[1][0] =  1.0 / L[0][0] * (A[1][0]);
    L[1][1] =  sqrtf(A[1][1] - L[1][0] * L[1][0]);
    L[2][0] =  1.0 / L[0][0] * (A[2][0] );
    L[2][1] =  1.0 / L[1][1] * (A[2][1] - L[2][0] * L[1][0]);
    L[2][2] =  sqrtf(A[2][2] - (L[2][0] * L[2][0] + L[2][1] * L[2][1]));
    L[3][0] =  1.0 / L[0][0] * (A[3][0]);
    L[3][1] =  1.0 / L[1][1] * (A[3][1] - L[3][0] * L[1][0]);
    L[3][2] =  1.0 / L[2][2] * (A[3][2] - (L[3][0] * L[2][0] + L[3][1] * L[2][1]));
    L[3][3] =  sqrtf(A[3][3] - (L[3][0] * L[3][0] + L[3][1] * L[3][1] + L[3][2] * L[3][2]));
}

void multivariate_normal_fill(float (&samp)[MAX_GAUSS_SAMPLES][4], float (&mean)[4], float (&cov)[4][4], const int size, const int offset) {

    float m[4];
    float L[4][4];
    cholesky44(cov, L);

    for (int j = offset; j < offset + size; j++)
    {
        gaussian(&m[0],&m[2]);
        gaussian(&m[1],&m[3]);
        samp[j][0] = mean[0] + L[0][0] * m[0];
        samp[j][1] = mean[1] + L[1][0] * m[0] + L[1][1] * m[1];
        samp[j][2] = mean[2] + L[2][0] * m[0] + L[2][1] * m[1] + L[2][2] * m[2];
        samp[j][3] = mean[3] + L[3][0] * m[0] + L[3][1] * m[1] + L[3][2] * m[2] + L[3][3] * m[3];
    }
}

void multivariate_normal(float (&samp)[4], float (&mean)[4], float (&L)[4][4])
{    
    float m[4];   
    gaussian(&m[0],&m[2]);
    gaussian(&m[1],&m[3]);
    samp[0] = mean[0] + L[0][0] * m[0];
    samp[1] = mean[1] + L[1][0] * m[0] + L[1][1] * m[1];
    samp[2] = mean[2] + L[2][0] * m[0] + L[2][1] * m[1] + L[2][2] * m[2];
    samp[3] = mean[3] + L[3][0] * m[0] + L[3][1] * m[1] + L[3][2] * m[2] + L[3][3] * m[3];
}
