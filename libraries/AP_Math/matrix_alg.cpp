/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * matrix3.cpp
 * Copyright (C) Siddharth Bharat Purohit, 3DRobotics Inc. 2015
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma GCC optimize("O3")

#include <AP_HAL/AP_HAL.h>

#include <stdio.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <fenv.h>
#endif

#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

//TODO: use higher precision datatypes to achieve more accuracy for matrix algebra operations

/*
 *    Does matrix multiplication of two regular/square matrices
 *
 *    @param     A,           Matrix A
 *    @param     B,           Matrix B
 *    @param     n,           dimemsion of square matrices
 *    @returns                multiplied matrix i.e. A*B
 */

float* mat_mul(float *A, float *B, uint8_t n)
{
    float* ret = new float[n*n];
    memset(ret,0.0f,n*n*sizeof(float));

    for(uint8_t i = 0; i < n; i++) {
        for(uint8_t j = 0; j < n; j++) {
            for(uint8_t k = 0;k < n; k++) {
                ret[i*n + j] += A[i*n + k] * B[k*n + j];
            }
        }
    }
    return ret;
}

static inline void swap(float &a, float &b)
{
    float c;
    c = a;
    a = b;
    b = c;
}

/*
 *    calculates pivot matrix such that all the larger elements in the row are on diagonal
 *
 *    @param     A,           input matrix matrix
 *    @param     pivot
 *    @param     n,           dimenstion of square matrix
 *    @returns                false = matrix is Singular or non positive definite, true = matrix inversion successful
 */

void mat_pivot(float* A, float* pivot, uint8_t n)
{
    for(uint8_t i = 0;i<n;i++){
        for(uint8_t j=0;j<n;j++) {
            pivot[i*n+j] = (i==j);
        }
    }

    for(uint8_t i = 0;i < n; i++) {
        uint8_t max_j = i;
        for(uint8_t j=i;j<n;j++){
            if(fabsf(A[j*n + i]) > fabsf(A[max_j*n + i])) {
                max_j = j;
            }
        }

        if(max_j != i) {
            for(uint8_t k = 0; k < n; k++) {
                swap(pivot[i*n + k], pivot[max_j*n + k]);
            }
        }
    }
}

/*
 *    calculates matrix inverse of Lower trangular matrix using forward substitution
 *
 *    @param     L,           lower triangular matrix
 *    @param     out,         Output inverted lower triangular matrix
 *    @param     n,           dimension of matrix
 */

void mat_forward_sub(float *L, float *out, uint8_t n)
{
    // Forward substitution solve LY = I
    for(int i = 0; i < n; i++) {
        out[i*n + i] = 1/L[i*n + i];
        for (int j = i+1; j < n; j++) {
            for (int k = i; k < j; k++) {
                out[j*n + i] -= L[j*n + k] * out[k*n + i];
            }
            out[j*n + i] /= L[j*n + j];
        }
    }
}

/*
 *    calculates matrix inverse of Upper trangular matrix using backward substitution
 *
 *    @param     U,           upper triangular matrix
 *    @param     out,         Output inverted upper triangular matrix
 *    @param     n,           dimension of matrix
 */

void mat_back_sub(float *U, float *out, uint8_t n)
{
    // Backward Substitution solve UY = I
    for(int i = n-1; i >= 0; i--) {
        out[i*n + i] = 1/U[i*n + i];
        for (int j = i - 1; j >= 0; j--) {
            for (int k = i; k > j; k--) {
                out[j*n + i] -= U[j*n + k] * out[k*n + i];
            }
            out[j*n + i] /= U[j*n + j];
        }
    }
}

/*
 *    Decomposes square matrix into Lower and Upper triangular matrices such that
 *    A*P = L*U, where P is the pivot matrix
 *    ref: http://rosettacode.org/wiki/LU_decomposition
 *    @param     U,           upper triangular matrix
 *    @param     out,         Output inverted upper triangular matrix
 *    @param     n,           dimension of matrix
 */

void mat_LU_decompose(float* A, float* L, float* U, float *P, uint8_t n)
{
    memset(L,0,n*n*sizeof(float));
    memset(U,0,n*n*sizeof(float));
    memset(P,0,n*n*sizeof(float));
    mat_pivot(A,P,n);

    float *APrime = mat_mul(P,A,n);
    for(uint8_t i = 0; i < n; i++) {
        L[i*n + i] = 1;
    }
    for(uint8_t i = 0; i < n; i++) {
        for(uint8_t j = 0; j < n; j++) {
            if(j <= i) {    
                U[j*n + i] = APrime[j*n + i];
                for(uint8_t k = 0; k < j; k++) {
                    U[j*n + i] -= L[j*n + k] * U[k*n + i]; 
                }
            }
            if(j >= i) {
                L[j*n + i] = APrime[j*n + i];
                for(uint8_t k = 0; k < i; k++) {
                    L[j*n + i] -= L[j*n + k] * U[k*n + i]; 
                }
                L[j*n + i] /= U[i*n + i];
            }
        }
    }
    free(APrime);
}

/*
 *    matrix inverse code for any square matrix using LU decomposition
 *    inv = inv(U)*inv(L)*P, where L and U are triagular matrices and P the pivot matrix
 *    ref: http://www.cl.cam.ac.uk/teaching/1314/NumMethods/supporting/mcmaster-kiruba-ludecomp.pdf
 *    @param     m,           input 4x4 matrix
 *    @param     inv,      Output inverted 4x4 matrix
 *    @param     n,           dimension of square matrix
 *    @returns                false = matrix is Singular, true = matrix inversion successful
 */
bool mat_inverse(float* A, float* inv, uint8_t n)
{
    float *L, *U, *P;
    bool ret = true;
    L = new float[n*n];
    U = new float[n*n];
    P = new float[n*n];
    mat_LU_decompose(A,L,U,P,n);

    float *L_inv = new float[n*n];
    float *U_inv = new float[n*n];

    memset(L_inv,0,n*n*sizeof(float));
    mat_forward_sub(L,L_inv,n);

    memset(U_inv,0,n*n*sizeof(float));
    mat_back_sub(U,U_inv,n);

    // decomposed matrices no longer required
    free(L);
    free(U);

    float *inv_unpivoted = mat_mul(U_inv,L_inv,n);
    float *inv_pivoted = mat_mul(inv_unpivoted, P, n);

    //check sanity of results
    for(uint8_t i = 0; i < n; i++) {
        for(uint8_t j = 0; j < n; j++) {
            if(isnan(inv_pivoted[i*n+j]) || isinf(inv_pivoted[i*n+j])){
                ret = false;
            }
        }
    }
    memcpy(inv,inv_pivoted,n*n*sizeof(float));

    //free memory
    free(inv_pivoted);
    free(inv_unpivoted);
    free(P);
    free(U_inv);
    free(L_inv);
    return ret;
}

/*
 *    fast matrix inverse code only for 3x3 square matrix
 *
 *    @param     m,           input 4x4 matrix
 *    @param     invOut,      Output inverted 4x4 matrix
 *    @returns                false = matrix is Singular, true = matrix inversion successful
 */

bool inverse3x3(float m[], float invOut[])
{
    float inv[9];
    // computes the inverse of a matrix m
    float  det = m[0] * (m[4] * m[8] - m[7] * m[5]) -
    m[1] * (m[3] * m[8] - m[5] * m[6]) +
    m[2] * (m[3] * m[7] - m[4] * m[6]);
    if (is_zero(det) || isinf(det)) {
        return false;
    }

    float invdet = 1 / det;

    inv[0] = (m[4] * m[8] - m[7] * m[5]) * invdet;
    inv[1] = (m[2] * m[7] - m[1] * m[8]) * invdet;
    inv[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
    inv[3] = (m[5] * m[6] - m[3] * m[8]) * invdet;
    inv[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
    inv[5] = (m[3] * m[2] - m[0] * m[5]) * invdet;
    inv[6] = (m[3] * m[7] - m[6] * m[4]) * invdet;
    inv[7] = (m[6] * m[1] - m[0] * m[7]) * invdet;
    inv[8] = (m[0] * m[4] - m[3] * m[1]) * invdet;

    for(uint8_t i = 0; i < 9; i++){
        invOut[i] = inv[i];
    }

    return true;
}

/*
 *    fast matrix inverse code only for 4x4 square matrix copied from
 *    gluInvertMatrix implementation in opengl for 4x4 matrices.
 *
 *    @param     m,           input 4x4 matrix
 *    @param     invOut,      Output inverted 4x4 matrix
 *    @returns                false = matrix is Singular, true = matrix inversion successful
 */

bool inverse4x4(float m[],float invOut[])
{
    float inv[16], det;
    uint8_t i;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    int old = fedisableexcept(FE_OVERFLOW);
    if (old < 0) {
        hal.console->printf("inverse4x4(): warning: error on disabling FE_OVERFLOW floating point exception\n");
    }
#endif

    inv[0] = m[5]  * m[10] * m[15] -
    m[5]  * m[11] * m[14] -
    m[9]  * m[6]  * m[15] +
    m[9]  * m[7]  * m[14] +
    m[13] * m[6]  * m[11] -
    m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
    m[4]  * m[11] * m[14] +
    m[8]  * m[6]  * m[15] -
    m[8]  * m[7]  * m[14] -
    m[12] * m[6]  * m[11] +
    m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
    m[4]  * m[11] * m[13] -
    m[8]  * m[5] * m[15] +
    m[8]  * m[7] * m[13] +
    m[12] * m[5] * m[11] -
    m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
    m[4]  * m[10] * m[13] +
    m[8]  * m[5] * m[14] -
    m[8]  * m[6] * m[13] -
    m[12] * m[5] * m[10] +
    m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
    m[1]  * m[11] * m[14] +
    m[9]  * m[2] * m[15] -
    m[9]  * m[3] * m[14] -
    m[13] * m[2] * m[11] +
    m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
    m[0]  * m[11] * m[14] -
    m[8]  * m[2] * m[15] +
    m[8]  * m[3] * m[14] +
    m[12] * m[2] * m[11] -
    m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
    m[0]  * m[11] * m[13] +
    m[8]  * m[1] * m[15] -
    m[8]  * m[3] * m[13] -
    m[12] * m[1] * m[11] +
    m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
    m[0]  * m[10] * m[13] -
    m[8]  * m[1] * m[14] +
    m[8]  * m[2] * m[13] +
    m[12] * m[1] * m[10] -
    m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
    m[1]  * m[7] * m[14] -
    m[5]  * m[2] * m[15] +
    m[5]  * m[3] * m[14] +
    m[13] * m[2] * m[7] -
    m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
    m[0]  * m[7] * m[14] +
    m[4]  * m[2] * m[15] -
    m[4]  * m[3] * m[14] -
    m[12] * m[2] * m[7] +
    m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
    m[0]  * m[7] * m[13] -
    m[4]  * m[1] * m[15] +
    m[4]  * m[3] * m[13] +
    m[12] * m[1] * m[7] -
    m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
    m[0]  * m[6] * m[13] +
    m[4]  * m[1] * m[14] -
    m[4]  * m[2] * m[13] -
    m[12] * m[1] * m[6] +
    m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
    m[1] * m[7] * m[10] +
    m[5] * m[2] * m[11] -
    m[5] * m[3] * m[10] -
    m[9] * m[2] * m[7] +
    m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
    m[0] * m[7] * m[10] -
    m[4] * m[2] * m[11] +
    m[4] * m[3] * m[10] +
    m[8] * m[2] * m[7] -
    m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
    m[0] * m[7] * m[9] +
    m[4] * m[1] * m[11] -
    m[4] * m[3] * m[9] -
    m[8] * m[1] * m[7] +
    m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
    m[0] * m[6] * m[9] -
    m[4] * m[1] * m[10] +
    m[4] * m[2] * m[9] +
    m[8] * m[1] * m[6] -
    m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (old >= 0 && feenableexcept(old) < 0) {
        hal.console->printf("inverse4x4(): warning: error on restoring floating exception mask\n");
    }
#endif

    if (is_zero(det) || isinf(det)){
        return false;
    }

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;
    return true;
}

/*
 *    generic matrix inverse code
 *
 *    @param     x,     input nxn matrix
 *    @param     y,     Output inverted nxn matrix
 *    @param     n,     dimension of square matrix
 *    @returns          false = matrix is Singular, true = matrix inversion successful
 */
bool inverse(float x[], float y[], uint16_t dim)
{
    switch(dim){
        case 3: return inverse3x3(x,y);
        case 4: return inverse4x4(x,y);
        default: return mat_inverse(x,y,dim);
    }
}
