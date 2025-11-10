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

#include <AP_HAL/AP_HAL.h>
#include "AP_Math.h"

#include <stdio.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <fenv.h>
#endif

/*
 *    Does matrix multiplication of two regular/square matrices
 *
 *    @param     A,           Matrix A
 *    @param     B,           Matrix B
 *    @param     n,           dimemsion of square matrices
 *    @returns                multiplied matrix i.e. A*B
 */
template<typename T>
static T* matrix_multiply(const T *A, const T *B, uint16_t n)
{
    T* ret = NEW_NOTHROW T[n*n];
    memset(ret,0.0f,n*n*sizeof(T));

    for(uint16_t i = 0; i < n; i++) {
        for(uint16_t j = 0; j < n; j++) {
            for(uint16_t k = 0;k < n; k++) {
                ret[i*n + j] += A[i*n + k] * B[k*n + j];
            }
        }
    }
    return ret;
}

template<typename T>
static inline void swap(T &a, T &b)
{
    T c;
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
template<typename T>
static void mat_pivot(const T* A, T* pivot, uint16_t n)
{
    for(uint16_t i = 0;i<n;i++){
        for(uint16_t j=0;j<n;j++) {
            pivot[i*n+j] = static_cast<T>(i==j);
        }
    }

    for(uint16_t i = 0;i < n; i++) {
        uint16_t max_j = i;
        for(uint16_t j=i;j<n;j++){
            if(fabsF(A[j*n + i]) > fabsF(A[max_j*n + i])) {
                max_j = j;
            }
        }

        if(max_j != i) {
            for(uint16_t k = 0; k < n; k++) {
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
template<typename T>
static void mat_forward_sub(const T *L, T *out, uint16_t n)
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
template<typename T>
static void mat_back_sub(const T *U, T *out, uint16_t n)
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
template<typename T>
static void mat_LU_decompose(const T* A, T* L, T* U, T *P, uint16_t n)
{
    memset(L,0,n*n*sizeof(T));
    memset(U,0,n*n*sizeof(T));
    memset(P,0,n*n*sizeof(T));
    mat_pivot(A,P,n);

    T *APrime = matrix_multiply(P,A,n);
    for(uint16_t i = 0; i < n; i++) {
        L[i*n + i] = 1;
    }
    for(uint16_t i = 0; i < n; i++) {
        for(uint16_t j = 0; j < n; j++) {
            if(j <= i) {    
                U[j*n + i] = APrime[j*n + i];
                for(uint16_t k = 0; k < j; k++) {
                    U[j*n + i] -= L[j*n + k] * U[k*n + i]; 
                }
            }
            if(j >= i) {
                L[j*n + i] = APrime[j*n + i];
                for(uint16_t k = 0; k < i; k++) {
                    L[j*n + i] -= L[j*n + k] * U[k*n + i]; 
                }
                L[j*n + i] /= U[i*n + i];
            }
        }
    }
    delete[] APrime;
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
template<typename T>
static bool mat_inverseN(const T* A, T* inv, uint16_t n)
{
    T *L, *U, *P;
    bool ret = true;
    L = NEW_NOTHROW T[n*n];
    U = NEW_NOTHROW T[n*n];
    P = NEW_NOTHROW T[n*n];
    mat_LU_decompose(A,L,U,P,n);

    T *L_inv = NEW_NOTHROW T[n*n];
    T *U_inv = NEW_NOTHROW T[n*n];

    memset(L_inv,0,n*n*sizeof(T));
    mat_forward_sub(L,L_inv,n);

    memset(U_inv,0,n*n*sizeof(T));
    mat_back_sub(U,U_inv,n);

    // decomposed matrices no longer required
    delete[] L;
    delete[] U;

    T *inv_unpivoted = matrix_multiply(U_inv,L_inv,n);
    T *inv_pivoted = matrix_multiply(inv_unpivoted, P, n);

    //check sanity of results
    for(uint16_t i = 0; i < n; i++) {
        for(uint16_t j = 0; j < n; j++) {
            if(isnan(inv_pivoted[i*n+j]) || isinf(inv_pivoted[i*n+j])){
                ret = false;
            }
        }
    }
    memcpy(inv,inv_pivoted,n*n*sizeof(T));

    //free memory
    delete[] inv_pivoted;
    delete[] inv_unpivoted;
    delete[] P;
    delete[] U_inv;
    delete[] L_inv;
    return ret;
}

/*
 *    fast matrix inverse code only for 3x3 square matrix
 *
 *    @param     m,           input 4x4 matrix
 *    @param     invOut,      Output inverted 4x4 matrix
 *    @returns                false = matrix is Singular, true = matrix inversion successful
 */
template<typename T>
static bool inverse3x3(const T m[], T invOut[])
{
    T inv[9];
    // computes the inverse of a matrix m
    T  det = m[0] * (m[4] * m[8] - m[7] * m[5]) -
    m[1] * (m[3] * m[8] - m[5] * m[6]) +
    m[2] * (m[3] * m[7] - m[4] * m[6]);
    if (is_zero(det) || isinf(det)) {
        return false;
    }

    T invdet = 1 / det;

    inv[0] = (m[4] * m[8] - m[7] * m[5]) * invdet;
    inv[1] = (m[2] * m[7] - m[1] * m[8]) * invdet;
    inv[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
    inv[3] = (m[5] * m[6] - m[3] * m[8]) * invdet;
    inv[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
    inv[5] = (m[3] * m[2] - m[0] * m[5]) * invdet;
    inv[6] = (m[3] * m[7] - m[6] * m[4]) * invdet;
    inv[7] = (m[6] * m[1] - m[0] * m[7]) * invdet;
    inv[8] = (m[0] * m[4] - m[3] * m[1]) * invdet;

    for(uint16_t i = 0; i < 9; i++){
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
template<typename T>
static bool inverse4x4(const T m[],T invOut[])
{
    T inv[16], det;
    uint16_t i;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    //disable FE_INEXACT detection as it fails on mac os runs
    int old = fedisableexcept(FE_INEXACT | FE_OVERFLOW);
    if (old < 0) {
        // hal.console->printf("inverse4x4(): warning: error on disabling FE_OVERFLOW floating point exception\n");
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

    if (is_zero(det) || isinf(det)){
        return false;
    }

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;
    
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (old >= 0 && feenableexcept(old) < 0) {
        // hal.console->printf("inverse4x4(): warning: error on restoring floating exception mask\n");
    }
#endif

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
template<typename T>
bool mat_inverse(const T x[], T y[], uint16_t dim)
{
    switch(dim){
    case 3: return inverse3x3(x,y);
    case 4: return inverse4x4(x,y);
    default: return mat_inverseN(x,y,dim);
    }
}

template <typename T>
void mat_mul(const T *A, const T *B, T *C, uint16_t n)
{
    memset(C, 0, sizeof(T)*n*n);
    for(uint16_t i = 0; i < n; i++) {
        for(uint16_t j = 0; j < n; j++) {
            for(uint16_t k = 0;k < n; k++) {
                C[i*n + j] += A[i*n + k] * B[k*n + j];
            }
        }
    }
}

template <typename T>
void mat_identity(T *A, uint16_t n)
{
    memset(A, 0, sizeof(T)*n*n);
    for (uint16_t i=0; i<n; i++) {
        A[i*n+i] = 1;
    }
}

template bool mat_inverse<float>(const float x[], float y[], uint16_t dim);
template void mat_mul<float>(const float *A, const float *B, float *C, uint16_t n);
template void mat_identity<float>(float x[], uint16_t dim);

template bool mat_inverse<double>(const double x[], double y[], uint16_t dim);
template void mat_mul<double>(const double *A, const double *B, double *C, uint16_t n);
template void mat_identity<double>(double x[], uint16_t dim);
