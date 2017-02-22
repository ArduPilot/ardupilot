/*
 *  Library for operations on matrices of arbitrary dimensions.
 *
 *  Based on the MatrixMath library created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, taken from unknown source.
 */

#pragma once

#include "math.h"
#include <stdint.h>

void matrix_copy(float* A, int32_t n, int32_t m, float* B);
void matrix_mult(float* A, float* B, int32_t m, int32_t p, int32_t n, float* C);
void matrix_mult_transpose(float* A, float* B, int32_t m, int32_t p, int32_t n, float* C);
void matrix_add(float* A, float* B, int32_t m, int32_t n, float* C);
void matrix_subtract(float* A, float* B, int32_t m, int32_t n, float* C);
void matrix_transpose(float* A, int32_t m, int32_t n, float* C);
void matrix_mult_scalar(float* A, float s, int32_t m, int32_t n, float* C);
int32_t matrix_invert(float* A, int32_t n);
void matrix_force_symmetry(float* A, int32_t n);
