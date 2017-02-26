/*
 *  Library for operations on matrices of arbitrary dimensions.
 *
 *  Based on the MatrixMath library created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, taken from unknown source.
 */

#pragma once

#include "math.h"
#include <stdint.h>

template <typename T>
class MatrixN {
public:
    static void matrix_copy(T* A, uint8_t n, uint8_t m, T* B);
    static void matrix_mult(T* A, T* B, uint8_t m, uint8_t p, uint8_t n, T* C);
    static void matrix_mult_transpose(T* A, T* B, uint8_t m, uint8_t p, uint8_t n, T* C);
    static void matrix_add(T* A, T* B, uint8_t m, uint8_t n, T* C);
    static void matrix_subtract(T* A, T* B, uint8_t m, uint8_t n, T* C);
    static void matrix_transpose(T* A, uint8_t m, uint8_t n, T* C);
    static void matrix_mult_scalar(T* A, float s, uint8_t m, uint8_t n, T* C);
    static bool matrix_invert(T* A, uint8_t n);
    static void matrix_force_symmetry(T* A, uint8_t n);
};
