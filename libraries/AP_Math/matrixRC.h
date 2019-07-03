/*
 *  R by C rectangular matrix operations
 */

#pragma once

#include "math.h"
#include <stdint.h>
#include "vectorN.h"
#include "matrixN.h"

template <typename T, uint8_t N>
class VectorN;

template <typename T, uint8_t N>
class MatrixN;

template <typename T, uint8_t R, uint8_t C>
class MatrixRC {
public:
    // constructor from zeros
    MatrixRC<T,R,C>(void) {
        memset(v, 0, sizeof(v));
    }

    // multiply two vectors to give a matrix, in-place
    void mult(const VectorN<T,R> &A, const VectorN<T,C> &B);

    // set a element in the matrix
    void set(const uint8_t i, const uint8_t j, const float value);

    // get a element from the matrix
    float get(const uint8_t i, const uint8_t j) const;

    // multiply a rectangular matrix A of dimensions [dim1, dim2] by square matrix B of [dim2 dim2]
    MatrixRC<T,R,C> operator *(const MatrixN<T,C> &B);

    // multiply a rectangular matrix A of dimensions [dim1, dim2] by vector B of [dim2]
    VectorN<T,R> operator *(const VectorN<T,C> &B);

    // multiply the transpose of rectangular matrix A by square matrix B to give a rectangular matrix, in-place
    void trans_mult(const MatrixRC<T,C,R> &A, const MatrixN<T,C> &B);

private:
    T v[R][C];
};
