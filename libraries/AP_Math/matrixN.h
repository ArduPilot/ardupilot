/*
 *  N dimensional matrix operations
 */

#pragma once

#include "math.h"
#include <stdint.h>
#include "vectorN.h"
#include "matrixRC.h"
#include <AP_Math/AP_Math.h>

template <typename T, uint8_t N>
class VectorN;

template <typename T, uint8_t R, uint8_t C>
class MatrixRC;

template <typename T, uint8_t N>
class MatrixN {

    friend class VectorN<T,N>;

public:
    // constructor from zeros
    MatrixN<T,N>(void) {
        memset(v, 0, sizeof(v));
    }

    // constructor from 4 diagonals
    MatrixN<T,N>(const float d[N]) {
        memset(v, 0, sizeof(v));
        for (uint8_t i = 0; i < N; i++) {
            v[i][i] = d[i];
        }
    }

    // constructor constant diagonal
    MatrixN<T,N>(const float d) {
        memset(v, 0, sizeof(v));
        for (uint8_t i = 0; i < N; i++) {
            v[i][i] = d;
        }
    }

    // multiply two vectors to give a matrix, in-place
    void mult(const VectorN<T,N> &A, const VectorN<T,N> &B);

    // subtract B from the matrix
    MatrixN<T,N> &operator -=(const MatrixN<T,N> &B);

    // add B to the matrix
    MatrixN<T,N> &operator +=(const MatrixN<T,N> &B);

    // Matrix symmetry routine
    void force_symmetry(void);

    // get a element from the matrix
    float get(const uint8_t i, const uint8_t j) const;

    // set a element to the matrix
    void set(const uint8_t i, const uint8_t j, const float value);

    // multiply rectangular matrix A by the transpose of rectangular matrix B to give a square matrix, in-place
    void mult_trans(const MatrixRC<T,N,11> &A, const MatrixRC<T,N,11> &B);
    void mult_trans(const MatrixRC<T,N,3> &A, const MatrixRC<T,N,3> &B);

    // add a vector to the diagonals of the matrix, in-place
    void diag_add(const VectorN<T,N> &A);

    // invert, note no error checking!
    MatrixN<T,N> inverse_matN();

    bool is_safe();

private:
    T v[N][N];
};
