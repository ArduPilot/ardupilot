/*
 *  N dimensional matrix operations
 */

#pragma once

#include "math.h"
#include <stdint.h>
#include "vectorN.h"

template <typename T, uint8_t N>
class VectorN;


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

    // multiply two vectors to give a matrix, in-place
    void mult(const VectorN<T,N> &A, const VectorN<T,N> &B)
    {
        for (uint8_t i = 0; i < N; i++) {
            for (uint8_t j = 0; j < N; j++) {
                v[i][j] = A[i] * B[j];
            }
        }
    }

    // subtract B from the matrix
    MatrixN<T,N> &operator -=(const MatrixN<T,N> &B)
    {
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            v[i][j] -= B.v[i][j];
        }
    }
    return *this;
    }

    // add B to the matrix
    MatrixN<T,N> &operator +=(const MatrixN<T,N> &B)
    {
        for (uint8_t i = 0; i < N; i++) {
            for (uint8_t j = 0; j < N; j++) {
                v[i][j] += B.v[i][j];
            }
        }
        return *this;
    }
    
    // Matrix symmetry routine
    void force_symmetry(void)
    {
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < (i - 1); j++) {
            v[i][j] = (v[i][j] + v[j][i]) / 2;
            v[j][i] = v[i][j];
        }
    }
}

private:
    T v[N][N];
};
