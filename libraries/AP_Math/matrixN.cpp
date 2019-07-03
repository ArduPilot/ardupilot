/*
 *  N dimensional matrix operations
 */

#pragma GCC optimize("O3")

#include "matrixN.h"


// multiply two vectors to give a matrix, in-place
template <typename T, uint8_t N>
void MatrixN<T,N>::mult(const VectorN<T,N> &A, const VectorN<T,N> &B)
{
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            v[i][j] = A[i] * B[j];
        }
    }
}

// subtract B from the matrix
template <typename T, uint8_t N>
MatrixN<T,N> &MatrixN<T,N>::operator -=(const MatrixN<T,N> &B)
{
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            v[i][j] -= B.v[i][j];
        }
    }
    return *this;
}

// add B to the matrix
template <typename T, uint8_t N>
MatrixN<T,N> &MatrixN<T,N>::operator +=(const MatrixN<T,N> &B)
{
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            v[i][j] += B.v[i][j];
        }
    }
    return *this;
}

// Matrix symmetry routine
template <typename T, uint8_t N>
void MatrixN<T,N>::force_symmetry(void)
{
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < (i - 1); j++) {
            v[i][j] = (v[i][j] + v[j][i]) / 2;
            v[j][i] = v[i][j];
        }
    }
}

// get a element from the matrix
template <typename T, uint8_t N>
float MatrixN<T,N>::get(const uint8_t i, const uint8_t j) const
{
    return v[i][j];
}

// set a element to the matrix
template <typename T, uint8_t N>
void MatrixN<T,N>::set(const uint8_t i, const uint8_t j, const float value)
{
    v[i][j] = value;
}

// multiply rectangular matrix A by the transpose of rectangular matrix B to give a square matrix, in-place
template <typename T, uint8_t N>
void MatrixN<T,N>::mult_trans(const MatrixRC<T,N,11> &A, const MatrixRC<T,N,11> &B)
{
    for (uint8_t i=0; i<N; i++) {
        for (uint8_t n=0; n<N; n++) {
            for (uint8_t j=0; j<11; j++) {
                v[i][n] += A.get(i,j) * B.get(n,j);
            }
        }
    }
}

// multiply rectangular matrix A by the transpose of rectangular matrix B to give a square matrix, in-place
template <typename T, uint8_t N>
void MatrixN<T,N>::mult_trans(const MatrixRC<T,N,3> &A, const MatrixRC<T,N,3> &B)
{
    for (uint8_t i=0; i<N; i++) {
        for (uint8_t n=0; n<N; n++) {
            for (uint8_t j=0; j<3; j++) {
                v[i][n] += A.get(i,j) * B.get(n,j);
            }
        }
    }
}

// add a vector to the diagonals of the matrix, in-place
template <typename T, uint8_t N>
void MatrixN<T,N>::diag_add(const VectorN<T,N> &A)
{
    for (uint8_t i = 0; i < N; i++) {
        v[i][i] += A[i];
    }
}

// invert, note no error checking!
template <typename T, uint8_t N>
MatrixN<T,N> MatrixN<T,N>::inverse_matN()
{
    MatrixN<T,N> ret;
    float input[N*N];

    // convert to array
    uint16_t index = 0;
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            input[index] = v[i][j];
            index++;
        }
    }

    float output[N*N];
    inverse(input,output,(uint16_t)N);

    // convert back again
    index = 0;
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            ret.v[i][j] = output[index];
            index++;
        }
    }
    return ret;
}

// Check if it is safe to use this matrix
template <typename T, uint8_t N>
bool MatrixN<T,N>::is_safe(void)
{
    for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            if (isinf(v[i][j]) || isnan(v[i][j])) {
                return false;
            }
        }
    }
    return true;
}

template void MatrixN<float,4>::mult(const VectorN<float,4> &A, const VectorN<float,4> &B);

template MatrixN<float,4> &MatrixN<float,4>::operator -=(const MatrixN<float,4> &B);
template MatrixN<float,11> &MatrixN<float,11>::operator -=(const MatrixN<float,11> &B);

template MatrixN<float,4> &MatrixN<float,4>::operator +=(const MatrixN<float,4> &B);

template float MatrixN<float,3>::get(const uint8_t i, const uint8_t j) const;
template float MatrixN<float,11>::get(const uint8_t i, const uint8_t j) const;

template void MatrixN<float,4>::force_symmetry(void);
template void MatrixN<float,11>::force_symmetry(void);

template void MatrixN<float,3>::mult_trans(const MatrixRC<float,3,11> &A, const MatrixRC<float,3,11> &B);
template void MatrixN<float,11>::mult_trans(const MatrixRC<float,11,3> &A, const MatrixRC<float,11,3> &B);

template void MatrixN<float,3>::diag_add(const VectorN<float,3> &A);
template void MatrixN<float,11>::diag_add(const VectorN<float,11> &A);

template MatrixN<float,3> MatrixN<float,3>::inverse_matN();

template bool MatrixN<float,11>::is_safe();
