/*
 *  R by C rectangular matrix operations
 */

#pragma GCC optimize("O3")

#include "matrixRC.h"


// multiply two vectors to give a matrix, in-place
template <typename T, uint8_t R, uint8_t C>
void MatrixRC<T,R,C>::mult(const VectorN<T,R> &A, const VectorN<T,C> &B)
{
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            v[i][j] = A[i] * B[j];
        }
    }
}

// set a element in the matrix
template <typename T, uint8_t R, uint8_t C>
void MatrixRC<T,R,C>::set(const uint8_t i, const uint8_t j, const float value)
{
    v[i][j] = value;
}

// get a element from the matrix
template <typename T, uint8_t R, uint8_t C>
float MatrixRC<T,R,C>::get(const uint8_t i, const uint8_t j) const
{
    return v[i][j];
}

// multiply a rectangular matrix A of dimensions [dim1, dim2] by square matrix B of [dim2 dim2]
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> MatrixRC<T,R,C>::operator *(const MatrixN<T,C> &B)
{
    MatrixRC<T,R,C> ret;

    for (uint8_t i=0; i<R; i++) {
        for (uint8_t n=0; n<C; n++) {
            for (uint8_t j=0; j<C; j++) {
                ret.v[i][n] = ret.v[i][n] + v[i][j] * B.get(j,n);
            }
        }
    }
    return ret;
}

// multiply a rectangular matrix A of dimensions [dim1, dim2] by vector B of [dim2]
template <typename T, uint8_t R, uint8_t C>
VectorN<T,R> MatrixRC<T,R,C>::operator *(const VectorN<T,C> &B)
{
    VectorN<T,R> ret;

    for (uint8_t i=0; i<R; i++) {
        for (uint8_t j=0; j<C; j++) {
            ret[i] = ret[i] + v[i][j] * B[j];
        }
    }
    return ret;
}

// multiply the transpose of rectangular matrix A by square matrix B to give a rectangular matrix, in-place
template <typename T, uint8_t R, uint8_t C>
void MatrixRC<T,R,C>::trans_mult(const MatrixRC<T,C,R> &A, const MatrixN<T,C> &B)
{
    for (uint8_t i=0; i<R; i++) {
        for (uint8_t n=0; n<C; n++) {
            for (uint8_t j=0; j<C; j++) {
                v[i][n] = v[i][n] + A.get(j,i) * B.get(j,n);
            }
        }
    }
}

template void MatrixRC<float,3,11> ::set(const uint8_t i, const uint8_t j, const float value);
template void MatrixRC<float,11,3> ::set(const uint8_t i, const uint8_t j, const float value);

template float MatrixRC<float,3,11> ::get(const uint8_t i, const uint8_t j) const;
template float MatrixRC<float,11,3> ::get(const uint8_t i, const uint8_t j) const;

template MatrixRC<float,3,11> MatrixRC<float,3,11>::operator *(const MatrixN<float,11> &B);
template MatrixRC<float,11,3> MatrixRC<float,11,3>::operator *(const MatrixN<float,3> &B);

template VectorN<float,11> MatrixRC<float,11,3>::operator *(const VectorN<float,3> &B);


template void MatrixRC<float,11,3>::trans_mult(const MatrixRC<float,3,11> &A, const MatrixN<float,3> &B);
