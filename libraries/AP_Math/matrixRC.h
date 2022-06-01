/*
 *  R x C rectangular matrix operations
 */

#pragma once

#include "math.h"
#include <stdint.h>
#include <AP_HAL/AP_HAL.h>

template <typename T, uint8_t R, uint8_t C>
class MatrixRC {
public:
    // constructor to zeros
    MatrixRC<T,R,C>() {
        memset(v, 0, sizeof(v));
    }

    T& operator () (uint8_t i, uint8_t j);
    const T&operator() (uint8_t i, uint8_t j) const;

    MatrixRC<T,R,C> &operator =(const T num);
    MatrixRC<T,R,C> &operator *=(const T num);

    MatrixRC<T,R,C> operator *(const T num) const;
    MatrixRC<T,R,C> operator +(const T num) const;
    MatrixRC<T,R,C> operator -(const T num) const;

    MatrixRC<T,R,C> &operator +=(const MatrixRC<T,R,C>& b);
    MatrixRC<T,R,C> &operator -=(const MatrixRC<T,R,C>& b);

    MatrixRC<T,R,C> operator +(const MatrixRC<T,R,C>& b) const;
    MatrixRC<T,R,C> operator -(const MatrixRC<T,R,C>& b) const;

    MatrixRC<T,R,C> per_element_mult(const MatrixRC<T,R,C>& b) const;
    MatrixRC<T,R,C> per_element_div(const MatrixRC<T,R,C>& b) const;
    MatrixRC<T,R,C> per_element_inv() const;

    MatrixRC<T,R,C> per_element_mult_vector_columns(const MatrixRC<T,1,C>& b) const;
    MatrixRC<T,R,C> per_element_mult_vector_rows(const MatrixRC<T,R,1>& b) const;

    T dot(const MatrixRC<T,R,C>& b) const;

    MatrixRC<T,C,R> transposed() const;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    void print(const char* name) const;
#endif

    T v[R][C];
};

template <typename T, uint8_t R>
MatrixRC<T,R,R> cholesky(const MatrixRC<T,R,R>& b);

template <typename T, uint8_t R>
MatrixRC<T,R,1> forward_sub(const MatrixRC<T,R,R>& a, const MatrixRC<T,R,1>& b);

template <typename T, uint8_t R>
MatrixRC<T,R,1> backward_sub_t(const MatrixRC<T,R,R>& a, const MatrixRC<T,R,1>& b);

template <typename T, uint8_t R, uint8_t C, uint8_t J>
MatrixRC<T,R,J> matrix_multiply(const MatrixRC<T,R,C> &a, const MatrixRC<T,C,J> &b);
