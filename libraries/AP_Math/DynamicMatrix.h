#pragma once

#ifndef MATH_CHECK_INDEXES
#define MATH_CHECK_INDEXES 0
#endif

#include "math.h"
#include <stdint.h>
#include "ftype.h"

// Dynamic Matrix that can be any size at construction, removes need for template instance per size
// requires heap pointer for memory allocation
template <typename T>
class DynamicMatrix {
public:

    DynamicMatrix(void *&_heap, bool &_error_flag):heap(_heap),error_flag(_error_flag) {};
    DynamicMatrix(void *&_heap, bool &_error_flag, uint8_t _rows, uint8_t _columns);

    DynamicMatrix (const DynamicMatrix &) = default;

    ~DynamicMatrix();

    void init(uint8_t _rows, uint8_t _columns);

    DynamicMatrix<T> operator *(const T num) const;
    DynamicMatrix<T> operator /(const T num) const;
    DynamicMatrix<T> operator +(const T num) const;
    DynamicMatrix<T> operator -(const T num) const;

    void operator =(const T num);
    void operator *=(const T num);
    void operator /=(const T num);
    void operator +=(const T num);
    void operator -=(const T num);

    DynamicMatrix<T> operator +(const DynamicMatrix<T>& m) const;
    DynamicMatrix<T> operator -(const DynamicMatrix<T>& m) const;

    void operator =(const DynamicMatrix<T>& m);
    void operator +=(const DynamicMatrix<T>& m);
    void operator -=(const DynamicMatrix<T>& m);

    DynamicMatrix<T> operator *(const DynamicMatrix<T>& b) const;

    DynamicMatrix<T> per_element_mult(const DynamicMatrix<T>& b) const;
    DynamicMatrix<T> per_element_div(const DynamicMatrix<T>& b) const;

    DynamicMatrix<T> per_element_mult_mv(const DynamicMatrix<T>& b) const;
    DynamicMatrix<T> per_element_div_mv(const DynamicMatrix<T>& b) const;

    void cholesky();

    DynamicMatrix<T> forward_sub(const DynamicMatrix<T>& m) const;
    DynamicMatrix<T> backward_sub(const DynamicMatrix<T>& m) const;

    DynamicMatrix<T> transposed() const;

    T dot(const DynamicMatrix<T>& b) const;

    T length_squared() const;

    T operator () (uint8_t i, uint8_t j);
    void operator () (uint8_t i, uint8_t j, T val);

    uint8_t get_rows() const { return rows; }
    uint8_t get_columns() const { return columns; }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    void print(const char* name) const;
#endif

protected:

    T *v = nullptr;

    uint8_t rows;
    uint8_t columns;

    T& get(uint8_t i, uint8_t j);
    const T& get_c(uint8_t i, uint8_t j) const;

private:

    bool &error_flag;

    void *&heap;
};

typedef DynamicMatrix<double> DynamicMatrixd;

template <typename T>
DynamicMatrix<T> interior_point_solve(const DynamicMatrix<T> &H, const DynamicMatrix<T> &f, const DynamicMatrix<T> &A, const DynamicMatrix<T> &b, void *&heap, bool &error_flag);
