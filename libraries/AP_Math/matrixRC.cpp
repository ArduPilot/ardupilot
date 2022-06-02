/*
 * R x C rectangular matrix operations
 */

//#pragma GCC optimize("O2")

#include "matrixRC.h"
#include "AP_Math.h"

extern const AP_HAL::HAL& hal;

// get and set with ()
template <typename T, uint8_t R, uint8_t C>
T& MatrixRC<T,R,C>::operator () (uint8_t i, uint8_t j) {
    return v[i][j];
}

// const getter
template <typename T, uint8_t R, uint8_t C>
const T& MatrixRC<T,R,C>::operator () (uint8_t i, uint8_t j) const {
    return v[i][j];
}

// set all values
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> &MatrixRC<T,R,C>::operator =(const T num) {
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            v[i][j] = num;
        }
    }
    return *this;
}

// scale by constant
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> &MatrixRC<T,R,C>::operator *=(const T num) {
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            v[i][j] *= num;
        }
    }
    return *this;
}

// multiply by constant
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> MatrixRC<T,R,C>::operator *(const T num) const {
    MatrixRC<T,R,C> r;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            r.v[i][j] = v[i][j] * num;
        }
    }
    return r;
}

// add constant
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> MatrixRC<T,R,C>::operator +(const T num) const {
    MatrixRC<T,R,C> r;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            r.v[i][j] = v[i][j] + num;
        }
    }
    return r;
}

// subtract constant
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> MatrixRC<T,R,C>::operator -(const T num) const {
    MatrixRC<T,R,C> r;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            r.v[i][j] = v[i][j] - num;
        }
    }
    return r;
}

// elementwise matrix addition
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> &MatrixRC<T,R,C>::operator +=(const MatrixRC<T,R,C>& b) {
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            v[i][j] += b.v[i][j];
        }
    }
    return *this;
}

// elementwise matrix subtraction
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> &MatrixRC<T,R,C>::operator -=(const MatrixRC<T,R,C>& b) {
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            v[i][j] -= b.v[i][j];
        }
    }
    return *this;
}

// elementwise matrix addition
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> MatrixRC<T,R,C>::operator +(const MatrixRC<T,R,C>& b) const {
    MatrixRC<T,R,C> r;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            r.v[i][j] = v[i][j] + b.v[i][j];
        }
    }
    return r;
}

// elementwise matrix subtraction
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> MatrixRC<T,R,C>::operator -(const MatrixRC<T,R,C>& b) const {
    MatrixRC<T,R,C> r;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            r.v[i][j] = v[i][j] - b.v[i][j];
        }
    }
    return r;
}

// elementwise multiplication
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> MatrixRC<T,R,C>::per_element_mult(const MatrixRC<T,R,C>& b) const{
    MatrixRC<T,R,C> r;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            r.v[i][j] = v[i][j] * b.v[i][j];
        }
    }
    return r;
}

// elementwise division
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> MatrixRC<T,R,C>::per_element_div(const MatrixRC<T,R,C>& b) const{
    MatrixRC<T,R,C> r;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            r.v[i][j] = v[i][j] / b.v[i][j];
        }
    }
    return r;
}

// elementwise inversion, to faster to inverse and multiply than several per_element_div calls
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> MatrixRC<T,R,C>::per_element_inv() const {
    MatrixRC<T,R,C> r;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            r.v[i][j] = 1.0 / v[i][j];
        }
    }
    return r;
}

// elementwise multiplication, matrix by vector
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> MatrixRC<T,R,C>::per_element_mult_vector_columns(const MatrixRC<T,1,C>& b) const {
    MatrixRC<T,R,C> r;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            r.v[i][j] = v[i][j] * b.v[0][j];
        }
    }
    return r;
}
template MatrixRC<float,4,12> MatrixRC<float,4,12>::per_element_mult_vector_columns(const MatrixRC<float,1,12>& b) const;

// elementwise division, matrix by vector
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,R,C> MatrixRC<T,R,C>::per_element_mult_vector_rows(const MatrixRC<T,R,1>& b) const {
    MatrixRC<T,R,C> r;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            r.v[i][j] = v[i][j] * b.v[i][0];
        }
    }
    return r;
}
template MatrixRC<float,25,12> MatrixRC<float,25,12>::per_element_mult_vector_rows(const MatrixRC<float,25,1>& b) const;

// dot product
template <typename T, uint8_t R, uint8_t C>
T MatrixRC<T,R,C>::dot(const MatrixRC<T,R,C>& b) const {
    T r = 0;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            r +=  v[i][j] * b.v[i][j];
        }
    }
    return r;
}

// transpose
template <typename T, uint8_t R, uint8_t C>
MatrixRC<T,C,R> MatrixRC<T,R,C>::transposed() const {
    MatrixRC<T,C,R> r;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < C; j++) {
            r.v[j][i] = v[i][j];
        }
    }
    return r;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// print matrix to console for debugging
template <typename T, uint8_t R, uint8_t C>
void MatrixRC<T,R,C>::print(const char* name) const {
    hal.console->printf("Matrix: %s [%u,%u]\n", name, R, C);
    for (uint8_t i = 0; i < R; i++) {
        hal.console->printf("\t");
        for (uint8_t j = 0; j < C; j++) {
            hal.console->printf("%+0.4f", v[i][j]);
            if (j < (C - 1)) {
                hal.console->printf(", ");
            }
        }
        hal.console->printf("\n");
    }
}
#endif

template class MatrixRC<float, 1, 4>;
template class MatrixRC<float, 4, 1>;
template class MatrixRC<float, 12, 1>;
template class MatrixRC<float, 12, 4>;
template class MatrixRC<float, 12, 12>;
template class MatrixRC<float, 25, 1>;

// perform Cholesky decomposition in place
// Note that this method does not zero the upper triangle
// No need with forward_sub and backward_sub implementation
template <typename T, uint8_t R>
MatrixRC<T,R,R> cholesky(const MatrixRC<T,R,R>& b) {
    MatrixRC<T,R,R> x = b;
    for (uint8_t i = 0; i < R; i++) {
        x.v[i][i] = sqrtf(x.v[i][i]);
        T ii_inv = 1.0 / x.v[i][i];
        for (uint8_t j = i+1; j < R; j++) {
            x.v[j][i] *= ii_inv;
        }
        for (uint8_t k = i+1; k < R; k++) {
            for (uint8_t j = k; j < R; j++) {
                x.v[j][k] -= x.v[j][i]*x.v[k][i];
            }
        }
    }
    return x;
}
template MatrixRC<float,12,12> cholesky<float,12>(const MatrixRC<float,12,12>& b);

// forward substitution
// equivelent to: x = a \ b if a is lower triangular
template <typename T, uint8_t R>
MatrixRC<T,R,1> forward_sub(const MatrixRC<T,R,R>& a, const MatrixRC<T,R,1>& b) {
    MatrixRC<T,R,1> x;
    for (uint8_t i = 0; i < R; i++) {
        x.v[i][0] = b.v[i][0];
        for (uint8_t j = 0; j < i; j++) {
            x.v[i][0] -= a.v[i][j] * x.v[j][0];
        }
        x.v[i][0] /= a.v[i][i];
    }
    return x;
}
template MatrixRC<float,12,1> forward_sub<float,12>(const MatrixRC<float,12,12>& a, const MatrixRC<float,12,1>& b);

// backwards substitution, transposing a
// equivelent to: x = a' \ b if a is lower triangular
template <typename T, uint8_t R>
MatrixRC<T,R,1> backward_sub_t(const MatrixRC<T,R,R>& a, const MatrixRC<T,R,1>& b) {
    MatrixRC<T,R,1> x;
    for (int8_t i = R-1; i >= 0; i--) {
        x.v[i][0] = b.v[i][0];
        for (int8_t j = R-1; j > i; j--) {
            x.v[i][0] -= a.v[j][i]*x.v[j][0];
        }
        x.v[i][0] /= a.v[i][i];
    }
    return x;
}
template MatrixRC<float,12,1> backward_sub_t<float,12>(const MatrixRC<float,12,12>& a, const MatrixRC<float,12,1>& b);

// matrix multiplication
template <typename T, uint8_t R, uint8_t C, uint8_t J>
MatrixRC<T,R,J> matrix_multiply(const MatrixRC<T,R,C>&a, const MatrixRC<T,C,J>& b) {
    MatrixRC<T,R,J> r;
    for (uint8_t i = 0; i < R; i++) {
        for (uint8_t j = 0; j < J; j++) {
            for (uint8_t k = 0; k < C; k++) {
                r.v[i][j] += a.v[i][k] * b.v[k][j];
            }
        }
    }
    return r;
}
template MatrixRC<float,12,12> matrix_multiply<float,12,4,12>(const MatrixRC<float,12,4>&a, const MatrixRC<float,4,12>& b);
template MatrixRC<float,12,1> matrix_multiply<float,12,4,1>(const MatrixRC<float,12,4>&a, const MatrixRC<float,4,1>& b);
template MatrixRC<float,12,1> matrix_multiply<float,12,12,1>(const MatrixRC<float,12,12>&a, const MatrixRC<float,12,1>& b);
