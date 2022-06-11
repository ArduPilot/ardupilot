#pragma once

#include <AP_HAL/AP_HAL.h>

#define use_DSP HAL_WITH_DSP && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && 0

#if use_DSP
#include <arm_math.h>
typedef arm_matrix_instance_f32 Matrix;
#else
struct Matrix {
    uint16_t numRows;
    uint16_t numCols;
    float *pData;
};
#endif

// setup matrix struct
void init_mat(Matrix *S, uint16_t numRows, uint16_t numCols, float *pData);

// element wise matrix vector multiplication
void per_element_mult_mv(const Matrix *A, float *B, Matrix *dest);

// transpose
void mat_trans(const Matrix *A, Matrix *dest);

// multiply two matrixes
void mat_mult(const Matrix *A, const Matrix *B, Matrix *dest);

// scale by constant
void vec_scale(const float *A, const float scale, float *dest, uint8_t size);

// set all values
void vec_fill(const float value, float *dest, uint8_t size);

// element wise add
void vec_add(const float *A, const float *B, float *dest, uint8_t size);

// element wise subtract
void vec_sub(const float *A, const float *B, float *dest, uint8_t size);

// element wise multiply
void vec_mult(const float *A, const float *B, float *dest, uint8_t size);

// dot product
void dot_prod(const float *A, const float *B, uint8_t size, float *dest);

// add constant
void vec_offset(const float *A, const float offset, float *dest, uint8_t size);

// element wise invert
void vec_inv(const float *A, float *dest, uint8_t size);

// matrix multiplied by vector
void mat_vec_mult(const Matrix *A, const float *B, float *dest);

// in place Cholesky factorisation
void cholesky(Matrix *A);

// forward substitution
void forward_sub(const Matrix *A, const float *B, float *dest);

// backwards substitution, transposing a
void backward_sub_t(const Matrix *A, const float *B, float *dest);

// print matrix to console, for debugging (SITL only)
void print_mat(const char* name, const Matrix *A);
