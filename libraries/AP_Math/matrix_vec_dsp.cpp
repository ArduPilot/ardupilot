#include "matrix_vec_dsp.h"
#include "AP_Math.h"

// usefull vector and matrix functions using the same format as Arm CMSIS DSP functions
// enable with use_DSP, our version of CMSIS is somewhat old, only used by GyroFFT.
// on H7 there seems to be little benfit (for floats), very dependant on varable sizes, could be significantly
// faster for some problems. Never seems to be more than 2x slower. Probably need more testing....
// not all functions have a CMSIS equivelent, some such as "cholesky" would it we were to update.

// setup matrix struct
void init_mat(Matrix &S, uint16_t numRows, uint16_t numCols, float *pData)
{
#if use_DSP
    arm_mat_init_f32(&S, numRows, numCols, (float32_t *)pData);
#else
    S.numRows = numRows;
    S.numCols = numCols;
    S.pData = pData;
#endif
}

// element wise matrix vector multiplication
void per_element_mult_mv(const Matrix &A, float *B, Matrix &dest)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if ((A.numRows != dest.numRows) || (A.numCols != dest.numCols)) {
        AP_HAL::panic("element wise matrices miss match");
    }
#endif
    for (uint8_t i = 0; i < A.numRows; i++) {
        for (uint8_t j = 0; j < A.numCols; j++) {
            dest.pData[i*A.numCols + j] = A.pData[i*A.numCols + j] * B[j];
        }
    }
}

// transpose
void mat_trans(const Matrix &A, Matrix &dest)
{
#if use_DSP
  arm_mat_trans_f32(&A, &dest);
#else
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if ((A.numRows != dest.numCols) || (A.numCols != dest.numRows)) {
        AP_HAL::panic("element wise matrices miss match");
    }
#endif
    for (uint8_t i = 0; i < A.numRows; i++) {
        for (uint8_t j = 0; j < A.numCols; j++) {
            dest.pData[j*dest.numCols + i] = A.pData[i*A.numCols + j];
        }
    }
#endif
}

// multiply two matrixes
void mat_mult(const Matrix &A, const Matrix &B, Matrix &dest)
{
#if use_DSP
    arm_mat_mult_f32(&A,&B,&dest);
#else
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if ((A.numCols != B.numRows) || (A.numRows != dest.numRows) || (B.numCols != dest.numCols)) {
        AP_HAL::panic("matrix mult size mis-match");
        return;
    }
#endif
    for (uint8_t i = 0; i < A.numRows; i++) {
        for (uint8_t j = 0; j < B.numCols; j++) {
            dest.pData[i*dest.numCols + j] = 0;
            for (uint8_t k = 0; k < A.numCols; k++) {
                dest.pData[i*dest.numCols + j] += A.pData[i*A.numCols + k] * B.pData[k*B.numCols + j];
            }
        }
    }
#endif
}

// scale by constant
void vec_scale(const float *A, const float scale, float *dest, uint8_t size)
{
#if use_DSP
    arm_scale_f32(A,scale,dest,size);
#else
    for (uint16_t i = 0; i < size; i++) {
        dest[i] =  A[i] * scale;
    }
#endif
}

// set all values
void vec_fill(const float value, float *dest, uint8_t size)
{
#if use_DSP
    arm_fill_f32(value,dest,size);
#else
    for (uint16_t i = 0; i < size; i++) {
        dest[i] =  value;
    }
#endif
}

// element wise add
void vec_add(const float *A, const float *B, float *dest, uint8_t size)
{
#if use_DSP
    arm_add_f32(A,B,dest,size);
#else
    for (uint16_t i = 0; i < size; i++) {
        dest[i] = A[i] + B[i];
    }
#endif
}

// element wise subtract
void vec_sub(const float *A, const float *B, float *dest, uint8_t size)
{
#if use_DSP
    arm_sub_f32(A,B,dest,size);
#else
    for (uint16_t i = 0; i < size; i++) {
        dest[i] =  A[i] - B[i];
    }
#endif
}

// element wise multiply
void vec_mult(const float *A, const float *B, float *dest, uint8_t size)
{
#if use_DSP
    arm_mult_f32(A,B,dest,size);
#else
    for (uint16_t i = 0; i < size; i++) {
        dest[i] =  A[i] * B[i];
    }
#endif
}

// dot product
void dot_prod(const float *A, const float *B, uint8_t size, float *dest)
{
#if use_DSP
    arm_dot_prod_f32(A,B,size,dest);
#else
    *dest = 0;
    for (uint8_t i = 0; i < size; i++) {
        *dest += A[i] * B[i];
    }
#endif
}

// add constant
void vec_offset(const float *A, const float offset, float *dest, uint8_t size)
{
#if use_DSP
    arm_offset_f32(A,offset,dest,size);
#else
    for (uint16_t i = 0; i < size; i++) {
        dest[i] = A[i] + offset;
    }
#endif
}

// element wise invert
void vec_inv(const float *A, float *dest, uint8_t size)
{
    for (uint16_t i = 0; i < size; i++) {
        dest[i] =  1.0 / A[i];
    }
}

// matrix multiplied by vector
void mat_vec_mult(const Matrix &A, const float *B, float *dest)
{
    for (uint8_t i = 0; i < A.numRows; i++) {
        dest[i] = 0;
        for (uint8_t j = 0; j < A.numCols; j++) {
            dest[i] += A.pData[i*A.numCols + j] * B[j];
        }
    }
}

// in place Cholesky factorisation
// Note that this method does not zero the upper triangle
// No need with forward_sub and backward_sub implementation
void cholesky(Matrix &A)
{
    for (uint8_t i = 0; i < A.numRows; i++) {
        A.pData[i*A.numRows + i] = safe_sqrt(A.pData[i*A.numRows + i]);
        float ii_inv = 1.0 / A.pData[i*A.numCols + i];
        for (uint8_t j = i+1; j < A.numRows; j++) {
            A.pData[j*A.numRows + i] *= ii_inv;
        }
        for (uint8_t k = i+1; k < A.numRows; k++) {
            for (uint8_t j = k; j < A.numRows; j++) {
                A.pData[j*A.numCols + k] -= A.pData[j*A.numCols + i]*A.pData[k*A.numCols + i];
            }
        }
    }
}

// forward substitution
// equivelent to: x = a \ b if a is lower triangular
void forward_sub(const Matrix &A, const float *B, float *dest) {
    for (uint8_t i = 0; i < A.numRows; i++) {
        dest[i] = B[i];
        for (uint8_t j = 0; j < i; j++) {
           dest[i] -= A.pData[i*A.numCols + j] * dest[j];
        }
        dest[i] /=  A.pData[i*A.numCols + i];
    }
}

// backwards substitution, transposing a
// equivelent to: x = a' \ b if a is lower triangular
void backward_sub_t(const Matrix &A, const float *B, float *dest) {
    for (int8_t i = A.numRows-1; i >= 0; i--) {
        dest[i] = B[i];
        for (int8_t j = A.numRows-1; j > i; j--) {
            dest[i] -= A.pData[j*A.numCols + i]*dest[j];
        }
        dest[i] /=  A.pData[i*A.numCols + i];
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// print matrix to console for debugging
extern const AP_HAL::HAL& hal;
void print_mat(const char* name, const Matrix &A) {
    hal.console->printf("Matrix: %s [%u,%u]\n", name, A.numRows, A.numCols);
    for (uint8_t i = 0; i < A.numRows; i++) {
        hal.console->printf("\t");
        for (uint8_t j = 0; j < A.numCols; j++) {
            hal.console->printf("%+0.4f", A.pData[i*A.numCols + j]);
            if (j < (A.numCols - 1)) {
                hal.console->printf(", ");
            }
        }
        hal.console->printf("\n");
    }
}
#endif
