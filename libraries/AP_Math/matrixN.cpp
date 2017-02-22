/*
 *  Library for operations on matrices of arbitrary dimensions.
 *
 *  Based on the MatrixMath library created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, taken from unknown source.
 */

#pragma GCC optimize("O3")
 
#include "matrixN.h"

void matrix_copy(float* A, int32_t n, int32_t m, float* B){
    int32_t i, j;
    
    for (i = 0; i < m; i++){
        for (j = 0; j < n; j++){
            B[n * i + j] = A[n * i + j];
        }
    }
}

    
// Matrix Multiplication Routine
// C = A * B
void matrix_mult(float* A, float* B, int32_t m, int32_t p, int32_t n, float* C){
    // A = input matrix (m x p)
    // B = input matrix (p x n)
    // m = number of rows in A
    // p = number of columns in A = number of rows in B
    // n = number of columns in B
    // C = output matrix = A*B (m x n)
    int32_t i, j, k;
    for (i = 0; i < m; i++){
        for (j = 0; j < n; j++){
            C[n * i + j] = 0;
            for (k = 0; k < p; k++){
                C[n * i + j]= C[n * i + j] + A[p * i + k] * B[n * k + j];
            }
        }
    }
}


// Matrix Multiplication Routine
// C = A * B(transpose)
void matrix_mult_transpose(float* A, float* B, int32_t m, int32_t p, int32_t n, float* C){
    // A = input matrix (m x p)
    // B = input matrix (n x p)
    // m = number of rows in A
    // p = number of columns in A = number of columns in B
    // n = number of rows in B
    // C = output matrix = A*B (m x n)
    int32_t i, j, k;
    for (i = 0; i < m; i++){
        for(j = 0; j < n; j++){
            C[n * i + j] = 0;
            for (k = 0; k < p; k++){
                C[n * i + j]= C[n * i + j] + A[p * i + k] * B[p * j + k];
            }
        }
    }
}


// Matrix Addition Routine
void matrix_add(float* A, float* B, int32_t m, int32_t n, float* C){
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A+B (m x n)
    int32_t i, j;
    for (i = 0; i < m; i++){
        for (j = 0; j < n; j++){
            C[n * i + j] = A[n * i + j] + B[n * i + j];
        }
    }
}


// Matrix Subtraction Routine
void matrix_subtract(float* A, float* B, int32_t m, int32_t n, float* C){
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A-B (m x n)
    int32_t i, j;
    for (i = 0; i < m; i++){
        for (j = 0; j < n; j++){
            C[n * i + j] = A[n * i + j] - B[n * i + j];
        }
    }
}


// Matrix Transpose Routine
void matrix_transpose(float* A, int32_t m, int32_t n, float* C){
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = the transpose of A (n x m)
    int32_t i, j;
    for (i = 0; i < m;i++){
        for(j = 0; j < n; j++){
            C[m * j + i]=A[n * i + j];
        }
    }
}


// Matrix Transpose Routine
void matrix_mult_scalar(float* A, float s, int32_t m, int32_t n, float* C)
{
    // A = input matrix (m x n)
    // s = scalar value to multiply by
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = the transpose of A (n x m)
    int32_t i, j;
    for (i = 0; i < m; i++){
        for(j = 0; j < n; j++){
            C[m * j + i] = A[n * j + i] * s;
        }
    }
}


// Matrix Inversion Routine
// * This function inverts a matrix based on the Gauss Jordan method.
// * Specifically, it uses partial pivoting to improve numeric stability.
// * The algorithm is drawn from those presented in 
//   NUMERICAL RECIPES: The Art of Scientific Computing.
// * The function returns 1 on success, 0 on failure.
// * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
int32_t matrix_invert(float* A, int32_t n){
    // A = input matrix AND result matrix
    // n = number of rows = number of columns in A (n x n)
    int32_t pivrow = -1;     // keeps track of current pivot row
    int32_t k, i, j;      // k: overall index along diagonal; i: row index; j: col index
    int32_t pivrows[n]; // keeps track of rows swaps to undo at end
    float tmp;      // used for finding max value and making column swaps
    
    for (k = 0; k < n; k++){
        // find pivot row, the row with biggest entry in current column
        tmp = 0;
        for (i = k; i < n; i++){
            if (fabs(A[i * n + k]) >= tmp){  // 'Avoid using other functions inside abs()?'
                tmp = fabs(A[i * n + k]);
                pivrow = i;
            }
        }
        
        // check for singular matrix
        if (A[pivrow * n + k] == 0.0f){
            // Inversion failed due to the matrix being singular
            return 0;
        }
        
        // Execute pivot (row swap) if needed
        if (pivrow != k){
            // swap row k with pivrow
            for (j = 0; j < n; j++){
                tmp = A[k * n + j];
                A[k * n + j] = A[pivrow * n + j];
                A[pivrow * n + j] = tmp;
            }
        }
        pivrows[k] = pivrow;    // record row swap (even if no swap happened)
        
        tmp = 1.0f / A[k * n + k];    // invert pivot element
        A[k * n + k] = 1.0f;        // This element of input matrix becomes result matrix
        
        // Perform row reduction (divide every element by pivot)
        for (j = 0; j < n; j++){
            A[k * n + j] = A[k * n + j] * tmp;
        }
        
        // Now eliminate all other entries in this column
        for (i = 0; i < n; i++){
            if (i != k){
                tmp = A[i * n + k];
                A[i * n + k] = 0.0f;  // The other place where in matrix becomes result mat
                for (j = 0; j < n; j++){
                    A[i * n + j] = A[i * n + j] - A[k * n + j] * tmp;
                }
            }
        }
    }
    
    // Done, now need to undo pivot row swaps by doing column swaps in reverse order
    for (k = n-1; k >= 0; k--){
        if (pivrows[k] != k){
            for (i = 0; i < n; i++){
                tmp = A[i * n + k];
                A[i * n + k] = A[i * n + pivrows[k]];
                A[i * n + pivrows[k]] = tmp;
            }
        }
    }
    return 1;
}


// Matrix symmetry Routine
void matrix_force_symmetry(float* A, int32_t n){
    // A = input matrix (m x n)
    // n = number of columns in A = number of columns in B
    int32_t i, j;
    for (i = 0; i < n; i++){
        for(j = 0; j < (i - 1); j++){
            A[n * i + j]=(A[n * i + j] + A[n * j + i]) / 2;
            A[n * j + i] = A[n * i + j];
        }
    }       
}