//
// Unit tests for the AP_Math rotations code
//

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define MAT_ALG_ACCURACY    1e-4f

typedef float Ftype;

static uint16_t get_random(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 65535) + (m_z >> 16);
    m_w = 18000 * (m_w & 65535) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xF;
}


static void show_matrix(Ftype *A, int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++)
            printf("%.10f  ", A[i * n + j]);
        printf("\n");
    }
}

static bool compare_mat(const Ftype *A, const Ftype *B, const uint8_t n)
{
    for(uint8_t i = 0; i < n; i++) {
        for(uint8_t j = 0; j < n; j++) {
            if(fabsf(A[i*n + j] - B[i*n + j]) > MAT_ALG_ACCURACY) {
                return false;
            }
        }
    }
    return true;
}

static void test_matrix_inverse(void)
{
    //fast inverses
    Ftype test_mat[25],ident_mat[25];
    Ftype out_mat[25], out_mat2[25], mat[25];
    for(uint8_t i = 0;i<25;i++) {
        test_mat[i] = powf(-1,i)*get_random()/0.7f;
    }


    //Test for 3x3 matrix
    mat_identity(ident_mat, 3);
    if (mat_inverse(test_mat,mat,3) && mat_inverse(mat, out_mat2, 3)) {
        mat_mul(test_mat, mat, out_mat, 3);
    } else {
        hal.console->printf("3x3 Matrix is Singular!\n");
        return;

    }
    printf("\n\n3x3 Test Matrix:\n");
    show_matrix(test_mat,3);
    printf("\nInverse of Inverse of matrix\n");
    show_matrix(mat,3);
    printf("\nInv(A) * A\n");
    show_matrix(out_mat,3);
    printf("\n");

    // compare matrix
    if (!compare_mat(test_mat, out_mat2, 3)) {
        printf("Test Failed!!\n");
        return;
    }
    if (!compare_mat(ident_mat, out_mat, 3)) {
        printf("Identity output Test Failed!!\n");
        return;
    }


    //Test for 4x4 matrix
    mat_identity(ident_mat, 4);
    if (mat_inverse(test_mat, mat, 4) && mat_inverse(mat, out_mat2, 4)){
        mat_mul(test_mat, mat, out_mat, 4);
    } else {
        hal.console->printf("4x4 Matrix is Singular!\n");
        return;
    }
    printf("\n\n4x4 Test Matrix:\n");
    show_matrix(test_mat,4);
    printf("\nInverse of Inverse of matrix\n");
    show_matrix(mat,4);
    printf("\nInv(A) * A\n");
    show_matrix(out_mat,4);
    printf("\n");
    if (!compare_mat(test_mat, out_mat2, 4)) {
        printf("Test Failed!!\n");
        return;
    }
    if (!compare_mat(ident_mat,out_mat,4)) {
        printf("Identity output Test Failed!!\n");
        return;
    }

    //Test for 5x5 matrix
    mat_identity(ident_mat, 5);
    if (mat_inverse(test_mat,mat,5) && mat_inverse(mat, out_mat2, 5)) {
        mat_mul(test_mat, mat, out_mat, 5);
    } else {
        hal.console->printf("5x5 Matrix is Singular!\n");
        return;
    }

    printf("\n\n5x5 Test Matrix:\n");
    show_matrix(test_mat,5);
    printf("\nInverse of Inverse of matrix\n");
    show_matrix(mat,5);
    printf("\nInv(A) * A\n");
    show_matrix(out_mat,5);
    printf("\n");
    if (!compare_mat(test_mat, out_mat2, 5)) {
        printf("Test Failed!!\n");
        return;
    }
    if (!compare_mat(ident_mat, out_mat, 5)) {
        printf("Identity output Test Failed!!\n");
        return;
    }

    hal.console->printf("All tests succeeded!!\n");
}


void setup(void)
{
    hal.console->printf("Matrix Algebra test\n\n");
    test_matrix_inverse();
    hal.console->printf("Matrix Algebra tests done\n\n");
}

void loop(void) {}

AP_HAL_MAIN();
