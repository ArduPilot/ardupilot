//
// Unit tests for the AP_Math rotations code
//

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <cstdio>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define MAT_ALG_ACCURACY    1e-4f

static uint16_t get_random(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 65535) + (m_z >> 16);
    m_w = 18000 * (m_w & 65535) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xF;
}


static void show_matrix(float *A, int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++)
            printf("%.10f  ", A[i * n + j]);
        printf("\n");
    }
}

static bool compare_mat(const float *A, const float *B, const uint8_t n)
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
    float test_mat[25],ident_mat[25];
    float *out_mat;
    for(uint8_t i = 0;i<25;i++) {
        test_mat[i] = pow(-1,i)*get_random()/0.7f;
    }
    float mat[25];


    //Test for 3x3 matrix
    memset(ident_mat,0,sizeof(ident_mat));
    for(uint8_t i=0; i<3; i++) {
        ident_mat[i*3+i] = 1.0f;
    }
    if(inverse(test_mat,mat,3)){
        out_mat = mat_mul(test_mat,mat,3);
        inverse(mat,mat,3);
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
    //compare matrix
    if(!compare_mat(test_mat,mat,3)) {
        printf("Test Failed!!\n");
        return;
    }
    if(!compare_mat(ident_mat,out_mat,3)) {
        printf("Identity output Test Failed!!\n");
        return;
    }


    //Test for 4x4 matrix
    memset(ident_mat,0,sizeof(ident_mat));
    for(uint8_t i=0; i<4; i++) {
        ident_mat[i*4+i] = 1.0f;
    }
    if(inverse(test_mat,mat,4)){
        out_mat = mat_mul(test_mat,mat,4);
        inverse(mat,mat,4);
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
    if(!compare_mat(test_mat,mat,4)) {
        printf("Test Failed!!\n");
        return;
    }
    if(!compare_mat(ident_mat,out_mat,4)) {
        printf("Identity output Test Failed!!\n");
        return;
    }



    //Test for 5x5 matrix
    memset(ident_mat,0,sizeof(ident_mat));
    for(uint8_t i=0; i<5; i++) {
        ident_mat[i*5+i] = 1.0f;
    }
    if(inverse(test_mat,mat,5)) {
        out_mat = mat_mul(test_mat,mat,5);
        inverse(mat,mat,5);
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
    if(!compare_mat(test_mat,mat,5)) {
        printf("Test Failed!!\n");
        return;
    }
    if(!compare_mat(ident_mat,out_mat,5)) {
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
