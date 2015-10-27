/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Unit tests for the AP_Math rotations code
//

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static void test_matrix_inverse(void)
{
    //fast inverses
    float mat5x5[] = {1.0f, 2.0f, 0.3f, 0.4f, 0.3f,   \
                      0.5f, 6.0f, 0.7f, 0.8f, 0.4f,   \
                      0.9f, 1.0f, 1.1f, 1.2f, 0.5f,   \
                      1.3f, 1.4f, 1.5f, 1.6f, 0.6f,   \
                      0.5f, 0.7f, 0.7f, 0.8f, 0.4f};

    float mat[25];
    if(inverse(mat5x5,mat,3)){
        inverse(mat,mat,3);
        for(uint8_t i=0;i<9;i++) {
            if(fabsf(mat5x5[i] - mat[i]) > 1.0f) {
                hal.console->printf("Matrix Inverse Failed for 3x3 matrix %f,%f!\n",mat5x5[i],mat[i]);
                return;
            }
        }
    } else {
        hal.console->printf("3x3 Matrix is Singular!\n");
        return;

    }
    if(inverse(mat5x5,mat,4)){
        inverse(mat,mat,4);
        for(uint8_t i=0;i<16;i++) {
            if(fabsf(mat5x5[i] - mat[i]) > 1.0f) {
                hal.console->printf("Matrix Inverse Failed for 4x4 matrix %f,%f!\n",mat5x5[i],mat[i]);
                return;
            }
        }
    } else {
        hal.console->printf("4x4 Matrix is Singular!\n");
        return;
    }
    if(inverse(mat5x5,mat,5)){
        inverse(mat,mat,5);
        for(uint8_t i=0;i<25;i++) {
            if(fabsf(mat5x5[i] - mat[i]) > 1.0f) {
                hal.console->printf("Matrix Inverse Failed for 5x5 matrix %f,%f!\n",mat5x5[i],mat[i]);
                return;
            }
        }
    } else {
        hal.console->printf("5x5 Matrix is Singular!\n");
        return;
    }
    hal.console->printf("All tests succeeded!!\n");
}

/*
 *  rotation tests
 */
void setup(void)
{
    hal.console->println("Matrix Algebra test\n");
    test_matrix_inverse();
    hal.console->println("Matrix Algebra tests done\n");
}

void loop(void) {}

AP_HAL_MAIN();
