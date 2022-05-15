#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#if HAL_HAVE_HARDWARE_DOUBLE

#include "AP_MotorsMatrix.h"
#include <AP_Math/DynamicMatrix.h>

class AP_MotorsMatrix_Optimal : public AP_MotorsMatrix {
public:

    using AP_MotorsMatrix::AP_MotorsMatrix;

    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    void output_armed_stabilizing() override;

private:

    // number of motors
    uint8_t num_motors;

    // motor output factors
    DynamicMatrixd motor_factors{_heap, error_flag};

    // weighting matrix
    DynamicMatrixd w{_heap, error_flag};

    // Hessian matrix
    DynamicMatrixd H{_heap, error_flag};

    // Target values
    DynamicMatrixd f{_heap, error_flag};

    // Constraints matrix
    DynamicMatrixd A{_heap, error_flag};

    // Constraint values
    DynamicMatrixd b{_heap, error_flag};

    // Input values
    DynamicMatrixd input{_heap, error_flag};

    // motor outputs
    DynamicMatrixd out{_heap, error_flag};

    void *_heap;

    bool error_flag;
};

#endif // HAL_HAVE_HARDWARE_DOUBLE

