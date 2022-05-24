#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#if HAL_HAVE_HARDWARE_DOUBLE

#include "AP_MotorsMatrix.h"
#include <AP_Math/matrixRC.h>

class AP_MotorsMatrix_Optimal : public AP_MotorsMatrix {
public:

    using AP_MotorsMatrix::AP_MotorsMatrix;

    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    void output_armed_stabilizing() override;

private:

    static constexpr uint8_t max_num_motors = AP_MOTORS_MAX_NUM_MOTORS;
    static constexpr uint8_t num_constraints = (max_num_motors*2) + 1;

    // motor output factors
    MatrixRC<double,max_num_motors,4> motor_factors;

    // Hessian matrix
    MatrixRC<double,max_num_motors,max_num_motors> H;

    // sparse representation of constraints matrix
    // full size matrix would be [max_num_motors, num_constraints]
    // [average_throttle', diag(max_throttle), diag(min_throttle)]
    // eg for four motors:
    // 1 1 0 0 0 1 0 0 0
    // 1 0 1 0 0 0 1 0 0
    // 1 0 0 1 0 0 0 1 0
    // 1 0 0 0 1 0 0 0 1
    struct sparse_A {
        MatrixRC<double,max_num_motors,1> average_throttle;
        MatrixRC<double,max_num_motors,1> max_throttle;
        MatrixRC<double,max_num_motors,1> min_throttle;
    } A;

    // sparse constraints matrix handling
    MatrixRC<double,max_num_motors,1> A_mult(const MatrixRC<double,num_constraints,1>& B) const;
    MatrixRC<double,num_constraints,1> At_mult(const MatrixRC<double,max_num_motors,1>& B) const;
    MatrixRC<double,max_num_motors,max_num_motors> A_mult_b_mult_At(const MatrixRC<double,num_constraints,1>& B) const;

    // solver
    MatrixRC<double,max_num_motors,1> interior_point_solve(const MatrixRC<double,max_num_motors,1> &f, const MatrixRC<double,num_constraints,1> &b) const;

};

#endif // HAL_HAVE_HARDWARE_DOUBLE

