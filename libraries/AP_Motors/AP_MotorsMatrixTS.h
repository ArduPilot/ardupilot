/// @file	AP_MotorsMatrixTS.h
/// @brief	Motor control class for tailsitters with multicopter motor configurations

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsMatrix.h"

/// @class      AP_MotorsMatrix
class AP_MotorsMatrixTS : public AP_MotorsMatrix {
public:

    AP_MotorsMatrixTS(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz) {
        AP_Param::setup_object_defaults(this, var_info);
    };

    virtual void        output_to_motors() override;

    virtual void        output_motor_mask(float thrust, uint8_t mask, float rudder_dt) override;

protected:
    // configures the motors for the defined frame_class and frame_type
    virtual void        setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // calculate motor outputs
    void                output_armed_stabilizing() override;
};
