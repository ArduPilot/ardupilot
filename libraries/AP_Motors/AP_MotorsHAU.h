/// @file	AP_MotorsHAU.h
/// @brief	Motor control class for HAU with direct control over 6DOF (or fewer) in movement

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"

/// @class      AP_MotorsMatrix
class AP_MotorsHAU : public AP_MotorsMatrix {
public:

    AP_MotorsHAU(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz) {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // Override parent
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // Override parent
    void output_min() override;

    // Map thrust input -1~1 to pwm output 1100~1900
    int16_t calc_thrust_to_pwm(float thrust_in) const;

    // output_to_motors - sends minimum values out to the motors
    void output_to_motors() override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

protected:
    // return current_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
    float               get_current_limit_max_throttle() override;

    //Override MotorsMatrix method
    void add_motor_raw_HAU(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float climb_fac, float forward_fac, float lat_fac, uint8_t testing_order);

    void output_armed_stabilizing() override;
    void output_armed_stabilizing_vectored();
    void output_armed_stabilizing_vectored_HAU();

    // Parameters
    AP_Int8             _motor_reverse[AP_MOTORS_MAX_NUM_MOTORS];
    AP_Float            _forwardVerticalCouplingFactor;

    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to throttle (climb/descent)
    float               _forward_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to forward/backward
    float               _lateral_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to lateral (left/right)

    // current limiting
    float _output_limited = 1.0f;
    float _batt_current_last = 0.0f;
	
	int16_t min_pwm_motor = 1100;
	int16_t default_pwm_servo = 1500;
};
