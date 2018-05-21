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
	
	//Map servo angle input 0~180 to pwm output min~max servo
	int16_t calc_angle_to_pwm(float angle_in) const;

    // output_to_motors - sends minimum values out to the motors
    void output_to_motors() override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];
	
	//add new set function
	//void                set_updown(float updown_in) { _updown_in = updown_in; };     // range -1 ~ +1

protected:
    // return current_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
    float               get_current_limit_max_throttle() override;

    //Override MotorsMatrix method
    void add_motor_raw_HAU(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float climb_fac, float forward_fac, uint8_t testing_order);
	void add_servo(int8_t servo_num, bool reverse); //quick method to use motor output as servo
	void add_servo_HAU(int8_t servo_num, float roll_fac, float pitch_fac, float yaw_fac, float throttle_fac, float forward_fac, uint8_t testing_order, float direction);

    void output_armed_stabilizing() override;

    // Parameters
    AP_Int8             _motor_reverse[AP_MOTORS_MAX_NUM_MOTORS];
    AP_Float            _forwardVerticalCouplingFactor;

    float               _throttle_mfactor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to throttle (climb/descent)
    float               _forward_mfactor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to forward/backward
    float               _roll_mfactor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to lateral (left/right)
	float               _pitch_mfactor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to lateral (left/right)
	float               _yaw_mfactor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to lateral (left/right)
	
	float               _throttle_sfactor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to throttle (climb/descent)
    float               _forward_sfactor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to forward/backward
    float               _roll_sfactor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to lateral (left/right)
	float               _pitch_sfactor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to lateral (left/right)
	float               _yaw_sfactor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to lateral (left/right)
	
	float				_servo_direction[AP_MOTORS_MAX_NUM_MOTORS];
	float 				_servo_mix_out[AP_MOTORS_MAX_NUM_MOTORS]; // vector tilt servo

    // current limiting
    float _output_limited = 1.0f;
    float _batt_current_last = 0.0f;
	
	float _thrust_rpyt_mixing = 0.0f;
	float _servo_vertical_mixing = 0.0f;
	float _servo_horizontal_mixing = 0.0f;
	float _servo_mixing = 0.0f;
	const float _angle_compensation = 5.0f;
	
	const int16_t min_pwm_motor = 1100;
	int16_t default_pwm_servo = 1500;
	
	//float _updown_in;
};
