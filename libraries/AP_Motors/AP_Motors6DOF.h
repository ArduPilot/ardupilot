// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Motors6DOF.h
/// @brief	Motor control class for ROVs with direct control over 6DOF (or fewer) in movement

#ifndef __AP_MOTORS_6DOF_H__
#define __AP_MOTORS_6DOF_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"



/// @class      AP_MotorsMatrix
class AP_Motors6DOF : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_Motors6DOF(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
    {
    	AP_Param::setup_object_defaults(this, var_info);

    	// Set gain to the closest notch near the lower-middle
    	_gain = _gain_min;
    	for ( uint8_t i = 0 ; i < _gain_steps/2-1 ; i++ ) {
    		increase_gain();
    	}
    };

    void output_min() override;

    int16_t calc_thrust_to_pwm(float thrust_in) const;

    // output_to_motors - sends minimum values out to the motors
    void output_to_motors() override;

    // gain control to scale motor outputs
    void set_gain(float gain);
	float get_gain();
	void increase_gain();
	void decrease_gain();

    // var_info for holding Parameter information
	static const struct AP_Param::GroupInfo        var_info[];
protected:


    //Override MotorsMatrix method
    void add_motor_raw_6dof(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float climb_fac, float forward_fac, float lat_fac, uint8_t testing_order);

    void output_armed_stabilizing() override;

    // Parameters
    AP_Int8             _motor_reverse[8];
    AP_Float			_forwardVerticalCouplingFactor;
    AP_Float            _gain_min;
    AP_Float            _gain_max;
    AP_Int8             _gain_steps;

    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to throttle (climb/descent)
    float               _forward_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to forward/backward
    float               _lateral_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to lateral (left/right)

    float               _gain;
};

#endif  // AP_MOTORS6DOF
