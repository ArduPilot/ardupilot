// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//	Initial Code by Jon Challinger
//  Modified by Paul Riseborough
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.

#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_Common.h>
#include "AP_PitchController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_PitchController::var_info[] PROGMEM = {
	AP_GROUPINFO("OMEGA",        0, AP_PitchController, _kp_angle,       1.0),
	AP_GROUPINFO("K_P",        1, AP_PitchController, _kp_ff,          0.4),
	AP_GROUPINFO("K_D",        2, AP_PitchController, _kp_rate,        0.0),
	AP_GROUPINFO("K_I",        3, AP_PitchController, _ki_rate,        0.0),
	AP_GROUPINFO("RMAX_U",    4, AP_PitchController, _max_rate_pos,   0.0),
	AP_GROUPINFO("RMAX_D",    5, AP_PitchController, _max_rate_neg,   0.0),
	AP_GROUPINFO("K_RLL",    6, AP_PitchController, _roll_ff,        1.0),
	AP_GROUPEND
};

// Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
// A positive demand is up
// Inputs are: 
// 1) demanded pitch angle in centi-degrees
// 2) control gain scaler = scaling_speed / aspeed
// 3) boolean which is true when stabilise mode is active
// 4) minimum FBW airspeed (metres/sec)
// 5) maximum FBW airspeed (metres/sec)
//
int32_t AP_PitchController::get_servo_out(int32_t angle, float scaler, bool stabilize, int16_t aspd_min, int16_t aspd_max)
{
	uint32_t tnow = hal.scheduler->millis();
	uint32_t dt = tnow - _last_t;
	
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	
	if(_ahrs == NULL) return 0;
	float delta_time    = (float)dt / 1000.0f;
	
	// Calculate offset to pitch rate demand required to maintain pitch angle whilst banking
	// Calculate ideal turn rate from bank angle and airspeed assuming a level coordinated turn
	// Pitch rate offset is the component of turn rate about the pitch axis
	float aspeed;
	float rate_offset;
	float bank_angle = _ahrs->roll;
	// limit bank angle between +- 80 deg if right way up
	if (fabsf(bank_angle) < 1.5707964f)	{
	    bank_angle = constrain_float(bank_angle,-1.3962634f,1.3962634f);
	}
	if (!_ahrs->airspeed_estimate(&aspeed)) {
	    // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aspd_min) + float(aspd_max));
	}
    rate_offset = fabsf(ToDeg((9.807f / max(aspeed , float(aspd_min))) * tanf(bank_angle) * sinf(bank_angle))) * _roll_ff;     
	
	//Calculate pitch angle error in centi-degrees
	int32_t angle_err = angle - _ahrs->pitch_sensor;

	// Calculate the desired pitch rate (deg/sec) from the angle error
	float desired_rate = angle_err * 0.01f * _kp_angle;
	
	// limit the maximum pitch rate demand
	if (_max_rate_neg && desired_rate < -_max_rate_neg) {
        desired_rate = -_max_rate_neg;
    } else if (_max_rate_pos && desired_rate > _max_rate_pos) {
        desired_rate = _max_rate_pos;
    }
	
	// Apply the turn correction offset
	desired_rate = desired_rate + rate_offset;

	// Get body rate vector (radians/sec)
	float omega_y = _ahrs->get_gyro().y;
	
	// Calculate the pitch rate error (deg/sec) and scale
	float rate_error = (desired_rate - ToDeg(omega_y)) * scaler;
	
	// Multiply pitch rate error by _ki_rate and integrate
	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	if (!stabilize) {
		//only integrate if gain and time step are positive and airspeed above min value.
		if ((fabsf(_ki_rate) > 0) && (dt > 0) && (aspeed > float(aspd_min)))
		{
		    float integrator_delta = rate_error * _ki_rate * scaler * delta_time;
			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -45) integrator_delta = max(integrator_delta , 0);
			// prevent the integrator from decreasing if surface defln demand  is below the lower limit
			else if (_last_out > 45) integrator_delta = min(integrator_delta , 0);
			_integrator += integrator_delta;
		}
	} else {
		_integrator = 0;
	}
	
	// Calculate the demanded control surface deflection
	// Note the scaler is applied again. We want a 1/speed scaler applied to the feed-forward
	// path, but want a 1/speed^2 scaler applied to the rate error path. 
	// This is because acceleration scales with speed^2, but rate scales with speed.
	_last_out = ( (rate_error * _kp_rate) + _integrator + (desired_rate * _kp_ff) ) * scaler;
	
	// Convert to centi-degrees and constrain
	return constrain_float(_last_out * 100, -4500, 4500);
}

void AP_PitchController::reset_I()
{
	_integrator = 0;
}
