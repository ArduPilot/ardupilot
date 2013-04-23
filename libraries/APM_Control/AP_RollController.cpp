// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//	Code by Jon Challinger
//  Modified by Paul Riseborough
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.

#include <AP_Math.h>
#include <AP_HAL.h>

#include "AP_RollController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RollController::var_info[] PROGMEM = {
	AP_GROUPINFO("OMEGA",  0, AP_RollController, _kp_angle,           1.0),
	AP_GROUPINFO("K_P",    1, AP_RollController, _kp_ff,              0.4),
	AP_GROUPINFO("K_D",    2, AP_RollController, _kp_rate,            0.0),
	AP_GROUPINFO("K_I",    3, AP_RollController, _ki_rate,            0.0),
	AP_GROUPINFO("RMAX",   4, AP_RollController, _max_rate,           60),
	AP_GROUPEND
};

// Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
// A positive demand is up
// Inputs are: 
// 1) demanded bank angle in centi-degrees
// 2) control gain scaler = scaling_speed / aspeed
// 3) boolean which is true when stabilise mode is active
// 4) minimum FBW airspeed (metres/sec)
//
int32_t AP_RollController::get_servo_out(int32_t angle, float scaler, bool stabilize, int16_t aspd_min)
{
	uint32_t tnow = hal.scheduler->millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	
	if(_ahrs == NULL) return 0; // can't control without a reference
	float delta_time    = (float)dt / 1000.0f;
	
	// Calculate bank angle error in centi-degrees
	int32_t angle_err = angle - _ahrs->roll_sensor;

	// Calculate the desired roll rate (deg/sec) from the angle error
	float desired_rate = angle_err * 0.01f * _kp_angle;
	
	// Limit the demanded roll rate
	if (_max_rate && desired_rate < -_max_rate) desired_rate = -_max_rate;
	else if (_max_rate && desired_rate > _max_rate) desired_rate = _max_rate;
	
    // Get body rate vector (radians/sec)
	Vector3f omega = _ahrs->get_gyro();
	
	// Apply a first order lowpass filter with a 20Hz cut-off
	// Coefficients derived using a first order hold discretisation method
	// Use of FOH discretisation increases high frequency noise rejection 
	// and reduces phase loss compared to other methods
	float rate = 0.0810026f * _last_rate_out + 0.6343426f * omega.x + 0.2846549f * _last_rate_in;
	_last_rate_out = rate;
	_last_rate_in = omega.x;

	// Calculate the roll rate error (deg/sec) and apply gain scaler
	float rate_error = (desired_rate - ToDeg(rate)) * scaler;
	
	// Get an airspeed estimate - default to zero if none available
	float aspeed;
	if (!_ahrs->airspeed_estimate(&aspeed)) aspeed = 0.0f;

	// Multiply roll rate error by _ki_rate and integrate
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
	float _last_out = ( (rate_error * _kp_rate) + _integrator + (desired_rate * _kp_ff) ) * scaler;
	
	// Convert to centi-degrees and constrain
	float out = constrain(_last_out * 100, -4500, 4500);
	return out;
}

void AP_RollController::reset_I()
{
	_integrator = 0;
}

