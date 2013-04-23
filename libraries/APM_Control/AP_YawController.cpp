// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//	Code by Jon Challinger
//  Modified by Paul Riseborough to implement a three loop autopilot
//  topology
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.

#include <AP_Math.h>
#include <AP_HAL.h>
#include "AP_YawController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_YawController::var_info[] PROGMEM = {
	AP_GROUPINFO("K_A",    0, AP_YawController, _K_A,       0),
	AP_GROUPINFO("K_I",    1, AP_YawController, _K_I,       0),
	AP_GROUPINFO("K_D",    2, AP_YawController, _K_D,     0),
	AP_GROUPINFO("K_RLL",   3, AP_YawController, _K_FF,   1),
	AP_GROUPEND
};

int32_t AP_YawController::get_servo_out(float scaler, bool stabilize, int16_t aspd_min, int16_t aspd_max)
{
	uint32_t tnow = hal.scheduler->millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	
	if(_ins == NULL) { // can't control without a reference
		return 0; 
	}
	
	float delta_time = (float) dt / 1000.0f;
	
	// Calculate yaw rate required to keep up with a constant height coordinated turn
	float aspeed;
	float rate_offset;
	float bank_angle = _ahrs->roll;
	// limit bank angle between +- 80 deg if right way up
	if (fabsf(bank_angle) < 1.5707964f)	{
	    bank_angle = constrain(bank_angle,-1.3962634,1.3962634f);
	}
	if (!_ahrs->airspeed_estimate(&aspeed)) {
	    // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aspd_min) + float(aspd_max));
	}
	rate_offset = (9.807f / constrain(aspeed , float(aspd_min), float(aspd_max))) * tanf(bank_angle) * cosf(bank_angle) * _K_FF;

    // Get body rate vector (radians/sec)
	float omega_z = _ahrs->get_gyro();
	
	// Apply a first order lowpass filter with a 20Hz cut-off
	// Coefficients derived using a first order hold discretisation method
	// Use of FOH discretisation increases high frequency noise rejection 
	// and reduces phase loss compared to other methods
	float rate = 0.0810026f * _last_rate_out + 0.6343426f * omega_z + 0.2846549f * _last_rate_in;
	_last_rate_out = rate;
	_last_rate_in = omega_z;
	
	// Get the accln vector (m/s^2)
	float accel_y = _ins->get_accel().y;

	// Subtract the steady turn component of rate from the measured rate
	// to calculate the rate relative to the turn requirement in degrees/sec
	float rate_hp_in = ToDeg(rate - rate_offset);
	
	// Apply a high-pass filter to the rate to washout any steady state error
	// due to bias errors in rate_offset
	// Use a cut-off frequency of omega = 0.2 rad/sec
	// Could make this adjustable by replacing 0.9960080 with (1 - omega * dt)
	float rate_hp_out = 0.9960080f * _last_rate_hp_out + rate_hp_in - _last_rate_hp_in;
	_last_rate_hp_out = rate_hp_out;
	_last_rate_hp_in = rate_hp_in;

	//Calculate input to integrator
	float integ_in = - _K_I * (_K_A * accel_y + rate_hp_out);
	
	// Apply integrator, but clamp input to prevent control saturation and freeze integrator below min FBW speed
	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	// Don't integrate if _K_D is zero as integrator will keep winding up
	if (!stabilize && _K_D > 0) {
		//only integrate if airspeed above min value
		if (aspeed > float(aspd_min))
		{
			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -45) _integrator += max(integ_in * delta_time , 0);
			// prevent the integrator from decreasing if surface defln demand  is below the lower limit
			else if (_last_out > 45) _integrator += min(integ_in * delta_time , 0);
			else _integrator += integ_in * delta_time;
		}
	} else {
		_integrator = 0;
	}
	
	// Protect against increases to _K_D during in-flight tuning from creating large control transients
	// due to stored integrator values
	if (_K_D > _K_D_last && _K_D > 0) {
	    _integrator = _K_D_last/_K_D * _integrator;
	}
	_K_D_last = _K_D;
	
	// Calculate demanded rudder deflection, +Ve deflection yaws nose right
	// Save to last value before application of limiter so that integrator limiting
	// can detect exceedance next frame
	// Scale using inverse dynamic pressure (1/V^2)
	_last_out =  _K_D * (_integrator - rate_hp_out) * scaler * scaler;

	// Convert to centi-degrees and constrain
	float out = constrain(_last_out * 100, -4500, 4500);
	return out;
}

void AP_YawController::reset_I()
{
	_integrator = 0;
}
