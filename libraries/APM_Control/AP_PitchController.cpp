// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//	Code by Jon Challinger
//
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
	AP_GROUPINFO("AP",        0, AP_PitchController, _kp_angle,       1.0),
	AP_GROUPINFO("FF",        1, AP_PitchController, _kp_ff,          0.4),
	AP_GROUPINFO("RP",        2, AP_PitchController, _kp_rate,        0.0),
	AP_GROUPINFO("RI",        3, AP_PitchController, _ki_rate,        0.0),
	AP_GROUPINFO("RMAX_U",    4, AP_PitchController, _max_rate_pos,   0.0),
	AP_GROUPINFO("RMAX_D",    5, AP_PitchController, _max_rate_neg,   0.0),
	AP_GROUPINFO("RLL_FF",    6, AP_PitchController, _roll_ff,        0.0),
	AP_GROUPINFO("STAB_GAIN", 7, AP_PitchController, _stabilize_gain, 1),
	AP_GROUPEND
};

int32_t AP_PitchController::get_servo_out(int32_t angle, float scaler, bool stabilize)
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
	    bank_angle = constrain(bank_angle,-1.3962634,1.3962634f);
	}
	if (!_ahrs->airspeed_estimate(&aspeed)) {
	    // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aspd_min) + float(aspd_max));
	}
    rate_offset = fabsf(ToDeg((9.807f / max(aspeed , float(aspd_min))) * tanf(bank_angle) * sinf(bank_angle))) * _roll_ff;	

	//Calculate pitch angle error in centi-degrees
	int32_t angle_err = angle - _ahrs->pitch_sensor;
	angle_err *= inv;
	
	float rate = _ahrs->get_pitch_rate_earth();
	
	float RC = 1/(2*PI*_fCut);
	rate = _last_rate +
	(delta_time / (RC + delta_time)) * (rate - _last_rate);
	_last_rate = rate;
	
	float roll_scaler = 1/constrain_float(cosf(_ahrs->roll),0.33f,1);
	
	int32_t desired_rate = angle_err * _kp_angle;
	
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
		if ((fabsf(_ki_rate) > 0) && (dt > 0))
		{
			_integrator += (rate_error * _ki_rate) * scaler * delta_time;
			if (_integrator < -4500-out) _integrator = -4500-out;
			else if (_integrator > 4500-out) _integrator = 4500-out;
		}
	} else {
		_integrator = 0;
	}
	
	return out + _integrator;
}

void AP_PitchController::reset_I()
{
	_integrator = 0;
}
