// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//	Code by Jon Challinger
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.

#include <math.h>
#include <AP_HAL.h>

#include "AP_RollController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RollController::var_info[] PROGMEM = {
	AP_GROUPINFO("AP",    0, AP_RollController, _kp_angle,           1.0),
	AP_GROUPINFO("FF",    1, AP_RollController, _kp_ff,              0.4),
	AP_GROUPINFO("RP",    2, AP_RollController, _kp_rate,            0.0),
	AP_GROUPINFO("RI",    3, AP_RollController, _ki_rate,            0.0),
	AP_GROUPINFO("RMAX",  4, AP_RollController, _max_rate,           0),
	AP_GROUPINFO("STAB_GAIN", 5, AP_RollController, _stabilize_gain, 1),
	AP_GROUPEND
};

int32_t AP_RollController::get_servo_out(int32_t angle, float scaler, bool stabilize)
{
	uint32_t tnow = hal.scheduler->millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	
	if(_ahrs == NULL) return 0; // can't control without a reference
	float delta_time    = (float)dt / 1000.0;
	
	int32_t angle_err = angle - _ahrs->roll_sensor;
	
	float rate = _ahrs->get_roll_rate_earth();
	
	float RC = 1/(2*M_PI*_fCut);
	rate = _last_rate +
	(delta_time / (RC + delta_time)) * (rate - _last_rate);
	_last_rate = rate;
	
	int32_t desired_rate = angle_err * _kp_angle;
	
	if (_max_rate && desired_rate < -_max_rate) desired_rate = -_max_rate;
	else if (_max_rate && desired_rate > _max_rate) desired_rate = _max_rate;
	
	if(stabilize) {
		desired_rate *= _stabilize_gain;
	}

	int32_t rate_error = desired_rate - ToDeg(rate)*100;
	
	float out = (rate_error * _kp_rate + desired_rate * _kp_ff) * scaler;
	
	//rate integrator
        if (!stabilize) {
		if ((fabs(_ki_rate) > 0) && (dt > 0))
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

void AP_RollController::reset_I()
{
	_integrator = 0;
}

