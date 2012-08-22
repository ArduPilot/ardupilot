// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//	Code by Jon Challinger
//
//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.

#include <math.h>

#include "AP_YawController.h"

const AP_Param::GroupInfo AP_YawController::var_info[] PROGMEM = {
	AP_GROUPINFO("P",    0, AP_YawController, _kp,       0),
	AP_GROUPINFO("I",    1, AP_YawController, _ki,       0),
	AP_GROUPINFO("IMAX", 2, AP_YawController, _imax,     0),
	AP_GROUPEND
};

int32_t AP_YawController::get_servo_out(float scaler, bool stick_movement)
{
	uint32_t tnow = millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	
	if(_imu == NULL) { // can't control without a reference
		return 0; 
	}
	
	float delta_time = (float) dt / 1000.0;
	
	if(stick_movement) {
		if(!_stick_movement) {
			_stick_movement_begin = tnow;
		} else {
			if(_stick_movement_begin < tnow-333) {
				_freeze_start_time = tnow;
			}
		}
	}
	_stick_movement = stick_movement;

	Vector3f accels = _imu->get_accel();
	
	// I didn't pull 512 out of a hat - it is a (very) loose approximation of 100*ToDeg(asin(-accels.y/9.81))
	// which, with a P of 1.0, would mean that your rudder angle would be equal to your roll angle when
	// the plane is still. Thus we have an (approximate) unit to go by.
	float error = 512 * -accels.y;
	
	// strongly filter the error
	float RC = 1/(2*M_PI*_fCut);
	error = _last_error +
	(delta_time / (RC + delta_time)) * (error - _last_error);
	_last_error = error;
	// integrator
	if(_freeze_start_time < (tnow - 2000)) {
		if ((fabs(_ki) > 0) && (dt > 0))
		{
			_integrator += (error * _ki) * scaler * delta_time;
			if (_integrator < -_imax) _integrator = -_imax;
			else if (_integrator > _imax) _integrator = _imax;
		}
	} else {
		_integrator = 0;
	}
	
	return (error * _kp * scaler) + _integrator;
}

void AP_YawController::reset_I()
{
	_integrator = 0;
}
