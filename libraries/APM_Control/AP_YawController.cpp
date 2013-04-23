// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//	Code by Jon Challinger
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
	AP_GROUPINFO("P",    0, AP_YawController, _kp,       0),
	AP_GROUPINFO("I",    1, AP_YawController, _ki,       0),
	AP_GROUPINFO("IMAX", 2, AP_YawController, _imax,     0),
	AP_GROUPEND
};

// Low pass filter cut frequency for derivative calculation.
// FCUT macro computes a frequency cut based on an acceptable delay.
#define FCUT(d) (1 / ( 2 * 3.14f * (d) ) )
const float AP_YawController::_fCut = FCUT(0.5f);

int32_t AP_YawController::get_servo_out(float scaler, bool stick_movement)
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
	
	if(stick_movement) {
		if(!_stick_movement) {
			_stick_movement_begin = tnow;
		} else {
			if(_stick_movement_begin < tnow-333) {
				_freeze_start_time = tnow;
			}
		}
	}
	rate_offset = (9.807f / constrain(aspeed , float(aspd_min), float(aspd_max))) * tanf(bank_angle) * cosf(bank_angle) * _K_FF;

    // Get body rate vector (radians/sec)
	float omega_z = _ahrs->get_gyro().z;
	
	// Subtract the steady turn component of rate from the measured rate
	// to calculate the rate relative to the turn requirement in degrees/sec
	float rate_hp_in = ToDeg(omega_z - rate_offset);
	
	// Apply a high-pass filter to the rate to washout any steady state error
	// due to bias errors in rate_offset
	// Use a cut-off frequency of omega = 0.2 rad/sec
	// Could make this adjustable by replacing 0.9960080 with (1 - omega * dt)
	float rate_hp_out = 0.9960080f * _last_rate_hp_out + rate_hp_in - _last_rate_hp_in;
	_last_rate_hp_out = rate_hp_out;
	_last_rate_hp_in = rate_hp_in;

	// Get the accln vector (m/s^2)
	Vector3f accel = _ins->get_accel();

	// Calculate input to integrator
	float integ_in = - _K_I * (_K_A * accel.y + rate_hp_out);
	
	// strongly filter the error
	float RC = 1/(2*PI*_fCut);
	error = _last_error +
	(delta_time / (RC + delta_time)) * (error - _last_error);
	_last_error = error;
	// integrator
	if(_freeze_start_time < (tnow - 2000)) {
		if ((fabsf(_ki) > 0) && (dt > 0)) {
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
