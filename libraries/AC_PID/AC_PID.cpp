// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm

#include <math.h>
#include "AC_PID.h"

const AP_Param::GroupInfo AC_PID::var_info[] PROGMEM = {
	AP_GROUPINFO("P",    0, AC_PID, _kp),
	AP_GROUPINFO("I",    1, AC_PID, _ki),
	AP_GROUPINFO("D",    2, AC_PID, _kd),
	AP_GROUPINFO("IMAX", 3, AC_PID, _imax),
	AP_GROUPEND
};

int32_t AC_PID::get_p(int32_t error)
{
	return (float)error * _kp;
}

int32_t AC_PID::get_i(int32_t error, float dt)
{
	if((_ki != 0) && (dt != 0)){
		_integrator += ((float)error * _ki) * dt;
		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}
		return _integrator;
	}
	return 0;
}

int32_t AC_PID::get_d(int32_t input, float dt)
{
	if ((_kd != 0) && (dt != 0)) {
		_derivative = (input - _last_input) / dt;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		_derivative = _last_derivative +
		        (dt / ( _filter + dt)) * (_derivative - _last_derivative);

		// update state
		_last_input 		= input;
		_last_derivative    = _derivative;

		// add in derivative component
		return _kd * _derivative;
	}
	return 0;
}

int32_t AC_PID::get_pi(int32_t error, float dt)
{
	return get_p(error) + get_i(error, dt);
}


int32_t AC_PID::get_pid(int32_t error, float dt)
{
	return get_p(error) + get_i(error, dt) + get_d(error, dt);
}



/*
int32_t AC_PID::get_pid(int32_t error, float dt)
{
	// Compute proportional component
	_output = error * _kp;

	// Compute derivative component if time has elapsed
	if ((fabs(_kd) > 0) && (dt > 0)) {
		_derivative = (error - _last_error) / dt;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		_derivative = _last_derivative +
		        (dt / ( _filter + dt)) * (_derivative - _last_derivative);

		// update state
		_last_error 		= error;
		_last_derivative    = _derivative;

		// add in derivative component
		_output 	+= _kd * _derivative;
	}

	// Compute integral component if time has elapsed
	if ((fabs(_ki) > 0) && (dt > 0)) {
		_integrator 		+= (error * _ki) * dt;
		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}
		_output 	+= _integrator;
	}

	return _output;
}
*/


void
AC_PID::reset_I()
{
	_integrator = 0;
	_last_input = 0;
	_last_derivative = 0;
}

void
AC_PID::load_gains()
{
	_kp.load();
	_ki.load();
	_kd.load();
	_imax.load();
}

void
AC_PID::save_gains()
{
	_kp.save();
	_ki.save();
	_kd.save();
	_imax.save();
}
