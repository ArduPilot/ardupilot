// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	PID.cpp
/// @brief	Generic PID algorithm

#include <math.h>

#include "PID.h"

long
PID::get_pid(int32_t error, uint16_t dt, float scaler)
{
	float output		= 0;
 	float delta_time	= (float)dt / 1000.0;

	// Compute proportional component
	output += error * _kp;

	// Compute derivative component if time has elapsed
	if ((fabs(_kd) > 0) && (dt > 0)) {
		float derivative = 0;
		// add in
		_filter[_filter_index] = (error - _last_error) / delta_time;
		_filter_index++;

		if(_filter_index >= PID_FILTER_SIZE)
			_filter_index = 0;

		// sum our filter
		for(uint8_t i = 0; i < PID_FILTER_SIZE; i++){
			derivative += _filter[i];
		}

		// grab result
		derivative /= PID_FILTER_SIZE;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		//float RC = 1/(2*M_PI*_fCut);
		//derivative = _last_derivative + (delta_time / (RC + delta_time)) * (derivative - _last_derivative);

		// update state
		_last_error 		= error;
		//_last_derivative    = derivative;

		// add in derivative component
		output 				+= _kd * derivative;
	}

	// scale the P and D components
	output *= scaler;

	// Compute integral component if time has elapsed
	if ((fabs(_ki) > 0) && (dt > 0)) {
		_integrator 		+= (error * _ki) * scaler * delta_time;
		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}
		output 				+= _integrator;
	}

	return output;
}


long
PID::get_pi(int32_t error, uint16_t dt, float scaler)
{
	float output		= 0;
 	float delta_time	= (float)dt / 1000.0;

	// Compute proportional component
	output += error * _kp;

	// scale the P components
	output *= scaler;

	// Compute integral component if time has elapsed
	if ((fabs(_ki) > 0) && (dt > 0)) {
		_integrator 		+= (error * _ki) * scaler * delta_time;
		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}
		output 				+= _integrator;
	}

	return output;
}

void
PID::reset_I()
{
	_integrator = 0;
	_last_error = 0;
	//_last_derivative = 0;
}

void
PID::load_gains()
{
    _group.load();
}

void
PID::save_gains()
{
    _group.save();
}
