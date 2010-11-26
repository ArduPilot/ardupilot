// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	PID.cpp
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#include <math.h>
#include <avr/eeprom.h>

#include "PID.h"

long
PID::get_pid(long error, long dt, float scaler)
{
	float output		= 0;
 	float delta_time	= (float)dt / 1000;

	// Compute proportional component
	output += error * _kp;

	// Compute derivative component if time has elapsed
	if (_kd && (dt > 0)) {
		float derivative = (error - _last_error) / delta_time;

		// discrete low pass filter, cuts out the 
		// high frequency noise that can drive the controller crazy
		derivative = _last_derivative + 
			((delta_time / (_RC + delta_time)) * (derivative - _last_derivative));

		// update state
		_last_error 		= error;
		_last_derivative    = derivative;

		// add in derivative component
		output 				+= _kd * derivative;
	}

	// scale the P and D components
	output *= scaler;
	
	// Compute integral component if time has elapsed
	if (_ki && (dt > 0)) {
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
PID::imax(float v)
{
	_imax = fabs(v); 
}

void
PID::load_gains()
{
	if (_address) {
		_kp 	= (float)(eeprom_read_word((uint16_t *)	 _address))      / 1000.0;
		_ki 	= (float)(eeprom_read_word((uint16_t *)	(_address + 2))) / 1000.0;
		_kd 	= (float)(eeprom_read_word((uint16_t *)	(_address + 4))) / 1000.0;
		_imax 	= eeprom_read_word((uint16_t *)	(_address + 6)) * 100;
	} else {
		_kp 	= _gain_array[0]/ 1000.0;
		_ki 	= _gain_array[1]/ 1000.0;
		_kd 	= _gain_array[2]/ 1000.0;
		_imax 	= _gain_array[3]/ 1000.0;
	}
}

void
PID::save_gains()
{
	if (_address) {
		eeprom_write_word((uint16_t *)	 _address, 		(int)(_kp * 1000));
		eeprom_write_word((uint16_t *)	(_address + 2), (int)(_ki * 1000));
		eeprom_write_word((uint16_t *)	(_address + 4), (int)(_kd * 1000));
		eeprom_write_word((uint16_t *)	(_address + 6), (int)_imax/100);
	} else {
		_gain_array[0] = _kp * 1000;
		_gain_array[1] = _ki * 1000;
		_gain_array[2] = _kd * 1000;
		_gain_array[3] = _imax * 1000;
	}
}

