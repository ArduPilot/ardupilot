// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	PID.cpp
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#include <math.h>
#include <avr/eeprom.h>

#include "PID.h"

// make the gain array members a little easier to identify
#define	_kp		_gain_array[0]
#define	_ki		_gain_array[1]
#define	_kd		_gain_array[2]
#define	_imax	_gain_array[3]

long
PID::get_pid(int32_t error, uint16_t dt, float scaler)
{
	float output		= 0;
 	float delta_time	= (float)dt / 1000.0;

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
	if ((_gain_array == &_local_gains[0]) && (_address)) {
		_kp 	= (float)(eeprom_read_word((uint16_t *)	 _address))      / 1000.0;
		_ki 	= (float)(eeprom_read_word((uint16_t *)	(_address + 2))) / 1000.0;
		_kd 	= (float)(eeprom_read_word((uint16_t *)	(_address + 4))) / 1000.0;
		_imax 	= eeprom_read_word((uint16_t *)	(_address + 6)) * 100;
	}
	// load is a NOP if the gain array is managed externally
}

void
PID::save_gains()
{
	if ((_gain_array == &_local_gains[0]) && (_address)) {
		eeprom_write_word((uint16_t *)	 _address, 		(int)(_kp * 1000));
		eeprom_write_word((uint16_t *)	(_address + 2), (int)(_ki * 1000));
		eeprom_write_word((uint16_t *)	(_address + 4), (int)(_kd * 1000));
		eeprom_write_word((uint16_t *)	(_address + 6), (int)_imax/100);
	}
	// save is a NOP if the gain array is managed externally
}

