// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm

#include <AP_Math.h>
#include "AC_PID.h"

// Examples for _filter:
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
const float  AC_PID::_filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";

const AP_Param::GroupInfo AC_PID::var_info[] PROGMEM = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_PID, _kp, 0),
    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I",    1, AC_PID, _ki, 0),
    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("D",    2, AC_PID, _kd, 0),
    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 3, AC_PID, _imax, 0),
    AP_GROUPEND
};

int32_t AC_PID::get_p(int32_t error)
{
    return (float)error * _kp;
}

int32_t AC_PID::get_i(int32_t error, float dt)
{
    if((_ki != 0) && (dt != 0)) {
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

// This is an integrator which tends to decay to zero naturally
// if the error is zero.

int32_t AC_PID::get_leaky_i(int32_t error, float dt, float leak_rate)
{
	if((_ki != 0) && (dt != 0)){
		_integrator -= (float)_integrator * leak_rate;
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
        float derivative;
		if (isnan(_last_derivative)) {
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change			
			derivative = 0;
			_last_derivative = 0;
		} else {
			// calculate instantaneous derivative
			derivative = (input - _last_input) / dt;
		}

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        derivative = _last_derivative +
                      (dt / ( _filter + dt)) * (derivative - _last_derivative);

        // update state
        _last_input             = input;
        _last_derivative    = derivative;

        // add in derivative component
        return _kd * derivative;
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

void
AC_PID::reset_I()
{
    _integrator = 0;
	// mark derivative as invalid
    _last_derivative = NAN;
}

void
AC_PID::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _imax.load();
    _imax = abs(_imax);
}

void
AC_PID::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _imax.save();
}
