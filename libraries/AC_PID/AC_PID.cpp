// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm

#include <AP_Math.h>
#include "AC_PID.h"

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

float AC_PID::get_p(float error) const
{
    return (float)error * _kp;
}

float AC_PID::get_i(float error, float dt)
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

float AC_PID::get_d(float input, float dt)
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
        derivative = _last_derivative + _d_lpf_alpha * (derivative - _last_derivative);

        // update state
        _last_input             = input;
        _last_derivative    = derivative;

        // add in derivative component
        return _kd * derivative;
    }
    return 0;
}

float AC_PID::get_pi(float error, float dt)
{
    return get_p(error) + get_i(error, dt);
}


float AC_PID::get_pid(float error, float dt)
{
    return get_p(error) + get_i(error, dt) + get_d(error, dt);
}

void AC_PID::reset_I()
{
    _integrator = 0;
	// mark derivative as invalid
    _last_derivative = NAN;
}

void AC_PID::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _imax.load();
    _imax = abs(_imax);
}

void AC_PID::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _imax.save();
}

void AC_PID::set_d_lpf_alpha(int16_t cutoff_frequency, float time_step)
{    
    // calculate alpha
    float rc = 1/(2*PI*cutoff_frequency);
    _d_lpf_alpha = time_step / (time_step + rc);
}
