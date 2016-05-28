// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_HELI_PID.cpp
/// @brief	Generic PID algorithm

#include <AP_Math.h>
#include "AC_HELI_PID.h"

const AP_Param::GroupInfo AC_HELI_PID::var_info[] PROGMEM = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_HELI_PID, _kp, 0),
    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I",    1, AC_HELI_PID, _ki, 0),
    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("D",    2, AC_HELI_PID, _kd, 0),
    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 3, AC_HELI_PID, _imax, 0),
    // @Param: FC
    // @DisplayName: PID+FF FeedForward Gain
    // @Description: FF Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("FF",    4, AC_HELI_PID, _ff, 0),
    AP_GROUPEND
};

float AC_HELI_PID::get_ff(float requested_rate) const
{
    return (float)requested_rate * _ff;
}

// This is an integrator which tends to decay to zero naturally
// if the error is zero.

float AC_HELI_PID::get_leaky_i(float error, float dt, float leak_rate)
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
