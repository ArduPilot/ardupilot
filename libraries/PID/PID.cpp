// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	PID.cpp
/// @brief	Generic PID algorithm

#include <math.h>

#include "PID.h"

const AP_Param::GroupInfo PID::var_info[] PROGMEM = {
    AP_GROUPINFO("P",    0, PID, _kp, 0),
    AP_GROUPINFO("I",    1, PID, _ki, 0),
    AP_GROUPINFO("D",    2, PID, _kd, 0),
    AP_GROUPINFO("IMAX", 3, PID, _imax, 0),
    AP_GROUPEND
};

int32_t
PID::get_pid(int32_t error, float scaler)
{
    uint32_t tnow = millis();
    uint32_t dt = tnow - _last_t;
    float output            = 0;
    float delta_time;

    if (_last_t == 0 || dt > 1000) {
        dt = 0;
    }
    _last_t = tnow;

    delta_time = (float)dt / 1000.0;

    // Compute proportional component
    output += error * _kp;

    // Compute derivative component if time has elapsed
    if ((fabs(_kd) > 0) && (dt > 0)) {
        float derivative = (error - _last_error) / delta_time;

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        float RC = 1/(2*M_PI*_fCut);
        derivative = _last_derivative +
                     (delta_time / (RC + delta_time)) * (derivative - _last_derivative);

        // update state
        _last_error             = error;
        _last_derivative    = derivative;

        // add in derivative component
        output                          += _kd * derivative;
    }

    // scale the P and D components
    output *= scaler;

    // Compute integral component if time has elapsed
    if ((fabs(_ki) > 0) && (dt > 0)) {
        _integrator             += (error * _ki) * scaler * delta_time;
        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
        output                          += _integrator;
    }

    return output;
}

void
PID::reset_I()
{
    _integrator = 0;
    _last_error = 0;
    _last_derivative = 0;
}

void
PID::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _imax.load();
}

void
PID::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _imax.save();
}
