// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	ACM_PI.cpp
/// @brief	Generic PI algorithm

#include <AP_Math.h>

#include "APM_PI.h"

const AP_Param::GroupInfo APM_PI::var_info[] PROGMEM = {
    AP_GROUPINFO("P",    0, APM_PI, _kp, 0),
    AP_GROUPINFO("I",    1, APM_PI, _ki, 0),
    AP_GROUPINFO("IMAX", 2, APM_PI, _imax, 0),
    AP_GROUPEND
};

int32_t APM_PI::get_p(int32_t error)
{
    return (float)error * _kp;
}

int32_t APM_PI::get_i(int32_t error, float dt)
{
    if(dt != 0) {
        _integrator += ((float)error * _ki) * dt;

        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
    }
    return _integrator;
}

int32_t APM_PI::get_pi(int32_t error, float dt)
{
    return get_p(error) + get_i(error, dt);
}

void
APM_PI::reset_I()
{
    _integrator = 0;
}

void
APM_PI::load_gains()
{
    _kp.load();
    _ki.load();
    _imax.load();
}

void
APM_PI::save_gains()
{
    _kp.save();
    _ki.save();
    _imax.save();
}
