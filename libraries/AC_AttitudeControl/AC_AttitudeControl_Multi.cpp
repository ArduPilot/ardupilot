// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl_Multi.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// get lean angle max for pilot input that prioritises altitude hold over lean angle
float AC_AttitudeControl_Multi::get_althold_lean_angle_max() const
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.throttle_max();
    return ToDeg(acos(constrain_float(_throttle_in_filt.get()/(0.9f * thr_max / 1000.0f), 0.0f, 1000.0f) / 1000.0f)) * 100.0f;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
float AC_AttitudeControl_Multi::get_boosted_throttle(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // reduces as a function of angle beyond 60 degrees
    // becomes zero at 90 degrees
    float min_throttle = _motors_multi.throttle_min();
    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(2.0f*cos_tilt, 0.0f, 1.0f);
    float boost_factor = 1.0f/constrain_float(cos_tilt, 0.5f, 1.0f);

    float throttle_out = (throttle_in-min_throttle)*inverted_factor*boost_factor + min_throttle;
    _angle_boost = constrain_float(throttle_out - throttle_in,-32000,32000);
    return throttle_out;
}
