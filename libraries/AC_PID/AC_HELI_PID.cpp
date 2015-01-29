 // -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AC_HELI_PID.cpp
/// @brief  Generic PID algorithm

#include <AP_Math.h>
#include "AC_HELI_PID.h"

const AP_Param::GroupInfo AC_HELI_PID::var_info[] PROGMEM = {
    // @Param: P
    // @DisplayName: PID比例增益
    // @Description: P增益会输出一个值，该值正比于当前误差值。
    AP_GROUPINFO("P",    0, AC_HELI_PID, _kp, 0),
    // @Param: I
    // @DisplayName: PID积分增益
    // @Description: I增益会输出一个值，该值正比于当前误差值的幅度和持续时间。
    AP_GROUPINFO("I",    1, AC_HELI_PID, _ki, 0),
    // @Param: D
    // @DisplayName: PID微分增益
    // @Description: D增益产生的输出是正比于误差的变化率。
    AP_GROUPINFO("D",    2, AC_HELI_PID, _kd, 0),
    // @Param: IMAX
    // @DisplayName: PID最大积分
    // @Description: I项可以输出的最大/最小值。
    AP_GROUPINFO("IMAX", 3, AC_HELI_PID, _imax, 0),
    // @Param: FC
    // @DisplayName: PID+FF前馈增益
    // @Description: FF增益会输出一个值，该值正比于当前误差值。
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
