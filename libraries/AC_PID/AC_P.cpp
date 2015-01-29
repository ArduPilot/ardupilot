// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_P.cpp
/// @brief	Generic P algorithm

#include <AP_Math.h>
#include "AC_P.h"

const AP_Param::GroupInfo AC_P::var_info[] PROGMEM = {
    // @Param: P
    // @DisplayName: PI 比例增益
    // @Description: P 增益会输出一个值，该值正比于当前误差值。
    AP_GROUPINFO("P",    0, AC_P, _kp, 0),
    AP_GROUPEND
};

float AC_P::get_p(float error) const
{
    return (float)error * _kp;
}

void AC_P::load_gains()
{
    _kp.load();
}

void AC_P::save_gains()
{
    _kp.save();
}
