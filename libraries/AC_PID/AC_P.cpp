/// @file	AC_P.cpp
/// @brief	Generic P algorithm

#include <AP_Math/AP_Math.h>
#include "AC_P.h"

AC_P::AC_P(const float &initial_p, const AP_Float& accel_max_cds, const float sqrt_accel_min, const float sqrt_accel_max, const float dt) :
    AC_P_Basic(initial_p),
    _accel_max_cds(accel_max_cds),
    _sqrt_accel_min(sqrt_accel_min),
    _sqrt_accel_max(sqrt_accel_max),
    _dt(dt)
{

}

float AC_P::update(float error) const
{
    const float accel_limit = get_accel_max_radss();
    if (!_disable_sqrt_control && !is_zero(accel_limit)) {
        return sqrt_controller(error, kP(), constrain_float(accel_limit * 0.5, _sqrt_accel_min, _sqrt_accel_max), _dt);
    }
    return constrain_float(kP() * error, -accel_limit, accel_limit);
}
