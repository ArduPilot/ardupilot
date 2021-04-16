/// @file	AC_P_1D.cpp
/// @brief	Generic P algorithm

#include <AP_Math/AP_Math.h>
#include "AC_P_1D.h"

const AP_Param::GroupInfo AC_P_1D::var_info[] = {
    // @Param: P
    // @DisplayName: P Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_P_1D, _kp, 0),
    AP_GROUPEND
};

// Constructor
AC_P_1D::AC_P_1D(float initial_p, float dt) :
    _dt(dt)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp = initial_p;
    _error_min = 0.0f;
    _error_max = 0.0f;
    _D1_max = 0.0f;
}

// update_all - set target and measured inputs to P controller and calculate outputs
// target and measurement are filtered
// if measurement is further than error_min or error_max (see set_limits method)
//   the target is moved closer to the measurement and limit_min or limit_max will be set true
float AC_P_1D::update_all(float &target, float measurement, bool &limit_min, bool &limit_max)
{
    limit_min = false;
    limit_max = false;

    // calculate distance _error
    _error = target - measurement;

    if (is_negative(_error_min) && (_error < _error_min)) {
        _error = _error_min;
        target = measurement + _error;
        limit_min = true;
    } else if (is_positive(_error_max) && (_error > _error_max)) {
        _error = _error_max;
        target = measurement + _error;
        limit_max = true;
    }

    // MIN(_Dxy_max, _D2xy_max / _kxy_P) limits the max accel to the point where max jerk is exceeded
    return sqrt_controller(_error, _kp, _D1_max, _dt);
}

// set limits on error, output and output from D term
// in normal use the error_min and output_min will be negative
// when using for a position controller, lim_err will be position error, lim_out will be correction velocity, lim_D will be acceleration, lim_D2 will be jerk
void AC_P_1D::set_limits(float error_min, float error_max, float output_min, float output_max, float D1_max, float D2_max)
{
    _D1_max = D1_max;
    if (is_positive(D2_max)) {
        // limit the first derivative so as not to exceed the second derivative
        _D1_max = MIN(_D1_max, D2_max / _kp);
    }
    _error_min = inv_sqrt_controller(output_min, _kp, _D1_max);
    if(!is_zero(_error_min) && is_negative(error_min) ) {
        _error_min = MAX(_error_min, error_min);
    } else {
        _error_min = error_min;
    }
    _error_min = MIN(0.0f, _error_min);

    _error_max = inv_sqrt_controller(output_max, _kp, _D1_max);
    if(!is_zero(_error_max) && is_positive(error_max) ) {
        _error_max = MIN(_error_max, error_max);
    } else {
        _error_max = error_max;
    }
    _error_max = MAX(0.0f, _error_max);
}
