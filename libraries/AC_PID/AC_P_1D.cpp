/// @file	AC_P_1D.cpp
/// @brief	Generic P algorithm

#include <AP_Math/AP_Math.h>
#include "AC_P_1D.h"

const AP_Param::GroupInfo AC_P_1D::var_info[] = {
    // @Param: P
    // @DisplayName: P Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    0, AC_P_1D, _kp, default_kp),
    AP_GROUPEND
};

// Constructor
AC_P_1D::AC_P_1D(float initial_p) :
    default_kp(initial_p)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);
}

// update_all - set target and measured inputs to P controller and calculate outputs
// target and measurement are filtered
float AC_P_1D::update_all(float &target, float measurement)
{
    // calculate distance _error
    _error = target - measurement;

    if (is_negative(_error_min) && (_error < _error_min)) {
        _error = _error_min;
        target = measurement + _error;
    } else if (is_positive(_error_max) && (_error > _error_max)) {
        _error = _error_max;
        target = measurement + _error;
    }

    // MIN(_Dxy_max, _D2xy_max / _kxy_P) limits the max accel to the point where max jerk is exceeded
    return sqrt_controller(_error, _kp, _D1_max, 0.0);
}

// set_limits - sets the maximum error to limit output and first and second derivative of output
// when using for a position controller, lim_err will be position error, lim_out will be correction velocity, lim_D will be acceleration, lim_D2 will be jerk
void AC_P_1D::set_limits(float output_min, float output_max, float D_Out_max, float D2_Out_max)
{
    _D1_max = 0.0f;
    _error_min = 0.0f;
    _error_max = 0.0f;

    if (is_positive(D_Out_max)) {
        _D1_max = D_Out_max;
    }

    if (is_positive(D2_Out_max) && is_positive(_kp)) {
        // limit the first derivative so as not to exceed the second derivative
        _D1_max = MIN(_D1_max, D2_Out_max / _kp);
    }

    if (is_negative(output_min) && is_positive(_kp)) {
        _error_min = inv_sqrt_controller(output_min, _kp, _D1_max);
    }

    if (is_positive(output_max) && is_positive(_kp)) {
        _error_max = inv_sqrt_controller(output_max, _kp, _D1_max);
    }
}

// set_error_limits - reduce maximum error to error_max
// to be called after setting limits
void AC_P_1D::set_error_limits(float error_min, float error_max)
{
    if (is_negative(error_min)) {
        if (!is_zero(_error_min)) {
            _error_min = MAX(_error_min, error_min);
        } else {
            _error_min = error_min;
        }
    }

    if (is_positive(error_max)) {
        if (!is_zero(_error_max)) {
            _error_max = MIN(_error_max, error_max);
        } else {
            _error_max = error_max;
        }
    }
}
