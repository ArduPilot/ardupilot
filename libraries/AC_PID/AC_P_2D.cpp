/// @file   AC_P_2D.cpp
/// @brief  2-axis P controller

#include <AP_Math/AP_Math.h>
#include "AC_P_2D.h"

const AP_Param::GroupInfo AC_P_2D::var_info[] = {
    // @Param: P
    // @DisplayName: Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    0, AC_P_2D, _kp, default_kp),
    AP_GROUPEND
};

// Constructor
AC_P_2D::AC_P_2D(float initial_p) :
    default_kp(initial_p)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);
}

// update_all - set target and measured inputs to P controller and calculate outputs
Vector2f AC_P_2D::update_all(postype_t &target_x, postype_t &target_y, const Vector2f &measurement)
{
    // calculate distance _error
    _error = (Vector2p{target_x, target_y} - measurement.topostype()).tofloat();

    // Constrain _error and target position
    // Constrain the maximum length of _vel_target to the maximum position correction velocity
    if (is_positive(_error_max) && _error.limit_length(_error_max)) {
        target_x = measurement.x + _error.x;
        target_y = measurement.y + _error.y;
    }

    // MIN(_Dmax, _D2max / _kp) limits the max accel to the point where max jerk is exceeded
    return sqrt_controller(_error, _kp, _D1_max, 0.0);
}

// set_limits - sets the maximum error to limit output and first and second derivative of output
// when using for a position controller, lim_err will be position error, lim_out will be correction velocity, lim_D will be acceleration, lim_D2 will be jerk
void AC_P_2D::set_limits(float output_max, float D_Out_max, float D2_Out_max)
{
    _D1_max = 0.0f;
    _error_max = 0.0f;

    if (is_positive(D_Out_max)) {
        _D1_max = D_Out_max;
    }

    if (is_positive(D2_Out_max) && is_positive(_kp)) {
        // limit the first derivative so as not to exceed the second derivative
        _D1_max = MIN(_D1_max, D2_Out_max / _kp);
    }

    if (is_positive(output_max) && is_positive(_kp)) {
        _error_max = inv_sqrt_controller(output_max, _kp, _D1_max);
    }
}

// set_error_max - reduce maximum position error to error_max
// to be called after setting limits
void AC_P_2D::set_error_max(float error_max)
{
    if (is_positive(error_max)) {
        if (!is_zero(_error_max) ) {
            _error_max = MIN(_error_max, error_max);
        } else {
            _error_max = error_max;
        }
    }
}
