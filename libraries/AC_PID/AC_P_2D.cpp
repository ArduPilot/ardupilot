/// @file   AC_P_2D.cpp
/// @brief  2D P controller with output limiting, derivative constraint, and EEPROM-backed gain storage.

#include <AP_Math/AP_Math.h>
#include "AC_P_2D.h"

const AP_Param::GroupInfo AC_P_2D::var_info[] = {
    // @Param: P
    // @DisplayName: Proportional Gain
    // @Description: Proportional gain that generates output based on the current error value
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

// Computes the P controller output given a target and measurement.
// Applies position error clamping based on configured limits.
// Optionally constrains output slope using the sqrt_controller.
Vector2f AC_P_2D::update_all(Vector2p &target, const Vector2p &measurement)
{
    // Compute vector error between target and measurement (NED frame)
    _error = (target - measurement).tofloat();

    // Limit error vector length to prevent exceeding output constraints
    if (is_positive(_error_max) && _error.limit_length(_error_max)) {
        target = measurement + _error.topostype();
    }

    // Use sqrt_controller to limit output and/or its derivative
    return sqrt_controller(_error, _kp, _D1_max, 0.0);
}

// Sets limits on output, output slope (D1), and output acceleration (D2).
// For position controllers: output = velocity, D1 = acceleration, D2 = jerk.
void AC_P_2D::set_limits(float output_max, float D_Out_max, float D2_Out_max)
{
    // Reset all limits to zero before applying new ones
    _D1_max = 0.0f;
    _error_max = 0.0f;

    // Set first derivative (acceleration) limit if specified
    if (is_positive(D_Out_max)) {
        _D1_max = D_Out_max;
    }

    // If second derivative (jerk) limit is specified, constrain velocity limit accordingly
    if (is_positive(D2_Out_max) && is_positive(_kp)) {
        // limit the first derivative so as not to exceed the second derivative
        _D1_max = MIN(_D1_max, D2_Out_max / _kp);
    }

    // Compute min/max allowable error from output limits
    if (is_positive(output_max) && is_positive(_kp)) {
        _error_max = inv_sqrt_controller(output_max, _kp, _D1_max);
    }
}

// Reduces the maximum allowable error after limit initialization.
// Intended to be called after set_limits().
void AC_P_2D::set_error_max(float error_max)
{
    // Update _error_min if it's non-zero and the new value is more conservative
    if (is_positive(error_max)) {
        if (!is_zero(_error_max) ) {
            _error_max = MIN(_error_max, error_max);
        } else {
            _error_max = error_max;
        }
    }
}
