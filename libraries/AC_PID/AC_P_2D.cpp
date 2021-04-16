/// @file   AC_P_2D.cpp
/// @brief  2-axis P controller

#include <AP_Math/AP_Math.h>
#include "AC_P_2D.h"

const AP_Param::GroupInfo AC_P_2D::var_info[] = {
    // @Param: P
    // @DisplayName: Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_P_2D, _kp, 0),
    AP_GROUPEND
};

// Constructor
AC_P_2D::AC_P_2D(float initial_p, float dt) :
    _dt(dt)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp = initial_p;
    _error_max = 0.0f;
    _D1_max = 0.0f;
}

//  set target and measured inputs to P controller and calculate outputs
Vector2f AC_P_2D::update_all(float &target_x, float &target_y, const Vector2f &measurement, bool &limit)
{
    limit = false;

    // calculate distance _error
    _error = Vector2f(target_x, target_y) - measurement;

    // Constrain _error and target position
    // Constrain the maximum length of _vel_target to the maximum position correction velocity
    if (is_positive(_error_max) && _error.limit_length(_error_max)) {
        target_x = measurement.x + _error.x;
        target_y = measurement.y + _error.y;
        limit = true;
    }

    // MIN(_Dmax, _D2max / _kp) limits the max accel to the point where max jerk is exceeded
    // return sqrt_controller(Vector2f(_error.x, _error.y), _kp, MIN(_D_max, _D2_max / _kp), _dt);
    return sqrt_controller(_error, _kp, _D1_max, _dt);
}

// set limits on error, output and output from D term
// in normal use the error_min and output_min will be negative
// when using for a position controller, lim_err will be position error, lim_out will be correction velocity, lim_D will be acceleration, lim_D2 will be jerk
void AC_P_2D::set_limits(float error_max, float output_max, float lim_D_Out, float D2_max)
{
    _D1_max = lim_D_Out;
    if (is_positive(D2_max)) {
        // limit the first derivative so as not to exceed the second derivative
        _D1_max = MIN(_D1_max, D2_max / _kp);
    }

    _error_max = inv_sqrt_controller(output_max, _kp, _D1_max);
    if(!is_zero(_error_max) && is_positive(error_max) ) {
        _error_max = MIN(_error_max, error_max);
    } else {
        _error_max = error_max;
    }
    _error_max = MAX(0.0f, _error_max);
}
