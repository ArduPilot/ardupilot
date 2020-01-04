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
}

//  set target and measured inputs to P controller and calculate outputs
Vector2f AC_P_2D::update_all(float &target_x, float &target_y, const Vector2f &measurement, float error_max, float D2_max)
{
    // calculate distance _error
    Vector2f error = Vector2f(target_x, target_y) - measurement;

    // Constrain _error and target position
    // Constrain the maximum length of _vel_target to the maximum position correction velocity
    if (error.limit_length(error_max)) {
        target_x = measurement.x + error.x;
        target_y = measurement.y + error.y;
    }

    // MIN(_Dmax, _D2max / _kp) limits the max accel to the point where max jerk is exceeded
    // return sqrt_controller(Vector2f(_error.x, _error.y), _kp, MIN(_D_max, _D2_max / _kp), _dt);
    return sqrt_controller(error, _kp, D2_max, _dt);
}
