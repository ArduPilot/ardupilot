/// @file   AC_P_2D.cpp
/// @brief  Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_P_2D.h"

const AP_Param::GroupInfo AC_P_2D::var_info[] = {
    // @Param: P
    // @DisplayName: PI Proportional Gain
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

    _D_max = 0.0f;
    _D2_max = 0.0f;
    _kp = initial_p;
}

// set_dt - set time step in seconds
void AC_P_2D::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and _error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
Vector2f AC_P_2D::update_all(float &target_x, float &target_y, Vector2f measurement, float max)
{
    // calculate distance _error
    _error = Vector2f(target_x, target_y) - measurement;

    // Constrain _error and target position
    // Constrain the maximum length of _vel_target to the maximum position correction velocity
    // TODO: replace the leash length with a user definable maximum position correction
    if (limit_vector_length(_error.x, _error.y, max)) {
        target_x = measurement.x + _error.x;
        target_y = measurement.y + _error.y;
    }

//    todo: Replace sqrt_controller with optimal acceleration and jerk limited curve
    // MIN(_Dmax, _D2max / _kp) limits the max accel to the point where max jerk is exceeded
    return sqrt_controller(Vector2f(_error.x, _error.y), _kp, MIN(_D_max, _D2_max / _kp), _dt);;
}

Vector2f AC_P_2D::get_p() const
{
    return _error * _kp;
}

/// Overload the function call operator to permit easy initialisation
void AC_P_2D::operator()(float initial_p, float dt)
{
    _kp = initial_p;
    _dt = dt;
}

/// limit vector to a given length, returns true if vector was limited
bool AC_P_2D::limit_vector_length(float& vector_x, float& vector_y, float max_length)
{
    float vector_length = norm(vector_x, vector_y);
    if ((vector_length > max_length) && is_positive(vector_length)) {
        vector_x *= (max_length / vector_length);
        vector_y *= (max_length / vector_length);
        return true;
    }
    return false;
}

/// Proportional controller with piecewise sqrt sections to constrain second derivative
Vector2f AC_P_2D::sqrt_controller(const Vector2f& _error, float p, float second_ord_lim, float dt)
{
    Vector2f correction;
    if (second_ord_lim < 0.0f || is_zero(second_ord_lim) || is_zero(p)) {
        correction = _error * p;
    }

    float _error_length = _error.length();
    if (!is_positive(second_ord_lim)) {
        // second order limit is zero or negative.
        correction =  _error * p;
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        float first_order_scale = safe_sqrt(2.0f * second_ord_lim * _error_length) / _error_length;
        correction =  _error * first_order_scale;
    } else {
        float linear_dist = second_ord_lim / sq(p);
        if (_error_length > linear_dist) {
            float first_order_scale = safe_sqrt(2.0f * second_ord_lim * (_error_length - (linear_dist * 0.5f))) / _error_length;
            correction =  _error * first_order_scale;
        } else {
            correction =  _error * p;
        }
    }
    if (!is_zero(dt)) {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        limit_vector_length(correction.x, correction.y, _error_length / dt);
    }
    return correction;
}
