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
    _error = 0.0f;
    _lim_err_neg = 0.0f;     // error value to enable filtering
    _lim_err_pos = 0.0f;     // error value to enable filtering
    _lim_out_neg = 0.0f;     // error value to enable filtering
    _lim_out_pos = 0.0f;     // error value to enable filtering
    _lim_D_Out = 10.0f;     // maximum first differential of output
}

// set_dt - set time step in seconds
void AC_P_1D::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
}

// set_dt - set time step in seconds
void AC_P_1D::set_limits_error(float lim_err_neg, float lim_err_pos, float lim_out_neg, float lim_out_pos, float lim_D_Out, float lim_D2_Out)
{
    _lim_err_neg = -inv_sqrt_controller(lim_err_neg, _kp, _lim_D_Out);
    _lim_err_pos = inv_sqrt_controller(lim_err_pos, _kp, _lim_D_Out);
    _lim_out_neg = lim_out_neg;     // error value to enable filtering
    _lim_out_pos = lim_out_pos;     // error value to enable filtering
    _lim_D_Out = lim_D_Out;
    if(is_positive(lim_D2_Out)) {
        // limit the first derivative so as not to exceed the second derivative
        _lim_D_Out = MIN(_lim_D_Out, lim_D2_Out / _kp);
    }
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and _error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
float AC_P_1D::update_all(float &target, float measurement, bool limit_min, bool limit_max)
{
    limit_min = false;
    limit_max = false;

    // calculate distance _error
    float error = target - measurement;

    if (asymetricLimit(error, _lim_err_neg, _lim_err_pos, limit_min, limit_max )) {
        target = measurement + error;
    }

//    todo: Replace sqrt_controller with optimal acceleration and jerk limited curve
    // MIN(_Dxy_max, _D2xy_max / _kxy_P) limits the max accel to the point where max jerk is exceeded
    return sqrt_controller(error, _kp, _lim_D_Out, _dt);
}

float AC_P_1D::get_p() const
{
    return _error * _kp;
}

void AC_P_1D::load_gains()
{
    _kp.load();
}

void AC_P_1D::save_gains()
{
    _kp.save();
}

/// Overload the function call operator to permit easy initialisation
void AC_P_1D::operator()(float initial_p, float dt)
{
    _kp = initial_p;
    _dt = dt;
}

/// asymetricLimit - set limits based on seperate max and min
bool AC_P_1D::asymetricLimit(float &input, float min, float max, bool &limitMin, bool &limitMax )
{
    if (input < min) {
        input = min;
        limitMin = true;
    }
    if (input > max) {
        input = max;
        limitMax = true;
    }

    return limitMin || limitMax;
}

// Proportional controller with piecewise sqrt sections to constrain second derivative
float AC_P_1D::sqrt_controller(float error, float p, float D_max, float dt)
{
    float output;
    if (!is_positive(D_max)) {
        // second order limit is zero or negative.
        output = error * p;
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        if (is_positive(error)) {
            output = safe_sqrt(2.0f * D_max * (error));
        } else if (is_negative(error)) {
            output = -safe_sqrt(2.0f * D_max * (-error));
        } else {
            output = 0.0f;
        }
    } else {
        // Both the P and second order limit have been defined.
        float linear_dist = D_max / sq(p);
        if (error > linear_dist) {
            output = safe_sqrt(2.0f * D_max * (error - (linear_dist / 2.0f)));
        } else if (error < -linear_dist) {
            output = -safe_sqrt(2.0f * D_max * (-error - (linear_dist / 2.0f)));
        } else {
            output = error * p;
        }
    }
    if (!is_zero(dt)) {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        return constrain_float(output, -fabsf(error) / dt, fabsf(error) / dt);
    } else {
        return output;
    }
}

// Proportional controller with piecewise sqrt sections to constrain second derivative
float AC_P_1D::inv_sqrt_controller(float output, float p, float D_max)
{
    output = fabsf(output);

    float error;
    if (!is_positive(D_max) && is_zero(p)) {
        error = 0.0f;
    } else if (!is_positive(D_max)) {
        // second order limit is zero or negative.
        error = output / p;
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        error = sq(output)/(2*D_max);
    } else {
        // Both the P and second order limit have been defined.
        float linear_out = D_max / p;
        if (output > linear_out) {
            error = sq(output)/(2*D_max) + D_max/(4*sq(p));
        } else {
            error = output / p;
        }
    }
    return error;
}

