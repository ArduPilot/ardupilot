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
    _lim_D_Out = 10.0f;     // maximum first differential of output
}

// update_all - set target and measured inputs to P controller and calculate outputs
// target and measurement are filtered
// if measurement is further than error_min or error_max (see set_limits_error method)
//   the target is moved closer to the measurement and limit_min or limit_max will be set true
float AC_P_1D::update_all(float &target, float measurement, bool &limit_min, bool &limit_max)
{
    limit_min = false;
    limit_max = false;

    // calculate distance _error
    float error = target - measurement;

    if (error < _lim_err_min) {
        error = _lim_err_min;
        target = measurement + error;
        limit_min = true;
    } else if (error > _lim_err_max) {
        error = _lim_err_max;
        target = measurement + error;
        limit_max = true;
    }

    // ToDo: Replace sqrt_controller with optimal acceleration and jerk limited curve
    // MIN(_Dxy_max, _D2xy_max / _kxy_P) limits the max accel to the point where max jerk is exceeded
    return sqrt_controller(error, _kp, _lim_D_Out, _dt);
}

// set limits on error, output and output from D term
// in normal use the lim_err_min and lim_out_min will be negative
// when using for a position controller, lim_err will be position error, lim_out will be correction velocity, lim_D will be acceleration, lim_D2 will be jerk
void AC_P_1D::set_limits_error(float lim_err_min, float lim_err_max, float lim_out_min, float lim_out_max, float lim_D_Out, float lim_D2_Out)
{
    _lim_D_Out = lim_D_Out;
    if (is_positive(lim_D2_Out)) {
        // limit the first derivative so as not to exceed the second derivative
        _lim_D_Out = MIN(_lim_D_Out, lim_D2_Out / _kp);
    }
    _lim_err_min = MAX(inv_sqrt_controller(lim_out_min, _kp, _lim_D_Out), lim_err_min);
    _lim_err_max = MAX(inv_sqrt_controller(lim_out_max, _kp, _lim_D_Out), lim_err_max);
}
