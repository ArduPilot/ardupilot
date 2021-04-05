/// @file	AC_PID_Basic.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include "AC_PID_Basic.h"

#define AC_PID_Basic_FILT_E_HZ_DEFAULT 20.0f   // default input filter frequency
#define AC_PID_Basic_FILT_E_HZ_MIN     0.01f   // minimum input filter frequency
#define AC_PID_Basic_FILT_D_HZ_DEFAULT 10.0f   // default input filter frequency
#define AC_PID_Basic_FILT_D_HZ_MIN     0.005f  // minimum input filter frequency

const AP_Param::GroupInfo AC_PID_Basic::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_PID_Basic, _kp, 0),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I",    1, AC_PID_Basic, _ki, 0),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 2, AC_PID_Basic, _kimax, 0),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTE", 3, AC_PID_Basic, _filt_E_hz, AC_PID_Basic_FILT_E_HZ_DEFAULT),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("D",    4, AC_PID_Basic, _kd, 0),

    // @Param: FLTD
    // @DisplayName: D term filter frequency in Hz
    // @Description: D term filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FLTD", 5, AC_PID_Basic, _filt_D_hz, AC_PID_Basic_FILT_D_HZ_DEFAULT),

    // @Param: FF
    // @DisplayName: PID Feed Forward Gain
    // @Description: FF Gain which produces an output that is proportional to the magnitude of the target
    AP_GROUPINFO("FF",   6, AC_PID_Basic, _kff, 0),

    AP_GROUPEND
};

// Constructor
AC_PID_Basic::AC_PID_Basic(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_E_hz, float initial_filt_D_hz, float dt) :
    _dt(dt)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp = initial_p;
    _ki = initial_i;
    _kd = initial_d;
    _kff = initial_ff;
    _kimax = fabsf(initial_imax);
    filt_E_hz(initial_filt_E_hz);
    filt_D_hz(initial_filt_D_hz);

    // reset input filter to first value received
    _reset_filter = true;
}

float AC_PID_Basic::update_all(float target, float measurement, bool limit)
{
    return update_all(target, measurement, (limit && is_negative(_integrator)), (limit && is_positive(_integrator)));
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
float AC_PID_Basic::update_all(float target, float measurement, bool limit_neg, bool limit_pos)
{
    // don't process inf or NaN
    if (!isfinite(target) || isnan(target) ||
        !isfinite(measurement) || isnan(measurement)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return 0.0f;
    }

    _target = target;

    // reset input filter to value received
    if (_reset_filter) {
        _reset_filter = false;
        _error = _target - measurement;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _error += get_filt_E_alpha() * ((_target - measurement) - _error);

        // calculate and filter derivative
        if (is_positive(_dt)) {
            float derivative = (_error - error_last) / _dt;
            _derivative += get_filt_D_alpha() * (derivative - _derivative);
        }
    }

    // update I term
    update_i(limit_neg, limit_pos);

    const float P_out = _error * _kp;
    const float D_out = _derivative * _kd;

    _pid_info.target = _target;
    _pid_info.actual = measurement;
    _pid_info.error = _error;
    _pid_info.P = _error * _kp;
    _pid_info.I = _integrator;
    _pid_info.D = _derivative * _kd;
    _pid_info.FF = _target * _kff;

    return P_out + _integrator + D_out + _target * _kff;
}

//  update_i - update the integral
//  if limit_neg is true, the integral can only increase
//  if limit_pos is true, the integral can only decrease
void AC_PID_Basic::update_i(bool limit_neg, bool limit_pos)
{
    if (!is_zero(_ki)) {
        // Ensure that integrator can only be reduced if the output is saturated
        if (!((limit_neg && is_negative(_error)) || (limit_pos && is_positive(_error)))) {
            _integrator += ((float)_error * _ki) * _dt;
            _integrator = constrain_float(_integrator, -_kimax, _kimax);
        }
    } else {
        _integrator = 0.0f;
    }
}

// save_gains - save gains to eeprom
void AC_PID_Basic::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _kff.save();
    _kimax.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
}

// get_filt_T_alpha - get the target filter alpha
float AC_PID_Basic::get_filt_E_alpha() const
{
    return calc_lowpass_alpha_dt(_dt, _filt_E_hz);
}

// get_filt_D_alpha - get the derivative filter alpha
float AC_PID_Basic::get_filt_D_alpha() const
{
    return calc_lowpass_alpha_dt(_dt, _filt_D_hz);
}

void AC_PID_Basic::set_integrator(float target, float measurement, float i)
{
    set_integrator(target - measurement, i);
}

void AC_PID_Basic::set_integrator(float error, float i)
{
    set_integrator(i - error * _kp);
}

void AC_PID_Basic::set_integrator(float i)
{
    _integrator = constrain_float(i, -_kimax, _kimax);
}
