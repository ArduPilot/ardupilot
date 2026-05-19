/// @file	AC_PID_Basic.cpp
/// @brief	Lightweight PID controller with error and derivative filtering, integrator limit, and EEPROM gain storage.

#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include "AC_PID_Basic.h"

#define AC_PID_Basic_FILT_E_HZ_MIN     0.01f   // minimum input filter frequency
#define AC_PID_Basic_FILT_D_HZ_MIN     0.005f  // minimum input filter frequency

const AP_Param::GroupInfo AC_PID_Basic::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    0, AC_PID_Basic, _kp, default_kp),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I",    1, AC_PID_Basic, _ki, default_ki),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("IMAX", 2, AC_PID_Basic, _kimax, default_kimax),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the error (Hz)
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTE", 3, AC_PID_Basic, _filt_E_hz, default_filt_E_hz),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D",    4, AC_PID_Basic, _kd, default_kd),

    // @Param: FLTD
    // @DisplayName: D term filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the derivative (Hz)
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD", 5, AC_PID_Basic, _filt_D_hz, default_filt_D_hz),

    // @Param: FF
    // @DisplayName: PID Feed Forward Gain
    // @Description: FF Gain which produces an output that is proportional to the magnitude of the target
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FF",   6, AC_PID_Basic, _kff, default_kff),

    AP_GROUPEND
};

// Constructor
AC_PID_Basic::AC_PID_Basic(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_E_hz, float initial_filt_D_hz) :
    default_kp(initial_p),
    default_ki(initial_i),
    default_kd(initial_d),
    default_kff(initial_ff),
    default_kimax(initial_imax),
    default_filt_E_hz(initial_filt_E_hz),
    default_filt_D_hz(initial_filt_D_hz)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    // reset input filter to first value received
    _reset_filter = true;
}

float AC_PID_Basic::update_all(float target, float measurement, float dt, bool limit)
{
    return update_all(target, measurement, dt, (limit && is_negative(_integrator)), (limit && is_positive(_integrator)));
}

// Computes the PID output using a target and measurement input.
// Applies filters to the error and derivative, then updates the integrator.
// If `limit` is true, the integrator is allowed to shrink but not grow.
float AC_PID_Basic::update_all(float target, float measurement, float dt, bool limit_neg, bool limit_pos)
{
    // Return zero if inputs are invalid (NaN or infinite)
    if (!isfinite(target) || isnan(target) ||
        !isfinite(measurement) || isnan(measurement)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return 0.0f;
    }

    _target = target;

    // Reset filter state to match current inputs (on first run or after reset)
    if (_reset_filter) {
        // Reset filters to match the current inputs
        _reset_filter = false;
        _error = _target - measurement;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _error += get_filt_E_alpha(dt) * ((_target - measurement) - _error);

        // Compute and low-pass filter the error derivative (D term)
        if (is_positive(dt)) {
            float derivative = (_error - error_last) / dt;
            _derivative += get_filt_D_alpha(dt) * (derivative - _derivative);
        }
    }

    // update I term
    update_i(dt, limit_neg, limit_pos);

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

// Updates the integrator using current error and dt.
// If `limit_neg` is true, integrator may only increase.
// If `limit_pos` is true, integrator may only decrease.
void AC_PID_Basic::update_i(float dt, bool limit_neg, bool limit_pos)
{
    if (!is_zero(_ki)) {
        // Ensure that integrator can only be reduced if the output is saturated
        if (!((limit_neg && is_negative(_error)) || (limit_pos && is_positive(_error)))) {
            _integrator += ((float)_error * _ki) * dt;
            _integrator = constrain_float(_integrator, -_kimax, _kimax);
        }
    } else {
        _integrator = 0.0f;
    }
}

void AC_PID_Basic::reset_I()
{
    _integrator = 0.0; 
}

// Saves controller configuration from EEPROM, including gains and filter frequencies. (not used)
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

// Returns alpha value for the error low-pass filter (based on filter frequency and dt)
float AC_PID_Basic::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_E_hz);
}

// Returns alpha value for the derivative low-pass filter (based on filter frequency and dt)
float AC_PID_Basic::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D_hz);
}

// Sets integrator based on target, measurement, and desired total PID output.
void AC_PID_Basic::set_integrator(float target, float measurement, float i)
{
    set_integrator(target - measurement, i);
}

// Sets integrator using error and desired total output.
void AC_PID_Basic::set_integrator(float error, float i)
{
    set_integrator(i - error * _kp);
}

// Sets the integrator directly.
void AC_PID_Basic::set_integrator(float i)
{
    _integrator = constrain_float(i, -_kimax, _kimax);
}
