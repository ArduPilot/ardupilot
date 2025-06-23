/// @file	AC_PID_2D.cpp
/// @brief	2D PID controller with vector support, input filtering, integrator clamping, and EEPROM-backed gain storage.

#include <AP_Math/AP_Math.h>
#include "AC_PID_2D.h"

#define AC_PID_2D_FILT_D_HZ_MIN      0.005f   // minimum input filter frequency

const AP_Param::GroupInfo AC_PID_2D::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    0, AC_PID_2D, _kp, default_kp),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I",    1, AC_PID_2D, _ki, default_ki),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("IMAX", 2, AC_PID_2D, _kimax, default_kimax),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the error (Hz)
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTE", 3, AC_PID_2D, _filt_E_hz, default_filt_E_hz),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D",    4, AC_PID_2D, _kd, default_kd),

    // @Param: FLTD
    // @DisplayName: D term filter frequency in Hz
    // @Description: Low-pass filter frequency applied to the derivative (Hz)
    // @Units: Hzs
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD", 5, AC_PID_2D, _filt_D_hz, default_filt_D_hz),

    // @Param: FF
    // @DisplayName: PID Feed Forward Gain
    // @Description: FF Gain which produces an output that is proportional to the magnitude of the target
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FF",    6, AC_PID_2D, _kff, default_kff),

    AP_GROUPEND
};

// Constructor
AC_PID_2D::AC_PID_2D(float initial_kP, float initial_kI, float initial_kD, float initial_kFF, float initial_imax, float initial_filt_E_hz, float initial_filt_D_hz) :
    default_kp(initial_kP),
    default_ki(initial_kI),
    default_kd(initial_kD),
    default_kff(initial_kFF),
    default_kimax(initial_imax),
    default_filt_E_hz(initial_filt_E_hz),
    default_filt_D_hz(initial_filt_D_hz)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    // reset input filter to first value received
    _reset_filter = true;
}

// Computes the 2D PID output from target and measurement vectors.
// Applies filtering to error and derivative terms.
// Integrator is updated only if it does not grow in the direction of the specified limit vector.
Vector2f AC_PID_2D::update_all(const Vector2f &target, const Vector2f &measurement, float dt, const Vector2f &limit)
{
    // Return zero if any component is NaN or infinite
    if (target.is_nan() || target.is_inf() ||
        measurement.is_nan() || measurement.is_inf()) {
        return Vector2f{};
    }

    _target = target;

    // reset input filter to value received
    if (_reset_filter) {
        // Reset filters to match the current inputs
        _reset_filter = false;
        _error = _target - measurement;
        // Reset the derivative to avoid transients
        _derivative.zero();
    } else {
        Vector2f error_last{_error};
        // Apply first-order low-pass filter to error
        _error += ((_target - measurement) - _error) * get_filt_E_alpha(dt);

        // Compute and low-pass filter the derivative
        if (is_positive(dt)) {
            const Vector2f derivative{(_error - error_last) / dt};
            _derivative += (derivative - _derivative) * get_filt_D_alpha(dt);
        }
    }

    // update I term
    update_i(dt, limit);
 
    _pid_info_x.target = _target.x;
    _pid_info_x.actual = measurement.x;
    _pid_info_x.error = _error.x;
    _pid_info_x.P = _error.x * _kp;
    _pid_info_x.I = _integrator.x;
    _pid_info_x.D = _derivative.x * _kd;
    _pid_info_x.FF = _target.x * _kff;

    _pid_info_y.target = _target.y;
    _pid_info_y.actual = measurement.y;
    _pid_info_y.error = _error.y;
    _pid_info_y.P = _error.y * _kp;
    _pid_info_y.I = _integrator.y;
    _pid_info_y.D = _derivative.y * _kd;
    _pid_info_y.FF = _target.y * _kff;

    // Return total control output: P + I + D + FF terms
    return _error * _kp + _integrator + _derivative * _kd + _target * _kff;
}

Vector2f AC_PID_2D::update_all(const Vector3f &target, const Vector3f &measurement, float dt, const Vector3f &limit)
{
    return update_all(Vector2f{target.x, target.y}, Vector2f{measurement.x, measurement.y}, dt, Vector2f{limit.x, limit.y});
}

// Updates the 2D integrator using the filtered error.
// The integrator is only allowed to grow if it does not push further in the direction of the limit vector.
void AC_PID_2D::update_i(float dt, const Vector2f &limit)
{
    _pid_info_x.limit = false;
    _pid_info_y.limit = false;

    Vector2f delta_integrator = (_error * _ki) * dt;
    float integrator_length = _integrator.length();
    _integrator += delta_integrator;
    // Compute integrator delta and apply anti-windup by limiting growth in the direction of the limit vector
    if (is_positive(delta_integrator * limit) && _integrator.limit_length(integrator_length)) {
        _pid_info_x.limit = true;
        _pid_info_y.limit = true;
    }

    // Clamp integrator to maximum length (IMAX)
    _integrator.limit_length(_kimax);
}

Vector2f AC_PID_2D::get_p() const
{
    return _error * _kp;
}

const Vector2f& AC_PID_2D::get_i() const
{
    return _integrator;
}

Vector2f AC_PID_2D::get_d() const
{
    return _derivative * _kd;
}

// Update FF terms in PID logs and return feedforward vector
Vector2f AC_PID_2D::get_ff()
{
    _pid_info_x.FF = _target.x * _kff;
    _pid_info_y.FF = _target.y * _kff;
    return _target * _kff;
}

void AC_PID_2D::reset_I()
{
    _integrator.zero(); 
}

// Saves controller configuration from EEPROM, including gains and filter frequencies. (not used)
void AC_PID_2D::save_gains()
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
float AC_PID_2D::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_E_hz);
}

// Returns alpha value for the derivative low-pass filter (based on filter frequency and dt)
float AC_PID_2D::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D_hz);
}

// Compute error from target and measurement, then set integrator
void AC_PID_2D::set_integrator(const Vector2f& target, const Vector2f& measurement, const Vector2f& i)
{
    set_integrator(target - measurement, i);
}

// Convert from desired total output and error to integrator value
void AC_PID_2D::set_integrator(const Vector2f& error, const Vector2f& i)
{
    set_integrator(i - error * _kp);
}

// Set integrator directly and clamp to IMAX
void AC_PID_2D::set_integrator(const Vector2f& i)
{
    _integrator = i;
    _integrator.limit_length(_kimax);
}

