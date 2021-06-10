/// @file	AC_PID_2D.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PID_2D.h"

#define AC_PID_2D_FILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_PID_2D_FILT_HZ_MIN      0.01f   // minimum input filter frequency
#define AC_PID_2D_FILT_D_HZ_DEFAULT  10.0f   // default input filter frequency
#define AC_PID_2D_FILT_D_HZ_MIN      0.005f   // minimum input filter frequency

const AP_Param::GroupInfo AC_PID_2D::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_PID_2D, _kp, 0),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I",    1, AC_PID_2D, _ki, 0),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 2, AC_PID_2D, _kimax, 0),

    // @Param: FILT
    // @DisplayName: PID Input filter frequency in Hz
    // @Description: Input filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FILT", 3, AC_PID_2D, _filt_E_hz, AC_PID_2D_FILT_HZ_DEFAULT),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("D",    4, AC_PID_2D, _kd, 0),

    // @Param: D_FILT
    // @DisplayName: D term filter frequency in Hz
    // @Description: D term filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("D_FILT", 5, AC_PID_2D, _filt_D_hz, AC_PID_2D_FILT_D_HZ_DEFAULT),

    // @Param: FF
    // @DisplayName: PID Feed Forward Gain
    // @Description: FF Gain which produces an output that is proportional to the magnitude of the target
    AP_GROUPINFO("FF",    6, AC_PID_2D, _kff, 0),

    AP_GROUPEND
};

// Constructor
AC_PID_2D::AC_PID_2D(float initial_kP, float initial_kI, float initial_kD, float initial_kFF, float initial_imax, float initial_filt_E_hz, float initial_filt_D_hz, float dt) :
    _dt(dt)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp = initial_kP;
    _ki = initial_kI;
    _kd = initial_kD;
    _kff = initial_kFF;
    _kimax = fabsf(initial_imax);
    filt_E_hz(initial_filt_E_hz);
    filt_D_hz(initial_filt_D_hz);

    // reset input filter to first value received
    _reset_filter = true;
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated if it does not increase in the direction of the limit vector
Vector2f AC_PID_2D::update_all(const Vector2f &target, const Vector2f &measurement, const Vector2f &limit)
{
    // don't process inf or NaN
    if (target.is_nan() || target.is_inf() ||
        measurement.is_nan() || measurement.is_inf()) {
        return Vector2f{};
    }

    _target = target;

    // reset input filter to value received
    if (_reset_filter) {
        _reset_filter = false;
        _error = _target - measurement;
        _derivative.zero();
    } else {
        Vector2f error_last{_error};
        _error += ((_target - measurement) - _error) * get_filt_E_alpha();

        // calculate and filter derivative
        if (_dt > 0.0f) {
            const Vector2f derivative{(_error - error_last) / _dt};
            _derivative += (derivative - _derivative) * get_filt_D_alpha();
        }
    }

    // update I term
    update_i(limit);

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

    return _error * _kp + _integrator + _derivative * _kd + _target * _kff;
}

Vector2f AC_PID_2D::update_all(const Vector3f &target, const Vector3f &measurement, const Vector3f &limit)
{
    return update_all(Vector2f{target.x, target.y}, Vector2f{measurement.x, measurement.y}, Vector2f{limit.x, limit.y});
}

//  update_i - update the integral
//  If the limit is set the integral is only allowed to reduce in the direction of the limit
void AC_PID_2D::update_i(const Vector2f &limit)
{
    Vector2f limit_direction = limit;
    Vector2f delta_integrator = (_error * _ki) * _dt;
    if (!is_zero(limit_direction.length_squared())) {
        // zero delta_vel if it will increase the velocity error
        limit_direction.normalize();
        if (is_positive(delta_integrator * limit)) {
            delta_integrator.zero();
        }
    }

    _integrator += delta_integrator;
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

Vector2f AC_PID_2D::get_ff()
{
    _pid_info_x.FF = _target.x * _kff;
    _pid_info_y.FF = _target.y * _kff;
    return _target * _kff;
}

// save_gains - save gains to eeprom
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

// get the target filter alpha
float AC_PID_2D::get_filt_E_alpha() const
{
    return calc_lowpass_alpha_dt(_dt, _filt_E_hz);
}

// get the derivative filter alpha
float AC_PID_2D::get_filt_D_alpha() const
{
    return calc_lowpass_alpha_dt(_dt, _filt_D_hz);
}

void AC_PID_2D::set_integrator(const Vector2f& target, const Vector2f& measurement, const Vector2f& i)
{
    set_integrator(target - measurement, i);
}

void AC_PID_2D::set_integrator(const Vector2f& error, const Vector2f& i)
{
    set_integrator(i - error * _kp);
}

void AC_PID_2D::set_integrator(const Vector2f& i)
{
    _integrator = i;
    const float integrator_length = _integrator.length();
    if (integrator_length > _kimax) {
        _integrator *= (_kimax / integrator_length);
    }
}

