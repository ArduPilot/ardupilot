/// @file	AC_PID_2D.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PID_2D.h"

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
AC_PID_2D::AC_PID_2D(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_E_hz, float initial_filt_D_hz, float dt) :
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
    _flags._reset_filter = true;

    memset(&_pid_info_x, 0, sizeof(_pid_info_x));
//    memset(&_pid_info_y, 0, sizeof(_pid_info_y));
}

// set_dt - set time step in seconds
void AC_PID_2D::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
}

// filt_E_hz - set error filter hz
void AC_PID_2D::filt_E_hz(float hz)
{
    _filt_E_hz.set(fabsf(hz));
}

// filt_D_hz - set derivative filter hz
void AC_PID_2D::filt_D_hz(float hz)
{
    _filt_D_hz.set(fabsf(hz));
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
Vector2f AC_PID_2D::update_all(float target_x, float target_y, Vector2f measurement, bool limit)
{
    _target = Vector2f(target_x, target_y);
    // don't process inf or NaN
    if (!isfinite(_target.x) || !isfinite(_target.y) || !isfinite(measurement.x) || !isfinite(measurement.y)) {
        return Vector2f(0.0f,0.0f);
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _error = _target - measurement;
        _derivative = Vector2f(0.0f,0.0f);
    } else {
        Vector2f error_last = _error;
        _error += ((_target - measurement) - _error) * get_filt_E_alpha();

        // calculate and filter derivative
        if (_dt > 0.0f) {
            Vector2f derivative = (_error - error_last) / _dt;
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

//  update_i - update the integral
//  If the limit flag is set the integral is only allowed to shrink
void AC_PID_2D::update_i(bool limit)
{
    float integrator_length_orig = _kimax;
    if (limit) {
        integrator_length_orig = MIN(integrator_length_orig, _integrator.length());
    }
    _integrator += (_error * _ki) * _dt;
    const float integrator_length_new = _integrator.length();
    if (integrator_length_new > integrator_length_orig) {
        _integrator *= (integrator_length_orig / integrator_length_new);
    }
}

Vector2f AC_PID_2D::get_p() const
{
    return _error * _kp;
}

Vector2f AC_PID_2D::get_i() const
{
    return _integrator;
}

Vector2f AC_PID_2D::get_d() const
{
    return _derivative * _kd;
}

Vector2f AC_PID_2D::get_ff()
{
    return _target * _kff;
}

void AC_PID_2D::reset_I()
{
    _integrator.zero();
}

void AC_PID_2D::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _kff.load();
    _kimax.load();
    _kimax = fabsf(_kimax);
    _filt_E_hz.load();
    _filt_D_hz.load();
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

/// Overload the function call operator to permit easy initialisation
void AC_PID_2D::operator()(float p_val, float i_val, float d_val, float ff_val, float imax_val, float input_filt_E_hz, float input_filt_D_hz, float dt)
{
    _kp = p_val;
    _ki = i_val;
    _kd = d_val;
    _kff = ff_val;
    _kimax = fabsf(imax_val);
    _filt_E_hz = input_filt_E_hz;
    _filt_D_hz = input_filt_D_hz;
    _dt = dt;
}

// get_filt_T_alpha - get the target filter alpha
float AC_PID_2D::get_filt_E_alpha() const
{
    return get_filt_alpha(_filt_E_hz);
}

// get_filt_D_alpha - get the derivative filter alpha
float AC_PID_2D::get_filt_D_alpha() const
{
    return get_filt_alpha(_filt_D_hz);
}

// get_filt_alpha - calculate a filter alpha
float AC_PID_2D::get_filt_alpha(float filt_hz) const
{
    if (is_zero(filt_hz)) {
        return 1.0f;
    }

    // calculate alpha
    float rc = 1 / (M_2PI * filt_hz);
    return _dt / (_dt + rc);
}

void AC_PID_2D::set_integrator(Vector2f target, Vector2f measurement, Vector2f i)
{
    set_integrator(target - measurement, i);
}

void AC_PID_2D::set_integrator(Vector2f error, Vector2f i)
{
    set_integrator(i - error * _kp);
}

void AC_PID_2D::set_integrator(Vector2f i)
{
    _integrator = i;
    const float integrator_length = _integrator.length();
    if (integrator_length > _kimax) {
        _integrator *= (_kimax / integrator_length);
    }
}

