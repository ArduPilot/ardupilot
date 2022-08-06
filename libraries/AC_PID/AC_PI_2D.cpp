/// @file	AC_PI_2D.cpp
/// @brief	2-axis PI controller

#include <AP_Math/AP_Math.h>
#include "AC_PI_2D.h"

const AP_Param::GroupInfo AC_PI_2D::var_info[] = {
    // @Param: P
    // @DisplayName: PI Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_PI_2D, _kp, 0),

    // @Param: I
    // @DisplayName: PI Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I",    1, AC_PI_2D, _ki, 0),

    // @Param: IMAX
    // @DisplayName: PI Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 2, AC_PI_2D, _imax, 0),

    // @Param: FILT_HZ
    // @DisplayName: PI Input filter frequency in Hz
    // @Description: Input filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO("FILT_HZ", 3, AC_PI_2D, _filt_hz, AC_PI_2D_FILT_HZ_DEFAULT),

    AP_GROUPEND
};

// Constructor
AC_PI_2D::AC_PI_2D(float initial_p, float initial_i, float initial_imax, float initial_filt_hz, float dt) :
    _dt(dt)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp.set_and_default(initial_p);
    _ki.set_and_default(initial_i);
    _imax.set_and_default(initial_imax);
    _filt_hz.set_and_default(initial_filt_hz);
    filt_hz(initial_filt_hz);

    // reset input filter to first value received
    _flags._reset_filter = true;
}

// set_dt - set time step in seconds
void AC_PI_2D::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
    calc_filt_alpha();
}

// filt_hz - set input filter hz
void AC_PI_2D::filt_hz(float hz)
{
    _filt_hz.set(fabsf(hz));

    // sanity check _filt_hz
    _filt_hz.set(MAX(_filt_hz, AC_PI_2D_FILT_HZ_MIN));

    // calculate the input filter alpha
    calc_filt_alpha();
}

// set_input - set input to PID controller
//  input is filtered before the PID controllers are run
//  this should be called before any other calls to get_p, get_i or get_d
void AC_PI_2D::set_input(const Vector2f &input)
{
    // don't process inf or NaN
    if (!isfinite(input.x) || !isfinite(input.y)) {
        return;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _input = input;
    }

    // update filter and calculate derivative
    Vector2f input_filt_change = (input - _input) * _filt_alpha;
    _input = _input + input_filt_change;
}

Vector2f AC_PI_2D::get_p() const
{
    return (_input * _kp);
}

Vector2f AC_PI_2D::get_i()
{
    if (!is_zero(_ki) && !is_zero(_dt)) {
        _integrator += (_input * _ki) * _dt;
        const float integrator_length = _integrator.length();
        if ((integrator_length > _imax) && (is_positive(integrator_length))) {
            _integrator *= (_imax / integrator_length);
        }
        return _integrator;
    }
    return Vector2f{};
}

// get_i_shrink - get_i but do not allow integrator to grow in length (it may shrink)
Vector2f AC_PI_2D::get_i_shrink()
{
    if (!is_zero(_ki) && !is_zero(_dt)) {
        const float integrator_length_orig = MIN(_integrator.length(),_imax);
        _integrator += (_input * _ki) * _dt;
        const float integrator_length_new = _integrator.length();
        if ((integrator_length_new > integrator_length_orig) && is_positive(integrator_length_new)) {
            _integrator *= (integrator_length_orig / integrator_length_new);
        }
        return _integrator;
    }
    return Vector2f{};
}

Vector2f AC_PI_2D::get_pi()
{
    return get_p() + get_i();
}

void AC_PI_2D::reset_I()
{
    _integrator.zero();
}

void AC_PI_2D::load_gains()
{
    _kp.load();
    _ki.load();
    _imax.load();
    _imax.set(fabsf(_imax));
    _filt_hz.load();

    // calculate the input filter alpha
    calc_filt_alpha();
}

// save_gains - save gains to eeprom
void AC_PI_2D::save_gains()
{
    _kp.save();
    _ki.save();
    _imax.save();
    _filt_hz.save();
}

/// Overload the function call operator to permit easy initialisation
void AC_PI_2D::operator() (float p, float i, float imaxval, float input_filt_hz, float dt)
{
    _kp.set(p);
    _ki.set(i);
    _imax.set(fabsf(imaxval));
    _filt_hz.set(input_filt_hz);
    _dt = dt;
    // calculate the input filter alpha
    calc_filt_alpha();
}

// calc_filt_alpha - recalculate the input filter alpha
void AC_PI_2D::calc_filt_alpha()
{
    if (is_zero(_filt_hz)) {
        _filt_alpha = 1.0f;
        return;
    }
  
    // calculate alpha
    const float rc = 1/(M_2PI*_filt_hz);
    _filt_alpha = _dt / (_dt + rc);
}
