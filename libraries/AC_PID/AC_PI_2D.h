// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

/// @file	AC_PI_2D.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>

#define AC_PI_2D_FILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_PI_2D_FILT_HZ_MIN      0.01f   // minimum input filter frequency

/// @class	AC_PI_2D
/// @brief	Copter PID control class
class AC_PI_2D {
public:

    // Constructor for PID
    AC_PI_2D(float initial_p, float initial_i, float initial_imax, float initial_filt_hz, float dt);

    // set_dt - set time step in seconds
    void        set_dt(float dt);

    // set_input - set input to PID controller
    //  input is filtered before the PID controllers are run
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input(const Vector2f &input);
    void        set_input(const Vector3f &input) { set_input(Vector2f(input.x, input.y)); }

    // get_pi - get results from pid controller
    Vector2f    get_pi();
    Vector2f    get_p() const;
    Vector2f    get_i();
    Vector2f    get_i_shrink();   // get_i but do not allow integrator to grow (it may shrink)

    // reset_I - reset the integrator
    void        reset_I();

    // reset_filter - input filter will be reset to the next value provided to set_input()
    void        reset_filter();

    // load gain from eeprom
    void        load_gains();

    // save gain to eeprom
    void        save_gains();

    /// operator function call for easy initialisation
    void operator() (float p, float i, float imaxval, float input_filt_hz, float dt);

    // get accessors
    AP_Float   &kP() { return _kp; }
    AP_Float   &kI() { return _ki; }
    float       imax() const { return _imax.get(); }
    float       filt_hz() const { return _filt_hz.get(); }
    float       get_filt_alpha() const { return _filt_alpha; }

    // set accessors
    void        kP(const float v) { _kp.set(v); }
    void        kI(const float v) { _ki.set(v); }
    void        imax(const float v) { _imax.set(fabsf(v)); }
    void        filt_hz(const float v);

    Vector2f    get_integrator() const { return _integrator; }
    void        set_integrator(const Vector2f &i) { _integrator = i; }
    void        set_integrator(const Vector3f &i) { _integrator.x = i.x; _integrator.y = i.y; }

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // calc_filt_alpha - recalculate the input filter alpha
    void        calc_filt_alpha();

    // parameters
    AP_Float        _kp;
    AP_Float        _ki;
    AP_Float        _imax;
    AP_Float        _filt_hz;                   // PID Input filter frequency in Hz

    // flags
    struct ac_pid_flags {
        bool        _reset_filter : 1;    // true when input filter should be reset during next call to set_input
    } _flags;

    // internal variables
    float           _dt;            // timestep in seconds
    Vector2f        _integrator;    // integrator value
    Vector2f        _input;         // last input for derivative
    float           _filt_alpha;    // input filter alpha
};
