#pragma once

/// @file	AC_PID_2D.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>

/// @class	AC_PID_2D
/// @brief	Copter PID control class
class AC_PID_2D {
public:

    // Constructor for PID
    AC_PID_2D(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float initial_filt_d_hz, float dt);

    // set_dt - set time step in seconds
    void        set_dt(float dt);

    // set_input - set input to PID controller
    //  input is filtered before the PID controllers are run
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input(const Vector2f &input);
    void        set_input(const Vector3f &input) { set_input(Vector2f(input.x, input.y)); }

    // get_pi - get results from pid controller
    Vector2f    get_pid();
    Vector2f    get_p() const;
    Vector2f    get_i();
    Vector2f    get_i_shrink();   // get_i but do not allow integrator to grow (it may shrink)
    Vector2f    get_d();

    // reset_I - reset the integrator
    void        reset_I();

    // reset_filter - input and D term filter will be reset to the next value provided to set_input()
    void        reset_filter();

    // load gain from eeprom
    void        load_gains();

    // save gain to eeprom
    void        save_gains();

    // get accessors
    AP_Float   &kP() { return _kp; }
    AP_Float   &kI() { return _ki; }
    float       imax() const { return _imax.get(); }
    float       filt_hz() const { return _filt_hz.get(); }
    float       get_filt_alpha() const { return _filt_alpha; }
    float       filt_d_hz() const { return _filt_hz.get(); }
    float       get_filt_alpha_D() const { return _filt_alpha_d; }

    // set accessors
    void        kP(const float v) { _kp.set(v); }
    void        kI(const float v) { _ki.set(v); }
    void        kD(const float v) { _kd.set(v); }
    void        imax(const float v) { _imax.set(fabsf(v)); }
    void        filt_hz(const float v);
    void        filt_d_hz(const float v);

    Vector2f    get_integrator() const { return _integrator; }
    void        set_integrator(const Vector2f &i) { _integrator = i; }
    void        set_integrator(const Vector3f &i) { _integrator.x = i.x; _integrator.y = i.y; }

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // set_input_filter_d - set input to PID controller
    //  only input to the D portion of the controller is filtered
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input_filter_d(const Vector2f& input_delta);

    // calc_filt_alpha - recalculate the input filter alpha
    void        calc_filt_alpha();

    // calc_filt_alpha - recalculate the input filter alpha
    void        calc_filt_alpha_d();

    // parameters
    AP_Float        _kp;
    AP_Float        _ki;
    AP_Float        _kd;
    AP_Float        _imax;
    AP_Float        _filt_hz;                   // PID Input filter frequency in Hz
    AP_Float        _filt_d_hz;                 // D term filter frequency in Hz

    // flags
    struct ac_pid_flags {
        bool        _reset_filter : 1;    // true when input filter should be reset during next call to set_input
    } _flags;

    // internal variables
    float           _dt;            // timestep in seconds
    float           _filt_alpha;    // input filter alpha
    float           _filt_alpha_d;  // input filter alpha
    Vector2f        _integrator;    // integrator value
    Vector2f        _input;         // last input for derivative
    Vector2f        _derivative;    // last derivative for low-pass filter
};
