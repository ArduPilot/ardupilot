#pragma once

/// @file	AC_PI_2D.h
/// @brief	2-axis PI controller with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>

#define AC_PI_2D_FILT_HZ_MIN      0.01f   // minimum input filter frequency

/// @class	AC_PI_2D
/// @brief	2-axis PI controller
class AC_PI_2D {
public:

    // constructor
    AC_PI_2D(float initial_p, float initial_i, float initial_imax, float initial_filt_hz, float dt);

    CLASS_NO_COPY(AC_PI_2D);

    // set time step in seconds
    void set_dt(float dt);

    // set_input - set input to PI controller
    //  input is filtered before the PI controllers are run
    //  this should be called before any other calls to get_p, get_i or get_d
    void set_input(const Vector2f &input);
    void set_input(const Vector3f &input) { set_input(Vector2f{input.x, input.y}); }

    // get_pi - get results from pid controller
    Vector2f get_pi();
    Vector2f get_p() const;
    Vector2f get_i();
    Vector2f get_i_shrink();   // get_i but do not allow integrator to grow (it may shrink)

    // reset_I - reset the integrator
    void reset_I();

    // reset_filter - input filter will be reset to the next value provided to set_input()
    void reset_filter();

    // Loads controller configuration from EEPROM, including gains and filter frequencies. (not used)
    void load_gains();

    // Saves controller configuration from EEPROM, including gains and filter frequencies. (not used)
    void save_gains();

    // get accessors
    AP_Float &kP() { return _kp; }
    AP_Float &kI() { return _ki; }
    float imax() const { return _imax.get(); }
    float filt_hz() const { return _filt_hz.get(); }
    float get_filt_alpha() const { return _filt_alpha; }

    // set accessors
    void kP(float v) { _kp.set(v); }
    void kI(float v) { _ki.set(v); }
    void imax(float v) { _imax.set(fabsf(v)); }
    void filt_hz(float hz);

    Vector2f get_integrator() const { return _integrator; }
    void set_integrator(const Vector2f &i) { _integrator = i; }
    void set_integrator(const Vector3f &i) { _integrator.x = i.x; _integrator.y = i.y; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // calc_filt_alpha - recalculate the input filter alpha
    void calc_filt_alpha();

    // parameters
    AP_Float _kp;
    AP_Float _ki;
    AP_Float _imax;
    AP_Float _filt_hz;  // PI Input filter frequency in Hz

    // flags
    struct ac_pid_flags {
        bool _reset_filter;    // true when input filter should be reset during next call to set_input
    } _flags;

    // internal variables
    float _dt;              // time step in seconds
    Vector2f _integrator;   // integrator value
    Vector2f _input;        // last input for derivative
    float _filt_alpha;      // input filter alpha

    const float default_kp;
    const float default_ki;
    const float default_imax;
    const float default_filt_hz;

};
