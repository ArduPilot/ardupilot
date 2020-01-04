#pragma once

/// @file	AC_PID_1D.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <AP_Logger/AP_Logger.h>

#define AC_PID_1D_FILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_PID_1D_FILT_HZ_MIN      0.01f   // minimum input filter frequency
#define AC_PID_1D_FILT_D_HZ_DEFAULT  10.0f   // default input filter frequency
#define AC_PID_1D_FILT_D_HZ_MIN      0.005f   // minimum input filter frequency

/// @class	AC_PID_1D
/// @brief	Copter PID control class
class AC_PID_1D {
public:

    // Constructor for PID
    AC_PID_1D(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_hz, float initial_filt_d_hz, float dt);

    // set_dt - set time step in seconds
    void        set_dt(float dt);

    //  update_all - set target and measured inputs to PID controller and calculate outputs
    //  target and error are filtered
    //  the derivative is then calculated and filtered
    //  the integral is then updated based on the setting of the limit flag
    float update_all(float target, float measurement, bool limit = false) {return update_all(target, measurement, (limit && is_negative(_integrator)), (limit && is_positive(_integrator)));};
    float update_all(float target, float measurement, bool limit_neg, bool limit_pos);

    //  update_i - update the integral
    //  if the limit flag is set the integral is only allowed to shrink
    void update_i(bool limit_neg, bool limit_pos);

    // get_pi - get results from pid controller
    float get_p() const;
    float get_i() const;
    float get_d() const;
    float get_ff();
    float get_error() const {return _error;}

    // reset_I - reset the integrator
    void        reset_I();

    // reset_filter - input and D term filter will be reset to the next value provided to set_input()
    void        reset_filter() {
        _flags._reset_filter = true;
    }

    // load gain from eeprom
    void        load_gains();

    // save gain to eeprom
    void        save_gains();

    /// operator function call for easy initialisation
    void operator()(float p_val, float i_val, float d_val, float ff_val, float imax_val, float input_filt_E_hz, float input_filt_D_hz, float dt);

    // get accessors
    AP_Float &kP() { return _kp; }
    AP_Float &kI() { return _ki; }
    AP_Float &kD() { return _kd; }
    AP_Float &ff() { return _kff;}
    AP_Float &filt_E_hz() { return _filt_E_hz; }
    AP_Float &filt_D_hz() { return _filt_D_hz; }
    float imax() const { return _kimax.get(); }
    float get_filt_alpha(float filt_hz) const;
    float get_filt_E_alpha() const;
    float get_filt_D_alpha() const;

    // set accessors
    void kP(const float v) { _kp.set(v); }
    void kI(const float v) { _ki.set(v); }
    void kD(const float v) { _kd.set(v); }
    void ff(const float v) { _kff.set(v); }
    void imax(const float v) { _kimax.set(fabsf(v)); }
    void filt_E_hz(const float v);
    void filt_D_hz(const float v);

    // integrator setting functions
    void set_integrator(float target, float measurement, float i);
    void set_integrator(float error, float i);
    void set_integrator(float i);

    const AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // parameters
    AP_Float _kp;
    AP_Float _ki;
    AP_Float _kd;
    AP_Float _kff;
    AP_Float _kimax;
    AP_Float _filt_E_hz;         // PID error filter frequency in Hz
    AP_Float _filt_D_hz;         // PID derivative filter frequency in Hz

    // flags
    struct ac_pid_flags {
        bool        _reset_filter : 1;    // true when input filter should be reset during next call to set_input
    } _flags;

    // internal variables
    float           _dt;            // timestep in seconds
    float        _target;        // target value to enable filtering
    float        _error;         // error value to enable filtering
    float        _derivative;    // last derivative for low-pass filter
    float        _integrator;    // integrator value

    AP_Logger::PID_Info _pid_info;
};
