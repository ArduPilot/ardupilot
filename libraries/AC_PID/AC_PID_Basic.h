#pragma once

/// @file	AC_PID_Basic.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "AP_PIDInfo.h"

/// @class	AC_PID_Basic
/// @brief	Copter PID control class
class AC_PID_Basic {
public:

    // Constructor for PID
    AC_PID_Basic(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_E_hz, float initial_filt_D_hz, float dt);

    // set time step in seconds
    void set_dt(float dt) { _dt = dt; }

    // set target and measured inputs to PID controller and calculate outputs
    // target and error are filtered
    // the derivative is then calculated and filtered
    // the integral is then updated based on the setting of the limit flag
    float update_all(float target, float measurement, bool limit = false) WARN_IF_UNUSED;
    float update_all(float target, float measurement, bool limit_neg, bool limit_pos) WARN_IF_UNUSED;

    // update the integral
    // if the limit flags are set the integral is only allowed to shrink
    void update_i(bool limit_neg, bool limit_pos);

    // get results from pid controller
    float get_p() const WARN_IF_UNUSED { return _error * _kp; }
    float get_i() const WARN_IF_UNUSED { return _integrator; }
    float get_d() const WARN_IF_UNUSED { return _derivative * _kd; }
    float get_ff() const WARN_IF_UNUSED { return _target * _kff; }
    float get_error() const WARN_IF_UNUSED { return _error; }

    // reset the integrator
    void reset_I();

    // input and D term filter will be reset to the next value provided to set_input()
    void reset_filter() { _reset_filter = true; }

    // save gain to eeprom
    void save_gains();

    // get accessors
    AP_Float &kP() WARN_IF_UNUSED { return _kp; }
    AP_Float &kI() WARN_IF_UNUSED { return _ki; }
    AP_Float &kD() WARN_IF_UNUSED { return _kd; }
    AP_Float &ff() WARN_IF_UNUSED { return _kff;}
    AP_Float &filt_E_hz() WARN_IF_UNUSED { return _filt_E_hz; }
    AP_Float &filt_D_hz() WARN_IF_UNUSED { return _filt_D_hz; }
    float imax() const WARN_IF_UNUSED { return _kimax.get(); }
    float get_filt_E_alpha() const WARN_IF_UNUSED;
    float get_filt_D_alpha() const WARN_IF_UNUSED;

    // set accessors
    void kP(float v) { _kp.set(v); }
    void kI(float v) { _ki.set(v); }
    void kD(float v) { _kd.set(v); }
    void ff(float v) { _kff.set(v); }
    void imax(float v) { _kimax.set(fabsf(v)); }
    void filt_E_hz(float hz) { _filt_E_hz.set(fabsf(hz)); }
    void filt_D_hz(float hz) { _filt_D_hz.set(fabsf(hz)); }

    // integrator setting functions
    void set_integrator(float target, float measurement, float i);
    void set_integrator(float error, float i);
    void set_integrator(float i);

    const AP_PIDInfo& get_pid_info(void) const WARN_IF_UNUSED { return _pid_info; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // parameters
    AP_Float _kp;
    AP_Float _ki;
    AP_Float _kd;
    AP_Float _kff;
    AP_Float _kimax;
    AP_Float _filt_E_hz;         // PID error filter frequency in Hz
    AP_Float _filt_D_hz;         // PID derivative filter frequency in Hz

    // internal variables
    float _dt;          // timestep in seconds
    float _target;      // target value to enable filtering
    float _error;       // error value to enable filtering
    float _derivative;  // last derivative for low-pass filter
    float _integrator;  // integrator value
    bool _reset_filter; // true when input filter should be reset during next call to set_input

    AP_PIDInfo _pid_info;
};
