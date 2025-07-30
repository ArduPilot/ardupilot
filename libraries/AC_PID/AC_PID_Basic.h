#pragma once

/// @file	AC_PID_Basic.h
/// @brief	Lightweight PID controller with error and derivative filtering, integrator limit, and EEPROM gain storage.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "AP_PIDInfo.h"

/// @class	AC_PID_Basic
/// @brief	Copter PID control class
class AC_PID_Basic {
public:

    /// Constructor for PID controller with EEPROM-backed gain.
    /// Parameters are initialized from defaults or EEPROM at runtime.
    AC_PID_Basic(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_E_hz, float initial_filt_D_hz);

    // Computes the PID output using a target and measurement input.
    // Applies filters to the error and derivative, then updates the integrator.
    // If `limit` is true, the integrator is allowed to shrink but not grow.
    float update_all(float target, float measurement, float dt, bool limit = false) WARN_IF_UNUSED;
    float update_all(float target, float measurement, float dt, bool limit_neg, bool limit_pos) WARN_IF_UNUSED;

    // Updates the integrator using current error and dt.
    // If `limit_neg` is true, integrator may only increase.
    // If `limit_pos` is true, integrator may only decrease.
    void update_i(float dt, bool limit_neg, bool limit_pos);

    // get results from pid controller
    float get_p() const WARN_IF_UNUSED { return _error * _kp; }
    float get_i() const WARN_IF_UNUSED { return _integrator; }
    float get_d() const WARN_IF_UNUSED { return _derivative * _kd; }
    float get_ff() const WARN_IF_UNUSED { return _target * _kff; }
    float get_error() const WARN_IF_UNUSED { return _error; }

    // Resets the integrator to zero.
    void reset_I();

    // Flags the filter to reset on the next call to update_all().
    void reset_filter() { _reset_filter = true; }

    // Saves controller configuration from EEPROM, including gains and filter frequencies. (not used)
    void save_gains();

    // get accessors
    AP_Float &kP() WARN_IF_UNUSED { return _kp; }
    AP_Float &kI() WARN_IF_UNUSED { return _ki; }
    AP_Float &kD() WARN_IF_UNUSED { return _kd; }
    AP_Float &ff() WARN_IF_UNUSED { return _kff;}
    AP_Float &filt_E_hz() WARN_IF_UNUSED { return _filt_E_hz; }
    AP_Float &filt_D_hz() WARN_IF_UNUSED { return _filt_D_hz; }
    float imax() const WARN_IF_UNUSED { return _kimax.get(); }
    float get_filt_E_alpha(float dt) const WARN_IF_UNUSED;
    float get_filt_D_alpha(float dt) const WARN_IF_UNUSED;

    // set accessors
    void set_kP(float v) { _kp.set(v); }
    void set_kI(float v) { _ki.set(v); }
    void set_kD(float v) { _kd.set(v); }
    void set_ff(float v) { _kff.set(v); }
    void set_imax(float v) { _kimax.set(fabsf(v)); }
    void set_filt_E_hz(float hz) { _filt_E_hz.set(fabsf(hz)); }
    void set_filt_D_hz(float hz) { _filt_D_hz.set(fabsf(hz)); }

    // Sets the integrator directly, with overloads supporting raw I value, target + measurement, or error.
    // Internally clamps to IMAX.
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
    float _target;      // target value to enable filtering
    float _error;       // error value to enable filtering
    float _derivative;  // last derivative for low-pass filter
    float _integrator;  // integrator value
    bool _reset_filter; // true when input filter should be reset during next call to set_input

    AP_PIDInfo _pid_info;

private:
    const float default_kp;
    const float default_ki;
    const float default_kd;
    const float default_kff;
    const float default_kimax;
    const float default_filt_E_hz;
    const float default_filt_D_hz;
};
