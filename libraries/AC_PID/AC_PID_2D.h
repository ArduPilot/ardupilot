#pragma once

/// @file	AC_PID_2D.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <AC_PID/AP_PIDInfo.h>

/// @class	AC_PID_2D
/// @brief	Copter PID control class
class AC_PID_2D {
public:

    // Constructor for PID
    AC_PID_2D(float initial_kP, float initial_kI, float initial_kD, float initial_kFF, float initial_imax, float initial_filt_hz, float initial_filt_d_hz);

    CLASS_NO_COPY(AC_PID_2D);

    // update_all - set target and measured inputs to PID controller and calculate outputs
    // target and error are filtered
    // the derivative is then calculated and filtered
    // the integral is then updated if it does not increase in the direction of the limit vector
    Vector2f update_all(const Vector2f &target, const Vector2f &measurement, float dt, const Vector2f &limit);
    Vector2f update_all(const Vector3f &target, const Vector3f &measurement, float dt, const Vector3f &limit);

    // update the integral
    // if the limit flag is set the integral is only allowed to shrink
    void update_i(float dt, const Vector2f &limit);

    // get results from pid controller
    Vector2f get_p() const;
    const Vector2f& get_i() const;
    Vector2f get_d() const;
    Vector2f get_ff();
    const Vector2f& get_error() const { return _error; }

    // reset the integrator
    void reset_I();

    // reset_filter - input and D term filter will be reset to the next value provided to set_input()
    void reset_filter() { _reset_filter = true; }

    // save gain to eeprom
    void save_gains();

    // get accessors
    AP_Float &kP() { return _kp; }
    AP_Float &kI() { return _ki; }
    AP_Float &kD() { return _kd; }
    AP_Float &ff() { return _kff;}
    AP_Float &filt_E_hz() { return _filt_E_hz; }
    AP_Float &filt_D_hz() { return _filt_D_hz; }
    float imax() const { return _kimax.get(); }
    float get_filt_E_alpha(float dt) const;
    float get_filt_D_alpha(float dt) const;

    // set accessors
    void kP(float v) { _kp.set(v); }
    void kI(float v) { _ki.set(v); }
    void kD(float v) { _kd.set(v); }
    void ff(float v) { _kff.set(v); }
    void imax(float v) { _kimax.set(fabsf(v)); }
    void filt_E_hz(float hz) { _filt_E_hz.set(fabsf(hz)); }
    void filt_D_hz(float hz) { _filt_D_hz.set(fabsf(hz)); }

    // integrator setting functions
    void set_integrator(const Vector2f& target, const Vector2f& measurement, const Vector2f& i);
    void set_integrator(const Vector2f& error, const Vector2f& i);
    void set_integrator(const Vector3f& i) { set_integrator(Vector2f{i.x, i.y}); }
    void set_integrator(const Vector2f& i);

    const AP_PIDInfo& get_pid_info_x(void) const { return _pid_info_x; }
    const AP_PIDInfo& get_pid_info_y(void) const { return _pid_info_y; }

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

    // internal variables
    Vector2f    _target;        // target value to enable filtering
    Vector2f    _error;         // error value to enable filtering
    Vector2f    _derivative;    // last derivative from low-pass filter
    Vector2f    _integrator;    // integrator value
    bool        _reset_filter;  // true when input filter should be reset during next call to update_all

    AP_PIDInfo _pid_info_x;
    AP_PIDInfo _pid_info_y;

private:
    const float default_kp;
    const float default_ki;
    const float default_kd;
    const float default_kff;
    const float default_kimax;
    const float default_filt_E_hz;
    const float default_filt_D_hz;
};
