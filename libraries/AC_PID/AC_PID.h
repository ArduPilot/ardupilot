// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef __AC_PID_H__
#define __AC_PID_H__

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <math.h>
#include <DataFlash/DataFlash.h>

#define AC_PID_FILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_PID_FILT_HZ_MIN      0.01f   // minimum input filter frequency

/// @class	AC_PID
/// @brief	Copter PID control class
class AC_PID {
public:

    // Constructor for PID
    AC_PID(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt);

    // set_dt - set time step in seconds
    void        set_dt(float dt);

    // set_input_filter_all - set input to PID controller
    //  input is filtered before the PID controllers are run
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input_filter_all(float input);

    // set_input_filter_d - set input to PID controller
    //  only input to the D portion of the controller is filtered
    //  this should be called before any other calls to get_p, get_i or get_d
    void        set_input_filter_d(float input);

    // get_pid - get results from pid controller
    float       get_pid();
    float       get_pi();
    float       get_p();
    float       get_i();
    float       get_d();

    // reset_I - reset the integrator
    void        reset_I();

    // reset_filter - input filter will be reset to the next value provided to set_input()
    void        reset_filter();

    // load gain from eeprom
    void        load_gains();

    // save gain to eeprom
    void        save_gains();

    /// operator function call for easy initialisation
    void operator() (float p, float i, float d, float imaxval, float input_filt_hz, float dt );

    // get accessors
    float       kP() const { return _kp.get(); }
    float       kI() const { return _ki.get(); }
    float       kD() const { return _kd.get(); }
    float       imax() const { return _imax.get(); }
    float       filt_hz() const { return _filt_hz.get(); }
    float       get_filt_alpha() const;

    // set accessors
    void        kP(const float v) { _kp.set(v); }
    void        kI(const float v) { _ki.set(v); }
    void        kD(const float v) { _kd.set(v); }
    void        imax(const float v) { _imax.set(fabsf(v)); }
    void        filt_hz(const float v);

    float       get_integrator() const { return _integrator; }
    void        set_integrator(float i) { _integrator = i; }

    // set the designed rate (for logging purposes)
    void        set_desired_rate(float desired) { _pid_info.desired = desired; }

    const       DataFlash_Class::PID_Info& get_pid_info(void) const { return _pid_info; }

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // parameters
    AP_Float        _kp;
    AP_Float        _ki;
    AP_Float        _kd;
    AP_Float        _imax;
    AP_Float        _filt_hz;                   // PID Input filter frequency in Hz

    // flags
    struct ac_pid_flags {
        bool        _reset_filter : 1;    // true when input filter should be reset during next call to set_input
    } _flags;

    // internal variables
    float           _dt;                    // timestep in seconds
    float           _integrator;            // integrator value
    float           _input;                 // last input for derivative
    float           _derivative;            // last derivative for low-pass filter

    DataFlash_Class::PID_Info        _pid_info;
};

#endif // __AC_PID_H__
