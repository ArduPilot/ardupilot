#pragma once

#include <AP_Common/AP_Common.h>
#include "AP_AutoTune.h"
#include <AC_PID/AC_PID.h>

class AP_FW_Controller
{
public:
    AP_FW_Controller(const AP_FixedWing &parms, const AC_PID::Defaults &defaults, AP_AutoTune::ATType _autotune_type);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_FW_Controller);

    float get_rate_out(float desired_rate, float scaler);
    virtual float get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode) = 0;

    // setup a one loop FF scale multiplier. This replaces any previous scale applied
    // so should only be used when only one source of scaling is needed
    void set_ff_scale(float _ff_scale) { ff_scale = _ff_scale; }

    void reset_I();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I();

    void autotune_start(void);
    void autotune_restore(void);

    const AP_PIDInfo& get_pid_info(void) const
    {
        return _pid_info;
    }

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) { rate_pid.set_notch_sample_rate(sample_rate); }

    AP_Float &kP(void) { return rate_pid.kP(); }
    AP_Float &kI(void) { return rate_pid.kI(); }
    AP_Float &kD(void) { return rate_pid.kD(); }
    AP_Float &kFF(void) { return rate_pid.ff(); }
    AP_Float &tau(void) { return gains.tau; }

protected:
    const AP_FixedWing &aparm;
    AP_AutoTune::ATGains gains;
    AP_AutoTune *autotune;
    bool failed_autotune_alloc;
    float _last_out;
    AC_PID rate_pid;
    float angle_err_deg;
    float ff_scale = 1.0;

    AP_PIDInfo _pid_info;

    float _get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed, bool ground_mode);

    virtual bool is_underspeed(const float aspeed) const = 0;

    virtual float get_airspeed() const = 0;

    virtual float get_measured_rate() const = 0;

    const AP_AutoTune::ATType autotune_type;
};
