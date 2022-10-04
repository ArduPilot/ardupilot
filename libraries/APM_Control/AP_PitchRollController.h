#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "AP_AutoTune.h"
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>

// base class for pitch and roll controller
class AP_PitchRollController
{

public:

    AP_PitchRollController(const AP_Vehicle::FixedWing &parms, AP_AutoTune::ATType _autotune_axis, const char *_axis_name);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_PitchRollController);

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I()
    {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
        rate_pid.set_integrator(rate_pid.get_i() * 0.995);
    }

    void autotune_start(void);
    void autotune_restore(void);

    void reset_I();

    const AP_PIDInfo& get_pid_info(void) const
    {
        return _pid_info;
    }

    // tuning accessors
    AP_Float &kP(void) { return rate_pid.kP(); }
    AP_Float &kI(void) { return rate_pid.kI(); }
    AP_Float &kD(void) { return rate_pid.kD(); }
    AP_Float &kFF(void) { return rate_pid.ff(); }
    AP_Float &tau(void) { return gains.tau; }

protected:
    const AP_Vehicle::FixedWing &aparm;
    AP_AutoTune::ATGains gains;
    AP_AutoTune *autotune;
    bool failed_autotune_alloc;
    float _last_out;
    AC_PID rate_pid{0.08, 0.15, 0, 0.345, 0.666, 3, 0, 12, 0.02, 150, 1};
    float angle_err_deg;

    AP_PIDInfo _pid_info;

private:

    AP_AutoTune::ATType autotune_axis;
    const char *axis_name;
};
