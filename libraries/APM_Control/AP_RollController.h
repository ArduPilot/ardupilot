#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "AP_AutoTune.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>

class AP_RollController {
public:
    AP_RollController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms);

    /* Do not allow copies */
    AP_RollController(const AP_RollController &other) = delete;
    AP_RollController &operator=(const AP_RollController&) = delete;

	int32_t get_rate_out(float desired_rate, float scaler);
	int32_t get_servo_out(int32_t angle_err, float scaler, bool disable_integrator);

	void reset_I();

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I() {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
        rate_pid.set_integrator(rate_pid.get_i() * 0.995);
    }
    
    void autotune_start(void);
    void autotune_restore(void);

    const       AP_Logger::PID_Info& get_pid_info(void) const { return _pid_info; }

    static const struct AP_Param::GroupInfo var_info[];


    // tuning accessors
    void kP(float v) { rate_pid.kP().set(v); }
    void kI(float v) { rate_pid.kI().set(v); }
    void kD(float v) { rate_pid.kD().set(v); }
    void kFF(float v) { rate_pid.ff().set(v); }

    AP_Float &kP(void) { return rate_pid.kP(); }
    AP_Float &kI(void) { return rate_pid.kI(); }
    AP_Float &kD(void) { return rate_pid.kD(); }
    AP_Float &kFF(void) { return rate_pid.ff(); }

    void convert_pid();

private:
    const AP_Vehicle::FixedWing &aparm;
    AP_AutoTune::ATGains gains;
    AP_AutoTune *autotune;
    bool failed_autotune_alloc;
	uint32_t _last_t;
	float _last_out;
    AC_PID rate_pid{0.08, 0.15, 0, 0.345, 0.666, 3, 0, 12, 0.02, 150, 1};
    float angle_err_deg;

    AP_Logger::PID_Info _pid_info;

    int32_t _get_rate_out(float desired_rate, float scaler, bool disable_integrator);

    AP_AHRS &_ahrs;
};
