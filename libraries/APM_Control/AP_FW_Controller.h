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

    // Run angle controller
    float run_angle_control(int32_t desired_angle_cd, float scaler, bool disable_integrator, bool ground_mode);

    // Run pure rate control
    float run_rate_control(float desired_rate_degs, float scaler);

    // setup a one loop FF scale multiplier. This replaces any previous scale applied
    // so should only be used when only one source of scaling is needed
    void set_ff_scale(float _ff_scale) { ff_scale = _ff_scale; }

    // Reset I term
    void reset_I();

    // Reset controller
    void reset();

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

    // Get input shaping angle, rate, and accel for logging
    void get_input_shaping(float &angle_deg, float &rate_degs, float &accel_degss) const;

    // Get the angle error in degrees
    float get_angle_error_deg() const { return angle_err_deg; }

    // Reset the attitude target to such that a change in attitude due to an ahrs change is smooth
    void ahrs_reset();

    // Get angle P gain
    float get_angle_p() const;

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

    virtual float run_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode) = 0;

    float run_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode);

    // Return true if the airspeed should be considered as under speed
    virtual bool is_underspeed() const = 0;

    // Return the airspeed in m/s
    float get_airspeed() const;

    // Return the measured angle in degrees
    virtual float get_measured_angle_deg() const = 0;

    // Return the measured rate in radians per second
    virtual float get_measured_rate_rads() const = 0;

    // Return true if input shaping should be used
    bool should_apply_input_shaping() const;

    // Return true if rate limits should be applied
    virtual bool should_apply_rate_limits() const = 0;

    // Apply positive and negative rate limits to passed in value
    float rate_limit_degs(float rate_degs) const;

    // Return rate target offset in deg per second, this is used in angle control
    virtual float get_rate_target_offset_degs() const { return 0.0; }

    // Return positive rate limit in deg per second, zero if disabled
    virtual float get_positive_rate_limit_degs() const = 0;

    // Return negative rate limit in deg per second, zero if disabled
    virtual float get_negative_rate_limit_degs() const = 0;

    // Reset input shaping to the given values
    void reset_input_shaping_deg(const float angle_deg, const float rate_degs);

    // Set point tracking for input shaping
    float angle_target_deg;
    float rate_target_degs;
    float accel_target_degss;

    // Input shaping accel limit and angle P gain
    AP_Float accel_limit;
    AP_Float angle_p;

    const AP_AutoTune::ATType autotune_type;
};
