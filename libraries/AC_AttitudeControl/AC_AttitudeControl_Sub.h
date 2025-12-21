#pragma once

/// @file    AC_AttitudeControl_Sub.h
/// @brief   ArduSub attitude control library

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

// default angle controller PID gains
// (Sub-specific defaults for parent class)
#define AC_ATC_SUB_ANGLE_P             6.0f
#define AC_ATC_SUB_ACCEL_Y_MAX         110000.0f

// default rate controller PID gains
#define AC_ATC_SUB_RATE_RP_P           0.135f
#define AC_ATC_SUB_RATE_RP_I           0.090f
#define AC_ATC_SUB_RATE_RP_D           0.0036f
#define AC_ATC_SUB_RATE_RP_IMAX        0.444f
#define AC_ATC_SUB_RATE_RP_FILT_HZ     30.0f
#define AC_ATC_SUB_RATE_YAW_P          0.180f
#define AC_ATC_SUB_RATE_YAW_I          0.018f
#define AC_ATC_SUB_RATE_YAW_D          0.0f
#define AC_ATC_SUB_RATE_YAW_IMAX       0.222f
#define AC_ATC_SUB_RATE_YAW_FILT_HZ    5.0f

#define MAX_YAW_ERROR                  radians(5)

class AC_AttitudeControl_Sub : public AC_AttitudeControl {
public:
    AC_AttitudeControl_Sub(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors);

    // empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl_Sub() {}

    // pid accessors
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }
    const AC_PID& get_rate_roll_pid() const override { return _pid_rate_roll; }
    const AC_PID& get_rate_pitch_pid() const override { return _pid_rate_pitch; }
    const AC_PID& get_rate_yaw_pid() const override { return _pid_rate_yaw; }

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // Calculate body-frame throttle required to produce the given earth-frame throttle input (accounts for vehicle tilt)
	float get_throttle_boosted(float throttle_in);

    // Set desired throttle vs attitude mixing (actual mix is slewed toward this value over 1~2 seconds)
    // Low values favor pilot/autopilot throttle over attitude control; high values prioritize attitude control
    // Has no effect when throttle is above hover throttle
    void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
    void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
    void set_throttle_mix_max(float ratio) override { _throttle_rpy_mix_desired = _thr_mix_max; }

    // Returns true if throttle mix is near minimum (i.e., attitude control is deprioritised)
    bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f*_thr_mix_min); }

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run() override;

    // sanity check parameters.  should be called once before take-off
    void parameter_sanity_check() override;

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) override;

    // Sets desired roll, pitch, and yaw angles (in centidegrees), with yaw slewing.
    // Slews toward target yaw at a fixed rate (in centidegrees/s) until the error is within 5 degrees.
    // Used to enforce consistent heading changes without large instantaneous yaw errors.
    void input_euler_angle_roll_pitch_slew_yaw_cd(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, float slew_yaw_rate_cds);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // Slews the throttle-to-attitude mix ratio (_throttle_rpy_mix) toward the requested value (_throttle_rpy_mix_desired).
    // Increases rapidly and decreases more slowly to ensure stability during transitions.
    void update_throttle_rpy_mix();

    // Returns a throttle value that accounts for the priority of attitude control over throttle.
    // This allows graceful reduction of control authority as thrust approaches its minimum.
    // returns a throttle including compensation for roll/pitch angle
    // throttle value should be 0 ~ 1
    float get_throttle_avg_max(float throttle_in);

    AP_MotorsMulticopter& _motors_multi;

    // Roll and Pitch rate PIDs share the same defaults:
    const AC_PID::Defaults rp_defaults {
        AC_PID::Defaults{
            .p         = AC_ATC_SUB_RATE_RP_P,
            .i         = AC_ATC_SUB_RATE_RP_I,
            .d         = AC_ATC_SUB_RATE_RP_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_SUB_RATE_RP_IMAX,
            .filt_T_hz = AC_ATC_SUB_RATE_RP_FILT_HZ,
            .filt_E_hz = 0.0,
            .filt_D_hz = AC_ATC_SUB_RATE_RP_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };
    AC_PID                _pid_rate_roll { rp_defaults };
    AC_PID                _pid_rate_pitch { rp_defaults };

    AC_PID                _pid_rate_yaw {
        AC_PID::Defaults{
            .p         = AC_ATC_SUB_RATE_YAW_P,
            .i         = AC_ATC_SUB_RATE_YAW_I,
            .d         = AC_ATC_SUB_RATE_YAW_D,
            .ff        = 0.0f,
            .imax      = AC_ATC_SUB_RATE_YAW_IMAX,
            .filt_T_hz = AC_ATC_SUB_RATE_YAW_FILT_HZ,
            .filt_E_hz = 0.0f,
            .filt_D_hz = AC_ATC_SUB_RATE_YAW_FILT_HZ,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };

    AP_Float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
};
