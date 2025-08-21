#pragma once

/// @file    AC_AttitudeControl_Heli.h
/// @brief   ArduCopter attitude control library for traditional helicopters

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsHeli.h>
#include <AC_PID/AC_HELI_PID.h>
#include <Filter/Filter.h>

// default rate controller PID gains
#define AC_ATC_HELI_RATE_RP_P                       0.024f
#define AC_ATC_HELI_RATE_RP_I                       0.15f
#define AC_ATC_HELI_RATE_RP_D                       0.001f
#define AC_ATC_HELI_RATE_RP_IMAX                    0.4f
#define AC_ATC_HELI_RATE_RP_FF                      0.15f
#define AC_ATC_HELI_RATE_RP_FILT_HZ                 20.0f
#define AC_ATC_HELI_RATE_YAW_P                      0.18f
#define AC_ATC_HELI_RATE_YAW_I                      0.12f
#define AC_ATC_HELI_RATE_YAW_D                      0.003f
#define AC_ATC_HELI_RATE_YAW_IMAX                   0.4f
#define AC_ATC_HELI_RATE_YAW_FF                     0.024f
#define AC_ATC_HELI_RATE_YAW_FILT_HZ                20.0f

#define AC_ATTITUDE_HELI_ANGLE_LIMIT_THROTTLE_MAX   0.95f    // Heli's use 95% of max collective before limiting frame angle
#define AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE  0.02f
#define AC_ATTITUDE_HELI_RATE_RP_FF_FILTER          20.0f
#define AC_ATTITUDE_HELI_RATE_Y_FF_FILTER          20.0f
#define AC_ATTITUDE_HELI_HOVER_ROLL_TRIM_DEFAULT    300
#define AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD   radians(30.0f)
#define AC_ATTITUDE_HELI_INVERTED_TRANSITION_TIME    3.0f

class AC_AttitudeControl_Heli : public AC_AttitudeControl {
public:
    AC_AttitudeControl_Heli( AP_AHRS_View &ahrs,
                        const AP_MultiCopter &aparm,
                        AP_MotorsHeli& motors);

    // pid accessors
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }
    const AC_PID& get_rate_roll_pid() const override { return _pid_rate_roll; }
    const AC_PID& get_rate_pitch_pid() const override { return _pid_rate_pitch; }
    const AC_PID& get_rate_yaw_pid() const override { return _pid_rate_yaw; }

    // passthrough_bf_roll_pitch_rate_yaw_norm - roll and pitch are passed through directly, body-frame rate target for yaw
    void passthrough_bf_roll_pitch_rate_yaw_norm(float roll_passthrough_norm, float pitch_passthrough_norm, float yaw_passthrough_norm) override;

    // subclass non-passthrough too, for external gyro, no flybar
    void input_rate_bf_roll_pitch_yaw_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads) override;

	// rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
	// should be called at 100hz or more
	virtual void rate_controller_run() override;

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

	// use_leaky_i - controls whether we use leaky i term for body-frame to motor output stage
	void use_leaky_i(bool leaky_i) override {  _flags_heli.leaky_i = leaky_i; }
    
    // use_flybar_passthrough - controls whether we pass-through
    // control inputs to swash-plate and tail
    void use_flybar_passthrough(bool passthrough, bool tail_passthrough) override {  
        _flags_heli.flybar_passthrough = passthrough; 
        _flags_heli.tail_passthrough = tail_passthrough; 
    }

    // set_hover_roll_scalar - scales Hover Roll Trim parameter. To be used by vehicle code according to vehicle condition.
    void set_hover_roll_trim_scalar(float scalar) override {_hover_roll_trim_scalar = constrain_float(scalar, 0.0f, 1.0f);}

    // get_roll_trim - angle in centi-degrees to be added to roll angle for learn hover collective. Used by helicopter to counter tail rotor thrust in hover
    float get_roll_trim_cd() override;

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_throttle_boosted(float throttle_in);

    // Sets desired roll and pitch angles (in radians) and yaw rate (in radians/s).
    // Used when roll/pitch stabilization is needed with manual or autonomous yaw rate control.
    // Applies acceleration-limited input shaping for smooth transitions and computes body-frame angular velocity targets.
    void input_euler_angle_roll_pitch_euler_rate_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads) override;

    // Sets desired roll, pitch, and yaw angles (in radians).
    // Used to follow an absolute attitude setpoint. Input shaping and yaw slew limits are applied.
    // Outputs are passed to the rate controller via shaped angular velocity targets.
    void input_euler_angle_roll_pitch_yaw_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_angle_rad, bool slew_yaw) override;
    
    // Sets desired thrust vector and heading rate (in radians/s).
    // Used for tilt-based navigation with independent yaw control.
    // The thrust vector defines the desired orientation (e.g., pointing direction for vertical thrust),
    // while the heading rate adjusts yaw. The input is shaped by acceleration and slew limits.
    void input_thrust_vector_rate_heading_rads(const Vector3f& thrust_vector, float heading_rate_rads, bool slew_yaw = true) override;
    
    // Sets desired thrust vector and heading (in radians) with heading rate (in radians/s).
    // Used for advanced attitude control where thrust direction is separated from yaw orientation.
    // Heading slew is constrained based on configured limits.
    void input_thrust_vector_heading_rad(const Vector3f& thrust_vector, float heading_angle_rad, float heading_rate_rads) override;

    // enable/disable inverted flight
    void set_inverted_flight(bool inverted) override { _inverted_flight = inverted; }

    // accessor for inverted flight flag
    bool get_inverted_flight() override { return _inverted_flight; }

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    // To-Do: move these limits flags into the heli motors class
    struct AttControlHeliFlags {
        uint8_t leaky_i             :   1;  // 1 if we should use leaky i term for body-frame rate to motor stage
        uint8_t flybar_passthrough  :   1;  // 1 if we should pass through pilots roll & pitch input directly to swash-plate
        uint8_t tail_passthrough    :   1;  // 1 if we should pass through pilots yaw input to tail
    } _flags_heli;

    // true in inverted flight mode
    bool _inverted_flight;

    // Integrate vehicle rate into _att_error_rot_vec_rad
    void integrate_bf_rate_error_to_angle_errors();

    //
    // body-frame rate controller
    //
	// rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target body-frame rate (in radians/sec) for roll, pitch and yaw
    // outputs are sent directly to motor class
    void rate_bf_to_motor_roll_pitch(const Vector3f &rate_rads, float rate_roll_target_rads, float rate_pitch_target_rads);
    float rate_target_to_motor_yaw(float rate_yaw_actual_rads, float rate_yaw_rads);

    //
    // throttle methods
    //
    
    // pass through for roll and pitch
    float _passthrough_roll_norm;
    float _passthrough_pitch_norm;

    // pass through for yaw if tail_passthrough is set
    float _passthrough_yaw_norm;

    // internal variables
    float _hover_roll_trim_scalar = 0;              // scalar used to suppress Hover Roll Trim


    // This represents an euler axis-angle rotation vector from the vehicles
    // estimated attitude to the reference (setpoint) attitude used in the attitude
    // controller, in radians in the vehicle body frame of reference.
    Vector3f            _att_error_rot_vec_rad;

    // parameters
    AP_Int8         _piro_comp_enabled;             // Flybar present or not.  Affects attitude controller used during ACRO flight mode
    AP_Int16        _hover_roll_trim_cd;               // Angle in centi-degrees used to counter tail rotor thrust in hover

    // Roll and Pitch rate PIDs share the same defaults:
    const AC_PID::Defaults rp_defaults {
        AC_PID::Defaults{
            .p         = AC_ATC_HELI_RATE_RP_P,
            .i         = AC_ATC_HELI_RATE_RP_I,
            .d         = AC_ATC_HELI_RATE_RP_D,
            .ff        = AC_ATC_HELI_RATE_RP_FF,
            .imax      = AC_ATC_HELI_RATE_RP_IMAX,
            .filt_T_hz = AC_ATTITUDE_HELI_RATE_RP_FF_FILTER,
            .filt_E_hz = AC_ATC_HELI_RATE_RP_FILT_HZ,
            .filt_D_hz = 0.0,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };
    AC_HELI_PID     _pid_rate_roll { rp_defaults };
    AC_HELI_PID     _pid_rate_pitch { rp_defaults };

    AC_HELI_PID     _pid_rate_yaw {
        AC_PID::Defaults{
            .p         = AC_ATC_HELI_RATE_YAW_P,
            .i         = AC_ATC_HELI_RATE_YAW_I,
            .d         = AC_ATC_HELI_RATE_YAW_D,
            .ff        = AC_ATC_HELI_RATE_YAW_FF,
            .imax      = AC_ATC_HELI_RATE_YAW_IMAX,
            .filt_T_hz = AC_ATTITUDE_HELI_RATE_Y_FF_FILTER,
            .filt_E_hz = AC_ATC_HELI_RATE_YAW_FILT_HZ,
            .filt_D_hz = 0.0,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };
    
};
