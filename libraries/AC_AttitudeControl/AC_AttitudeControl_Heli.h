#pragma once

/// @file    AC_AttitudeControl_Heli.h
/// @brief   ArduCopter attitude control library for traditional helicopters

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsHeli.h>
#include <AC_PID/AC_HELI_PID.h>
#include <Filter/Filter.h>

// default rate controller PID gains
#define AC_ATC_HELI_RATE_RP_P                       0.024f
#define AC_ATC_HELI_RATE_RP_I                       0.6f
#define AC_ATC_HELI_RATE_RP_D                       0.001f
#define AC_ATC_HELI_RATE_RP_IMAX                    1.0f
#define AC_ATC_HELI_RATE_RP_FF                      0.060f
#define AC_ATC_HELI_RATE_RP_FILT_HZ                 20.0f
#define AC_ATC_HELI_RATE_YAW_P                      0.18f
#define AC_ATC_HELI_RATE_YAW_I                      0.12f
#define AC_ATC_HELI_RATE_YAW_D                      0.003f
#define AC_ATC_HELI_RATE_YAW_IMAX                   1.0f
#define AC_ATC_HELI_RATE_YAW_FF                     0.024f
#define AC_ATC_HELI_RATE_YAW_FILT_HZ                20.0f

#define AC_ATTITUDE_HELI_ANGLE_LIMIT_THROTTLE_MAX   0.95f    // Heli's use 95% of max collective before limiting frame angle
#define AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE  0.02f
#define AC_ATTITUDE_HELI_RATE_RP_FF_FILTER          10.0f
#define AC_ATTITUDE_HELI_RATE_Y_VFF_FILTER          10.0f
#define AC_ATTITUDE_HELI_HOVER_ROLL_TRIM_DEFAULT    300
#define AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD   ToRad(30.0f)

class AC_AttitudeControl_Heli : public AC_AttitudeControl {
public:
    AC_AttitudeControl_Heli( AP_AHRS_View &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_MotorsHeli& motors,
                        float dt) :
        AC_AttitudeControl(ahrs, aparm, motors, dt),
        _pid_rate_roll(AC_ATC_HELI_RATE_RP_P, AC_ATC_HELI_RATE_RP_I, AC_ATC_HELI_RATE_RP_D, AC_ATC_HELI_RATE_RP_IMAX, AC_ATC_HELI_RATE_RP_FILT_HZ, dt, AC_ATC_HELI_RATE_RP_FF),
        _pid_rate_pitch(AC_ATC_HELI_RATE_RP_P, AC_ATC_HELI_RATE_RP_I, AC_ATC_HELI_RATE_RP_D, AC_ATC_HELI_RATE_RP_IMAX, AC_ATC_HELI_RATE_RP_FILT_HZ, dt, AC_ATC_HELI_RATE_RP_FF),
        _pid_rate_yaw(AC_ATC_HELI_RATE_YAW_P, AC_ATC_HELI_RATE_YAW_I, AC_ATC_HELI_RATE_YAW_D, AC_ATC_HELI_RATE_YAW_IMAX, AC_ATC_HELI_RATE_YAW_FILT_HZ, dt, AC_ATC_HELI_RATE_YAW_FF),
        pitch_feedforward_filter(AC_ATTITUDE_HELI_RATE_RP_FF_FILTER),
        roll_feedforward_filter(AC_ATTITUDE_HELI_RATE_RP_FF_FILTER),
        yaw_velocity_feedforward_filter(AC_ATTITUDE_HELI_RATE_Y_VFF_FILTER)
        {
            AP_Param::setup_object_defaults(this, var_info);

            // initialise flags
            _flags_heli.limit_roll = false;
            _flags_heli.limit_pitch = false;
            _flags_heli.limit_yaw = false;
            _flags_heli.leaky_i = true;
            _flags_heli.flybar_passthrough = false;
            _flags_heli.tail_passthrough = false;
            _flags_heli.do_piro_comp = false;
        }

    // pid accessors
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }

    // passthrough_bf_roll_pitch_rate_yaw - roll and pitch are passed through directly, body-frame rate target for yaw
    void passthrough_bf_roll_pitch_rate_yaw(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf_cds) override;

    // Integrate vehicle rate into _att_error_rot_vec_rad
    void integrate_bf_rate_error_to_angle_errors();

    // subclass non-passthrough too, for external gyro, no flybar
    void input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) override;

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

    // do_piro_comp - controls whether piro-comp is active or not
    void do_piro_comp(bool piro_comp) { _flags_heli.do_piro_comp = piro_comp; }

    // set_hover_roll_scalar - scales Hover Roll Trim parameter. To be used by vehicle code according to vehicle condition.
    void set_hover_roll_trim_scalar(float scalar) override {_hover_roll_trim_scalar = constrain_float(scalar, 0.0f, 1.0f);}

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
    void input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) override;

    // Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
    void input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw) override;
    
    // enable/disable inverted flight
    void set_inverted_flight(bool inverted) override {
        _inverted_flight = inverted;
    }
    
    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    // To-Do: move these limits flags into the heli motors class
    struct AttControlHeliFlags {
        uint8_t limit_roll          :   1;  // 1 if we have requested larger roll angle than swash can physically move
        uint8_t limit_pitch         :   1;  // 1 if we have requested larger pitch angle than swash can physically move
        uint8_t limit_yaw           :   1;  // 1 if we have requested larger yaw angle than tail servo can physically move
        uint8_t leaky_i             :   1;  // 1 if we should use leaky i term for body-frame rate to motor stage
        uint8_t flybar_passthrough  :   1;  // 1 if we should pass through pilots roll & pitch input directly to swash-plate
        uint8_t tail_passthrough    :   1;  // 1 if we should pass through pilots yaw input to tail
        uint8_t do_piro_comp        :   1;  // 1 if we should do pirouette compensation on roll/pitch
    } _flags_heli;

    //
    // body-frame rate controller
    //
	// rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target body-frame rate (in radians/sec) for roll, pitch and yaw
    // outputs are sent directly to motor class
    void rate_bf_to_motor_roll_pitch(const Vector3f &rate_rads, float rate_roll_target_rads, float rate_pitch_target_rads);
    float rate_target_to_motor_yaw(float rate_yaw_actual_rads, float rate_yaw_rads) override;

    //
    // throttle methods
    //
    
    // pass through for roll and pitch
    float _passthrough_roll;
    float _passthrough_pitch;

    // pass through for yaw if tail_passthrough is set
    float _passthrough_yaw;

    // get_roll_trim - angle in centi-degrees to be added to roll angle. Used by helicopter to counter tail rotor thrust in hover
    float get_roll_trim_rad() override { return constrain_float(radians(_hover_roll_trim_scalar * _hover_roll_trim * 0.01f), -radians(10.0f),radians(10.0f));}

    // internal variables
    float _hover_roll_trim_scalar = 0;              // scalar used to suppress Hover Roll Trim


    // This represents an euler axis-angle rotation vector from the vehicles
    // estimated attitude to the reference (setpoint) attitude used in the attitude
    // controller, in radians in the vehicle body frame of reference.
    Vector3f            _att_error_rot_vec_rad;

    // parameters
    AP_Int8         _piro_comp_enabled;             // Flybar present or not.  Affects attitude controller used during ACRO flight mode
    AP_Int16        _hover_roll_trim;               // Angle in centi-degrees used to counter tail rotor thrust in hover
    AC_HELI_PID     _pid_rate_roll;
    AC_HELI_PID     _pid_rate_pitch;
    AC_HELI_PID     _pid_rate_yaw;
    
    // LPF filters to act on Rate Feedforward terms to linearize output.
    // Due to complicated aerodynamic effects, feedforwards acting too fast can lead
    // to jerks on rate change requests.
    LowPassFilterFloat pitch_feedforward_filter;
    LowPassFilterFloat roll_feedforward_filter;
    LowPassFilterFloat yaw_velocity_feedforward_filter;
    LowPassFilterFloat yaw_acceleration_feedforward_filter;

};
