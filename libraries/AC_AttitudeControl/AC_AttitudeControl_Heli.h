// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl_Heli.h
/// @brief   ArduCopter attitude control library for traditional helicopters

#ifndef AC_ATTITUDECONTROL_HELI_H
#define AC_ATTITUDECONTROL_HELI_H

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsHeli.h>
#include <AC_PID/AC_HELI_PID.h>
#include <Filter/Filter.h>

#define AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE  0.02f
#define AC_ATTITUDE_HELI_RATE_RP_FF_FILTER          10.0f
#define AC_ATTITUDE_HELI_RATE_Y_VFF_FILTER          10.0f
#define AC_ATTITUDE_HELI_RATE_Y_AFF_FILTER          10.0f

class AC_AttitudeControl_Heli : public AC_AttitudeControl {
public:
    AC_AttitudeControl_Heli( AP_AHRS &ahrs,
                        const AP_Vehicle::MultiCopter &aparm,
                        AP_MotorsHeli& motors,
                        AC_P& p_angle_roll, AC_P& p_angle_pitch, AC_P& p_angle_yaw,
                        AC_HELI_PID& pid_rate_roll, AC_HELI_PID& pid_rate_pitch, AC_HELI_PID& pid_rate_yaw
                        ) :
        AC_AttitudeControl(ahrs, aparm, motors,
                           p_angle_roll, p_angle_pitch, p_angle_yaw,
                           pid_rate_roll, pid_rate_pitch, pid_rate_yaw),
        _passthrough_roll(0), _passthrough_pitch(0), _passthrough_yaw(0),
        pitch_feedforward_filter(AC_ATTITUDE_HELI_RATE_RP_FF_FILTER),
        roll_feedforward_filter(AC_ATTITUDE_HELI_RATE_RP_FF_FILTER),
        yaw_velocity_feedforward_filter(AC_ATTITUDE_HELI_RATE_Y_VFF_FILTER),
        yaw_acceleration_feedforward_filter(AC_ATTITUDE_HELI_RATE_Y_AFF_FILTER)
		{
            AP_Param::setup_object_defaults(this, var_info);
		}

    // passthrough_bf_roll_pitch_rate_yaw - roll and pitch are passed through directly, body-frame rate target for yaw
    void passthrough_bf_roll_pitch_rate_yaw(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf);

	// rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
	// should be called at 100hz or more
	virtual void rate_controller_run();

    // get lean angle max for pilot input that prioritises altitude hold over lean angle
    float get_althold_lean_angle_max() const;

	// use_leaky_i - controls whether we use leaky i term for body-frame to motor output stage
	void use_leaky_i(bool leaky_i) {  _flags_heli.leaky_i = leaky_i; }
    
    // use_flybar_passthrough - controls whether we pass-through
    // control inputs to swash-plate and tail
    void use_flybar_passthrough(bool passthrough, bool tail_passthrough) {  
        _flags_heli.flybar_passthrough = passthrough; 
        _flags_heli.tail_passthrough = tail_passthrough; 
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
    } _flags_heli;

    //
    // body-frame rate controller
    //
	// rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target body-frame rate (in centi-degrees/sec) for roll, pitch and yaw
    // outputs are sent directly to motor class
    void rate_bf_to_motor_roll_pitch(float rate_roll_target_cds, float rate_pitch_target_cds);
    virtual float rate_bf_to_motor_yaw(float rate_yaw_cds);

    //
    // throttle methods
    //

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_boosted_throttle(float throttle_in);
    
    // pass through for roll and pitch
    int16_t _passthrough_roll;
    int16_t _passthrough_pitch;

    // pass through for yaw if tail_passthrough is set
    int16_t _passthrough_yaw;
    
    // LPF filters to act on Rate Feedforward terms to linearize output.
    // Due to complicated aerodynamic effects, feedforwards acting too fast can lead
    // to jerks on rate change requests.
    LowPassFilterFloat pitch_feedforward_filter;
    LowPassFilterFloat roll_feedforward_filter;
    LowPassFilterFloat yaw_velocity_feedforward_filter;
    LowPassFilterFloat yaw_acceleration_feedforward_filter;

};

#endif //AC_ATTITUDECONTROL_HELI_H
