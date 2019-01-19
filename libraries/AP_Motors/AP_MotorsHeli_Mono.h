/// @file	AP_MotorsHeli_Mono.h
/// @brief	Motor control class for monocopters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>

#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"


/// @class      AP_MotorsHeli_Mono
class AP_MotorsHeli_Mono : public AP_MotorsHeli {
public:

    /// Constructor
    AP_MotorsHeli_Mono(AP_AHRS_View &ahrs,
                       uint16_t loop_rate,
                       uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(loop_rate, speed_hz),
        _ahrs(ahrs),
        _rotor(SRV_Channel::k_heli_rsc, CH_1)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // set update rate to motors - a value in hertz
    void set_update_rate( uint16_t speed_hz ) override; 

    // spin a motor at the pwm value specified
    void output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // output_to_motors - sends values out to the motors
    void output_to_motors() override;

    // set_desired_rotor_speed - sets target rotor speed as a number from 0 ~ 1000
    void set_desired_rotor_speed(float desired_speed) override;

    // get_estimated_rotor_speed - gets estimated rotor speed as a number from 0 ~ 1000
    float get_main_rotor_speed() const  override { return _rotor.get_rotor_speed(); }

    // get_desired_rotor_speed - gets target rotor speed as a number from 0 ~ 1000
    float get_desired_rotor_speed() const  override { return _rotor.get_rotor_speed(); }

    // rotor_speed_above_critical - return true if rotor speed is above that critical for flight
    bool rotor_speed_above_critical() const  override { return _rotor.get_rotor_speed() > _rotor.get_critical_speed(); }

    // calculate_scalars - recalculates various scalars used
    void calculate_scalars() override;

    // calculate_armed_scalars - recalculates scalars that can change while armed
    void calculate_armed_scalars() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint16_t get_motor_mask() override;

    // has_flybar - returns true if we have a mechanical flybar
    bool has_flybar() const  override { return true; }

    // supports_yaw_passthrought - returns true - monocopters use yaw pass through to set the virtual yaw
    bool supports_yaw_passthrough() const  override { return true; }

    // servo_test - move servos through full range of movement
    void servo_test() override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // init_outputs
    bool init_outputs () override;

    // update_motor_controls - sends commands to motor controllers
    void update_motor_control(RotorControlState state) override;

    // calculate_roll_pitch_collective_factors - setup rate factors
    void calculate_roll_pitch_collective_factors () override {};

    // move_actuators - moves swash plate to attitude of parameters passed in
    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out)  override;

    // calculated outputs
    float _aileron = 0.0f; // -1..1

    // last time called
    uint64_t _last_update_us;

    //  objects we depend upon
    AP_MotorsHeli_RSC           _rotor;   // main rotor controller
    AP_AHRS_View&               _ahrs;    // ahrs for rotating craft

    // parameters
    AP_Float _phase_offset;
    AP_Float _hover_rpm;
    AP_Int8 _sync_pin;
    AP_Float _sync_phase_offset;

    bool sync_on = false;

};
