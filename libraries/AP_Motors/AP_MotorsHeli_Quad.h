/// @file   AP_MotorsHeli_Quad.h
/// @brief  Motor control class collective pitch quad helicopter (such as stingray)

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>

#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"

// rsc function output channel
#define AP_MOTORS_HELI_QUAD_RSC                     CH_8

// default collective min, max and midpoints for the rear swashplate
#define AP_MOTORS_HELI_QUAD_COLLECTIVE_MIN 1100
#define AP_MOTORS_HELI_QUAD_COLLECTIVE_MAX 1900

#define AP_MOTORS_HELI_QUAD_NUM_MOTORS 4

class AP_MotorsHeli_Quad : public AP_MotorsHeli {
public:
    // constructor
    AP_MotorsHeli_Quad(uint16_t loop_rate,
                       uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(loop_rate, speed_hz),
        _rotor(SRV_Channel::k_heli_rsc, AP_MOTORS_HELI_QUAD_RSC)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // set_update_rate - set update rate to motors
    void set_update_rate( uint16_t speed_hz ) override;

    // output_test_seq - spin a motor at the pwm value specified
    virtual void output_test_seq(uint8_t motor_seq, int16_t pwm) override;

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
    uint16_t get_motor_mask() override;

    // has_flybar - returns true if we have a mechanical flybar
    bool has_flybar() const  override { return AP_MOTORS_HELI_NOFLYBAR; }

    // supports_yaw_passthrought - returns true if we support yaw passthrough
    bool supports_yaw_passthrough() const  override { return false; }

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
    void calculate_roll_pitch_collective_factors () override;

    // move_actuators - moves swash plate to attitude of parameters passed in
    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out)  override;

    //  objects we depend upon
    AP_MotorsHeli_RSC           _rotor;             // main rotor controller

    // rate factors
    float _rollFactor[AP_MOTORS_HELI_QUAD_NUM_MOTORS];
    float _pitchFactor[AP_MOTORS_HELI_QUAD_NUM_MOTORS];
    float _collectiveFactor[AP_MOTORS_HELI_QUAD_NUM_MOTORS];
    float _yawFactor[AP_MOTORS_HELI_QUAD_NUM_MOTORS];
    float _out[AP_MOTORS_HELI_QUAD_NUM_MOTORS];
};


