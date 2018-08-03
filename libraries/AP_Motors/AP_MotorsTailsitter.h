/// @file	AP_MotorsTailsitter.h
/// @brief	Motor control class for tailsitters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsMulticopter.h"

/// @class      AP_MotorsTailsitter
class AP_MotorsTailsitter : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsTailsitter(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // init
    void init(motor_frame_class frame_class, motor_frame_type frame_type);

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) {}
    void set_update_rate( uint16_t speed_hz ) {}

    virtual void output_test_seq(uint8_t motor_seq, int16_t pwm) override {}

    // output_to_motors - sends output to named servos
    void output_to_motors();

    // return 0 motor mask
    uint16_t get_motor_mask() { return 0; }

protected:
    // calculate motor outputs
    void output_armed_stabilizing();

    // calculated outputs
    float _aileron;  // -1..1
    float _elevator; // -1..1
    float _rudder;   // -1..1
    float _throttle; // 0..1
};
