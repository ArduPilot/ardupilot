/// @file	AP_MotorsTri.h
/// @brief	Motor control class for Tricopters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsMulticopter.h"

// tail servo uses channel 7
#define AP_MOTORS_CH_TRI_YAW    CH_7

#define AP_MOTORS_TRI_SERVO_RANGE_DEG_MIN   5   // minimum angle movement of tail servo in degrees
#define AP_MOTORS_TRI_SERVO_RANGE_DEG_MAX   80  // maximum angle movement of tail servo in degrees

/// @class      AP_MotorsTri
class AP_MotorsTri : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsTri(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(loop_rate, speed_hz)
    {
    };

    // init
    void                init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz ) override;

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // output_to_motors - sends minimum values out to the motors
    virtual void        output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint16_t            get_motor_mask() override;

    // output a thrust to all motors that match a given motor
    // mask. This is used to control tiltrotor motors in forward
    // flight. Thrust is in the range 0 to 1
    // rudder_dt applys diffential thrust for yaw in the range 0 to 1
    void                output_motor_mask(float thrust, uint8_t mask, float rudder_dt) override;

    // return the roll factor of any motor, this is used for tilt rotors and tail sitters
    // using copter motors for forward flight
    float               get_roll_factor(uint8_t i) override;

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing() override;

    // call vehicle supplied thrust compensation if set
    void                thrust_compensation(void) override;
    
    // calc_yaw_radio_output - calculate final radio output for yaw channel
    int16_t             calc_yaw_radio_output(float yaw_input, float yaw_input_max);        // calculate radio output for yaw servo, typically in range of 1100-1900

    // parameters

    SRV_Channel     *_yaw_servo; // yaw output channel
    float           _pivot_angle;                       // Angle of yaw pivot
    float           _thrust_right;
    float           _thrust_rear;
    float           _thrust_left;
};
