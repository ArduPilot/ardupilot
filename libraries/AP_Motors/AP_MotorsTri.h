/// @file	AP_MotorsTri.h
/// @brief	Motor control class for Tricopters
#pragma once

#include "AP_Motors_config.h"

#if AP_MOTORS_TRI_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include "AP_MotorsMulticopter.h"

// tail servo uses channel 7
#define AP_MOTORS_CH_TRI_YAW    CH_7

#define AP_MOTORS_TRI_SERVO_RANGE_DEG_MIN   5   // minimum angle movement of tail servo in degrees
#define AP_MOTORS_TRI_SERVO_RANGE_DEG_MAX   80  // maximum angle movement of tail servo in degrees

/// @class      AP_MotorsTri
class AP_MotorsTri : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsTri(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {
    };

    // init
    void                init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz ) override;

    // output_to_motors - sends minimum values out to the motors
    virtual void        output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint32_t            get_motor_mask() override;

    // output a thrust to all motors that match a given motor
    // mask. This is used to control tiltrotor motors in forward
    // flight. Thrust is in the range 0 to 1
    // rudder_dt applys diffential thrust for yaw in the range 0 to 1
    void                output_motor_mask(float thrust, uint32_t mask, float rudder_dt) override;

    // return the roll factor of any motor, this is used for tilt rotors and tail sitters
    // using copter motors for forward flight
    float               get_roll_factor(uint8_t i) override;

    // return the pitch factor of any motor, this is used for AP_Motors_test
    float               get_pitch_factor_json(uint8_t i);

    // Run arming checks
    bool arming_checks(size_t buflen, char *buffer) const override;

    // Get the testing order for the motors, this is used for AP_Motors_test
    uint8_t get_motor_test_order(uint8_t i);

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing() override;

    // call vehicle supplied thrust compensation if set
    void                thrust_compensation(void) override;

    const char* _get_frame_string() const override { return "TRI"; }
    const char*  get_type_string() const override { return _pitch_reversed ? "pitch-reversed" : ""; }

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // parameters

    float           _pivot_angle;                       // Angle of yaw pivot
    float           _thrust_right;
    float           _thrust_rear;
    float           _thrust_left;

    // reverse pitch
    bool _pitch_reversed;
    bool _have_tail_servo;
};

#endif  // AP_MOTORS_TRI_ENABLED
