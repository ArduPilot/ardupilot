// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsCoax.h
/// @brief	Motor and Servo control class for Co-axial helicopters with two motors and two flaps

#ifndef __AP_MOTORS_COAX_H__
#define __AP_MOTORS_COAX_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMulticopter.h"

// feedback direction
#define AP_MOTORS_COAX_POSITIVE      1
#define AP_MOTORS_COAX_NEGATIVE     -1

#define AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS 250 // update rate for digital servos
#define AP_MOTORS_SINGLE_SPEED_ANALOG_SERVOS 125  // update rate for analog servos

#define AP_MOTORS_COAX_SERVO_INPUT_RANGE    4500    // roll or pitch input of -4500 will cause servos to their minimum (i.e. radio_min), +4500 will move them to their maximum (i.e. radio_max)

/// @class      AP_MotorsSingle
class AP_MotorsCoax : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsCoax(RC_Channel& servo1, RC_Channel& servo2, uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(loop_rate, speed_hz),
        _servo1(servo1),
        _servo2(servo2)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // init
    virtual void        Init();

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz );

    // enable - starts allowing signals to be sent to motors
    virtual void        enable();

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test(uint8_t motor_seq, int16_t pwm);

    // output_min - sends minimum values out to the motors
    virtual void        output_min();

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    //  for coax copter, output channels 1 to 4 are used
    virtual uint16_t    get_motor_mask() { return 0x000F; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing();
    void                output_armed_not_stabilizing();
    void                output_disarmed();

    AP_Int8             _rev_roll;      // REV Roll feedback
    AP_Int8             _rev_pitch;     // REV pitch feedback
    AP_Int8             _rev_yaw;       // REV yaw feedback
    AP_Int16            _servo_speed;   // servo speed
    RC_Channel&         _servo1;
    RC_Channel&         _servo2;
};

#endif  // AP_MOTORSCOAX
