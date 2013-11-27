// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsSingle.h
/// @brief	Motor and Servo control class for Singlecopters

#ifndef __AP_MOTORS_SING_H__
#define __AP_MOTORS_SING_H__

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_Motors.h"

// feedback direction
#define AP_MOTORS_SING_POSITIVE      1
#define AP_MOTORS_SING_NEGATIVE     -1

#define AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS 250 // update rate for digital servos
#define AP_MOTORS_SINGLE_SPEED_ANALOG_SERVOS 125  // update rate for analog servos

/// @class      AP_MotorsSingle
class AP_MotorsSingle : public AP_Motors {
public:

    /// Constructor
    AP_MotorsSingle( RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, RC_Channel* servo1, RC_Channel* servo2, RC_Channel* servo3, RC_Channel* servo4, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors(rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz),
        _servo1(servo1),
        _servo2(servo2),
        _servo3(servo3),
        _servo4(servo4)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // init
    virtual void        Init();

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz );

    // enable - starts allowing signals to be sent to motors
    virtual void        enable();

    // output_test - spin each motor for a moment to allow the user to confirm the motor order and spin direction
    virtual void        output_test();

    // output_min - sends minimum values out to the motors
    virtual void        output_min();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // output - sends commands to the motors
    virtual void        output_armed();
    virtual void        output_disarmed();

    AP_Int8             _rev_roll;      // REV Roll feedback
    AP_Int8             _rev_pitch;     // REV pitch feedback
    AP_Int8             _rev_yaw;       // REV yaw feedback
    AP_Int16            _servo_speed;   // servo speed
    RC_Channel*         _servo1;
    RC_Channel*         _servo2;
    RC_Channel*         _servo3;
    RC_Channel*         _servo4;
};

#endif  // AP_MOTORSSINGLE
