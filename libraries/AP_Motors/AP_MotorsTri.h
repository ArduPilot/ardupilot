// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsTri.h
/// @brief	Motor control class for Tricopters

#ifndef __AP_MOTORS_TRI_H__
#define __AP_MOTORS_TRI_H__

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_Motors.h"

// tail servo uses channel 7
#define AP_MOTORS_CH_TRI_YAW    CH_7

/// @class      AP_MotorsTri
class AP_MotorsTri : public AP_Motors {
public:

    /// Constructor
    AP_MotorsTri( RC_Channel& rc_roll, RC_Channel& rc_pitch, RC_Channel& rc_throttle, RC_Channel& rc_yaw, RC_Channel& rc_tail, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors(rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz),
        _rc_tail(rc_tail) {
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
    virtual uint16_t    get_motor_mask();

protected:
    // output - sends commands to the motors
    virtual void        output_armed();
    virtual void        output_disarmed();

    RC_Channel&         _rc_tail;       // REV parameter used from this channel to determine direction of tail servo movement
};

#endif  // AP_MOTORSTRI
