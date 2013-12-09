// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsMatrix.h
/// @brief	Motor control class for Matrixcopters

#ifndef __AP_MOTORS_MATRIX_H__
#define __AP_MOTORS_MATRIX_H__

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_Motors_Class.h"

#define AP_MOTORS_MATRIX_YAW_FACTOR_CW   -1
#define AP_MOTORS_MATRIX_YAW_FACTOR_CCW   1

#define AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM    200

/// @class      AP_MotorsMatrix
class AP_MotorsMatrix : public AP_Motors {
public:

    /// Constructor
    AP_MotorsMatrix( RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors(rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz),
        _num_motors(0) {
    };

    // init
    virtual void        Init();

    // set update rate to motors - a value in hertz
    // you must have setup_motors before calling this
    virtual void        set_update_rate( uint16_t speed_hz );

    // set frame orientation (normally + or X)
    virtual void        set_frame_orientation( uint8_t new_orientation );

    // enable - starts allowing signals to be sent to motors
    virtual void        enable();

    // output_test - spin each motor for a moment to allow the user to confirm the motor order and spin direction
    virtual void        output_test();

    // output_min - sends minimum values out to the motors
    virtual void        output_min();

    // add_motor using just position and yaw_factor (or prop direction)
    void                add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order);

    // remove_motor
    void                remove_motor(int8_t motor_num);

    // remove_all_motors - removes all motor definitions
    void                remove_all_motors();

    // setup_motors - configures the motors for a given frame type - should be overwritten by child classes
    virtual void        setup_motors() {
        remove_all_motors();
    };

protected:
    // output - sends commands to the motors
    virtual void        output_armed();
    virtual void        output_disarmed();

    // add_motor using raw roll, pitch, throttle and yaw factors
    void                add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order);

    int8_t              _num_motors; // not a very useful variable as you really need to check the motor_enabled array to see which motors are enabled
    float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)
    uint8_t             _test_order[AP_MOTORS_MAX_NUM_MOTORS];  // order of the motors in the test sequence
};

#endif  // AP_MOTORSMATRIX
