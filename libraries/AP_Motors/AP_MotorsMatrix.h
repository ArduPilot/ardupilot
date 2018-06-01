// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsMatrix.h
/// @brief	Motor control class for Matrixcopters

#ifndef __AP_MOTORS_MATRIX_H__
#define __AP_MOTORS_MATRIX_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMulticopter.h"

#define AP_MOTORS_MATRIX_YAW_FACTOR_CW   -1
#define AP_MOTORS_MATRIX_YAW_FACTOR_CCW   1

/// @class      AP_MotorsMatrix
class AP_MotorsMatrix : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsMatrix(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(loop_rate, speed_hz)
    {};

    // init
    virtual void        Init();

    // set update rate to motors - a value in hertz
    // you must have setup_motors before calling this
    virtual void        set_update_rate( uint16_t speed_hz );

    // set frame orientation (normally + or X)
    virtual void        set_frame_orientation( uint8_t new_orientation );

    // enable - starts allowing signals to be sent to motors
    virtual void        enable();

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test(uint8_t motor_seq, int16_t pwm);

    // output_min - sends minimum values out to the motors
    virtual void        output_min();

    // add_motor using just position and yaw_factor (or prop direction)
    void                add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order);

    // add_motor using separate roll and pitch factors (for asymmetrical frames) and prop direction
    void                add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order);

    // remove_motor
    void                remove_motor(int8_t motor_num);

    // remove_all_motors - removes all motor definitions
    void                remove_all_motors();

    // setup_motors - configures the motors for a given frame type - should be overwritten by child classes
    virtual void        setup_motors() {
        remove_all_motors();
    };

    // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t    get_motor_mask();

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing();
    void                output_armed_not_stabilizing();
    void                output_disarmed();

    // add_motor using raw roll, pitch, throttle and yaw factors
    void                add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order);

    float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)
    uint8_t             _test_order[AP_MOTORS_MAX_NUM_MOTORS];  // order of the motors in the test sequence
};

#endif  // AP_MOTORSMATRIX
