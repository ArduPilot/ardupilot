// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_MotorsMatrix.h
/// @brief	Motor control class for Matrixcopters

#ifndef AP_MOTORSMATRIX
#define AP_MOTORSMATRIX

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <AP_Motors.h>

#define AP_MOTORS_MATRIX_MOTOR_UNDEFINED -1
#define AP_MOTORS_MATRIX_ORDER_UNDEFINED -1

#define AP_MOTORS_MATRIX_MOTOR_CW -1
#define AP_MOTORS_MATRIX_MOTOR_CCW 1

/// @class      AP_MotorsMatrix
class AP_MotorsMatrix : public AP_Motors {
public:

    /// Constructor
    AP_MotorsMatrix( uint8_t APM_version, APM_RC_Class* rc_out, RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors(APM_version, rc_out, rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz),
        instant_pwm_force01(false),
        instant_pwm_force23(false),
        instant_pwm_force67(false),
        _num_motors(0) {
    };

    // init
    virtual void        Init();

    // set update rate to motors - a value in hertz or AP_MOTORS_SPEED_INSTANT_PWM for instant pwm
    // you must have setup_motors before calling this
    virtual void            set_update_rate( uint16_t speed_hz );

    // set frame orientation (normally + or X)
    virtual void            set_frame_orientation( uint8_t new_orientation );

    // enable - starts allowing signals to be sent to motors
    virtual void            enable();

    // get basic information about the platform
    virtual uint8_t         get_num_motors() {
        return _num_motors;
    };

    // motor test
    virtual void        output_test();

    // output_min - sends minimum values out to the motors
    virtual void        output_min();

    // add_motor using just position and prop direction
    virtual void        add_motor(int8_t motor_num, float angle_degrees, int8_t direction, int8_t opposite_motor_num = AP_MOTORS_MATRIX_MOTOR_UNDEFINED, int8_t testing_order = AP_MOTORS_MATRIX_ORDER_UNDEFINED);

    // remove_motor
    virtual void        remove_motor(int8_t motor_num);

    // remove_all_motors - removes all motor definitions
    virtual void        remove_all_motors();

    // setup_motors - configures the motors for a given frame type - should be overwritten by child classes
    virtual void        setup_motors() {
        remove_all_motors();
    };

    // matrix
    AP_Int8         opposite_motor[AP_MOTORS_MAX_NUM_MOTORS];           // motor number of the opposite motor
    AP_Int8         test_order[AP_MOTORS_MAX_NUM_MOTORS];               // order of the motors in the test sequence

    // used for instant_pwm only
    bool            instant_pwm_force01;
    bool            instant_pwm_force23;
    bool            instant_pwm_force67;

protected:
    // output - sends commands to the motors
    virtual void        output_armed();
    virtual void        output_disarmed();

    // add_motor using raw roll, pitch, throttle and yaw factors
    virtual void        add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, int8_t opposite_motor_num = AP_MOTORS_MATRIX_MOTOR_UNDEFINED, int8_t testing_order = AP_MOTORS_MATRIX_ORDER_UNDEFINED);

    int8_t              _num_motors; // not a very useful variable as you really need to check the motor_enabled array to see which motors are enabled
    float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)
};

#endif  // AP_MOTORSMATRIX