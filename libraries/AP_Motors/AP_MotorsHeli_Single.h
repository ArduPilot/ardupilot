// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsHeli_Single.h
/// @brief	Motor control class for traditional heli

#ifndef __AP_MOTORS_HELI_SINGLE_H__
#define __AP_MOTORS_HELI_SINGLE_H__

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <RC_Channel.h>

#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"

// rsc and aux function output channels
#define AP_MOTORS_HELI_SINGLE_RSC                              CH_8
#define AP_MOTORS_HELI_SINGLE_AUX                              CH_7

// servo position defaults
#define AP_MOTORS_HELI_SINGLE_SERVO1_POS                       -60
#define AP_MOTORS_HELI_SINGLE_SERVO2_POS                       60
#define AP_MOTORS_HELI_SINGLE_SERVO3_POS                       180

// swash type definitions
#define AP_MOTORS_HELI_SINGLE_SWASH_CCPM                       0
#define AP_MOTORS_HELI_SINGLE_SWASH_H1                         1

// tail types
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO                   0
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO           1
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH    2
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH  3

// default direct-drive variable pitch speed
#define AP_MOTORS_HELI_SINGLE_TAIL_DIRECTDRIVE_SPEED_DEFAULT   500

// default external gyro gain
#define AP_MOTORS_HELI_SINGLE_EXT_GYRO_GAIN                    350

/// @class      AP_MotorsHeli_Single
class AP_MotorsHeli_Single : public AP_MotorsHeli {
public:
    // constructor
    AP_MotorsHeli_Single(RC_Channel&    rc_roll,
                         RC_Channel&    rc_pitch,
                         RC_Channel&    rc_throttle,
                         RC_Channel&    rc_yaw,
                         RC_Channel&    servo_aux,
                         RC_Channel&    servo_rsc,
                         RC_Channel&    servo_1,
                         RC_Channel&    servo_2,
                         RC_Channel&    servo_3,
                         RC_Channel&    servo_yaw,
                         uint16_t       loop_rate,
                         uint16_t       speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(rc_roll, rc_pitch, rc_throttle, rc_yaw, loop_rate, speed_hz),
        _servo_aux(servo_aux),
        _servo_1(servo_1),
        _servo_2(servo_2),
        _servo_3(servo_3),
        _servo_yaw(servo_yaw),
        _delta_phase_angle(0),
        _main_rotor(servo_rsc, AP_MOTORS_HELI_SINGLE_RSC),
        _tail_rotor(servo_aux, AP_MOTORS_HELI_SINGLE_AUX)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // init
    void            Init();

    // set update rate to motors - a value in hertz
    void            set_update_rate( uint16_t speed_hz );

    // enable - starts allowing signals to be sent to motors
    void            enable();

    // output_test - spin a motor at the pwm value specified
    void            output_test(uint8_t motor_seq, int16_t pwm);

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    uint16_t        get_motor_mask();

    // set_dt - sets main loop time
    void            set_dt(float dt);

    //
    // heli specific methods
    //

    // allow_arming - checks if we are ready to arm
    bool            allow_arming();

    // has_flybar - returns true if we have a mechical flybar
    bool            has_flybar() const { return _flybar_mode; }

    // set_speed_target - sets the desired speed of the rotors
    void            set_speed_target(int16_t speed_target);

    // recalc_scalers - update scalers from parameters
    void            recalc_scalers();

    //
    // single heli specific methods
    //

    // get_ext_gyro_gain - gets external gyro gain as a pwm (1000~2000)
    int16_t         get_ext_gyro_gain() const { return _ext_gyro_gain; }

    // set_ext_gyro_gain - gets external gyro gain as a pwm (1000~2000)
    void            set_ext_gyro_gain(int16_t ext_gyro_gain) { _ext_gyro_gain = ext_gyro_gain; }

    // var_info
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // init_servos
    void            init_servos ();

    // reset_servos
    void            reset_servos ();

    // calculate_swash_factors - calculate factors based on swash type and servo position
    void            calculate_swash_factors ();

    // move_swash - moves swash plate to attitude of parameters passed in
    void            move_swash(int16_t roll_out, int16_t pitch_out, int16_t coll_in, int16_t yaw_out);

    // output_armed_stabilizing
    virtual void    output_armed_stabilizing();

    // output_disarmed
    virtual void    output_disarmed();

    // output_yaw
    virtual void    output_yaw(int16_t yaw_out);

    // objects we depend upon
    RC_Channel&         _servo_aux;                 // output to ext gyro gain and tail direct drive esc (ch7)
    RC_Channel&         _servo_1;                   // swash plate servo #1
    RC_Channel&         _servo_2;                   // swash plate servo #2
    RC_Channel&         _servo_3;                   // swash plate servo #3
    RC_Channel&         _servo_yaw;                 // tail servo
    AP_MotorsHeli_RSC   _main_rotor;                // main rotor
    AP_MotorsHeli_RSC   _tail_rotor;                // tail rotor

    // parameters
    AP_Int16            _tail_type;                 // Tail type used: Servo, Servo with external gyro, direct drive variable pitch or direct drive fixed pitch
    AP_Int16            _servo1_pos;                // Angular location of swash servo #1
    AP_Int16            _servo2_pos;                // Angular location of swash servo #2
    AP_Int16            _servo3_pos;                // Angular location of swash servo #3
    AP_Int8             _swash_type;                // Swash Type Setting - either 3-servo CCPM or H1 Mechanical Mixing
    AP_Int16            _ext_gyro_gain;             // PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro
    AP_Int16            _phase_angle;               // Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
    AP_Int8             _flybar_mode;               // Flybar present or not.  Affects attitude controller used during ACRO flight mode
    AP_Int16            _tail_direct_drive_speed;   // Direct Drive VarPitch Tail ESC speed (0 ~ 1000)

private:

    // internal variables
    int16_t             _delta_phase_angle;         // phase angle dynamic compensation

    // write_aux - outputs pwm onto output aux channel (ch7). servo_out parameter is of the range 0 ~ 1000
    void                write_aux(int16_t servo_out);
};

#endif  // __AP_MOTORS_HELI_SINGLE_H__
