// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsHeli_Dual.h
/// @brief	Motor control class for dual heli (tandem or transverse)

#ifndef __AP_MOTORS_HELI_DUAL_H__
#define __AP_MOTORS_HELI_DUAL_H__

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <RC_Channel.h>

#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"

// servo position defaults
#define AP_MOTORS_HELI_DUAL_SERVO1_POS               -60
#define AP_MOTORS_HELI_DUAL_SERVO2_POS                60
#define AP_MOTORS_HELI_DUAL_SERVO3_POS               180
#define AP_MOTORS_HELI_DUAL_SERVO4_POS               -60
#define AP_MOTORS_HELI_DUAL_SERVO5_POS                60
#define AP_MOTORS_HELI_DUAL_SERVO6_POS               180

// rsc function output channel
#define AP_MOTORS_HELI_DUAL_RSC                      CH_8

// default swash limits
#define AP_MOTORS_HELI_DUAL_SWASH_YAW_MAX            2500

// tandem modes
#define AP_MOTORS_HELI_DUAL_MODE_TANDEM              0       // tandem mode (rotors front and aft)
#define AP_MOTORS_HELI_DUAL_MODE_TRANSVERSE          1       // transverse mode (rotors side by side)

// default differential-collective-pitch scaler
#define AP_MOTORS_HELI_DUAL_DCP_SCALER               1

/// @class      AP_MotorsHeli_Dual
class AP_MotorsHeli_Dual : public AP_MotorsHeli {
public:
    // constructor
    AP_MotorsHeli_Dual(RC_Channel&  rc_roll,
                       RC_Channel&  rc_pitch,
                       RC_Channel&  rc_throttle,
                       RC_Channel&  rc_yaw,
                       RC_Channel&  servo_rsc,
                       RC_Channel&  swash_servo_1,
                       RC_Channel&  swash_servo_2,
                       RC_Channel&  swash_servo_3,
                       RC_Channel&  swash_servo_4,
                       RC_Channel&  swash_servo_5,
                       RC_Channel&  swash_servo_6,
                       uint16_t     loop_rate,
                       uint16_t     speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(rc_roll, rc_pitch, rc_throttle, rc_yaw, loop_rate, speed_hz),
        _servo_1(swash_servo_1),
        _servo_2(swash_servo_2),
        _servo_3(swash_servo_3),
        _servo_4(swash_servo_4),
        _servo_5(swash_servo_5),
        _servo_6(swash_servo_6),
        _rotor(servo_rsc, AP_MOTORS_HELI_DUAL_RSC)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // init
    void        Init();

    // set_update_rate - set update rate to motors 
    void        set_update_rate( uint16_t speed_hz );

    // enable - starts allowing signals to be sent to motors
    void        enable();

    // output_test - spin a motor at the pwm value specified
    void        output_test(uint8_t motor_seq, int16_t pwm);

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    uint16_t    get_motor_mask();

    // set_dt - sets main loop time
    void        set_dt(float dt);

    //
    // heli specific methods
    //

    // allow_arming - checks if we are ready to arm
    bool        allow_arming();

    // has_flybar - returns true if we have a mechical flybar
    bool        has_flybar() const { return AP_MOTORS_HELI_NOFLYBAR; }

    // set_speed_target - sets the desired speed of the rotors
    void        set_speed_target(int16_t speed_target);

    // recalc_scalers - update scalers from parameters
    void        recalc_scalers();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // init_swash
    void        init_swash ();

    // init_servos
    void        init_servos ();

    // reset_swash
    void        reset_swash ();

    // reset_servos
    void        reset_servos ();

    // calculate_swash_factors - calculate factors based on swash type and servo position
    void        calculate_swash_factors ();

    // output_armed_stabilizing
    void        output_armed_stabilizing();

    // output_disarmed
    void        output_disarmed();

    // heli_move_swash - moves swash plate to attitude of parameters passed in
    void        move_swash(int16_t roll_out, int16_t pitch_out, int16_t coll_in, int16_t yaw_out);

private:

    //  objects we depend upon
    RC_Channel&                 _servo_1;           // swash plate servo #1
    RC_Channel&                 _servo_2;           // swash plate servo #2
    RC_Channel&                 _servo_3;           // swash plate servo #3
    RC_Channel&                 _servo_4;           // swash plate servo #4
    RC_Channel&                 _servo_5;           // swash plate servo #5
    RC_Channel&                 _servo_6;           // swash plate servo #6
    AP_MotorsHeli_RSC           _rotor;             // main rotor controller

    // parameters
    AP_Int16        _servo1_pos;                    // angular location of swash servo #1
    AP_Int16        _servo2_pos;                    // angular location of swash servo #2
    AP_Int16        _servo3_pos;                    // angular location of swash servo #3
    AP_Int16        _servo4_pos;                    // angular location of swash servo #4
    AP_Int16        _servo5_pos;                    // angular location of swash servo #5
    AP_Int16        _servo6_pos;                    // angular location of swash servo #6
    AP_Int16        _yaw_max;                       // maximum yaw angle of the swash plate in centi-degrees
    AP_Int16        _swash1_phase_angle;            // phase angle correction for 1st swash.
    AP_Int16        _swash2_phase_angle;            // phase angle correction for 2nd swash.
    AP_Int8         _dual_mode;                     // which dual mode the heli is
    AP_Float        _dcp_scaler;                    // scaling factor applied to the differential-collective-pitch
    AP_Int8         _dcp_yaw_effect;                // feed-forward compensation to automatically add yaw input when differential collective pitch is applied.

    // internal variables
    float           _yaw_factors[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];
    float           _yaw_scaler;                    // scaler to convert pitch input from radio (i.e. -4500 ~ 4500) to max pitch range
};

#endif  // AP_MotorsHeli_Dual
