// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsHeliSingle.h
/// @brief	Motor control class for Traditional Heli

#ifndef __AP_MOTORS_HELI_SINGLE_H__
#define __AP_MOTORS_HELI_SINGLE_H__

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_MotorsHeli.h"

// maximum number of swashplate servos
#define AP_MOTORS_HELI_SINGLE_NUM_SWASHPLATE_SERVOS    3

// TradHeli Aux Function Output Channels
#define AP_MOTORS_HELI_SINGLE_AUX                      CH_7

// servo position defaults
#define AP_MOTORS_HELI_SINGLE_SERVO1_POS               -60
#define AP_MOTORS_HELI_SINGLE_SERVO2_POS                60
#define AP_MOTORS_HELI_SINGLE_SERVO3_POS               180

// swash type definitions
#define AP_MOTORS_HELI_SINGLE_SWASH_CCPM               0
#define AP_MOTORS_HELI_SINGLE_SWASH_H1                 1

// tail types
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO                   0
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO           1
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH    2
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH  3

// default external gyro gain (ch7 out)
#define AP_MOTORS_HELI_SINGLE_EXT_GYRO_GAIN            350

// minimum outputs and ram incrementor direct drive motors
#define AP_MOTORS_HELI_SINGLE_DDTAIL_DEFAULT       500
#define AP_MOTORS_HELI_SINGLE_TAIL_RAMP_INCREMENT      5       // 5 is 2 seconds for direct drive tail rotor to reach to full speed (5 = (2sec*100hz)/1000)

/// @class      AP_MotorsHeliSingle
class AP_MotorsHeliSingle : public AP_MotorsHeli {
public:

    /// Constructor
    AP_MotorsHeliSingle(RC_Channel&      rc_roll,
                        RC_Channel&      rc_pitch,
                        RC_Channel&      rc_throttle,
                        RC_Channel&      rc_yaw,
                        RC_Channel&      servo_aux,
                        RC_Channel&      servo_rsc,
                        RC_Channel&      servo_1,
                        RC_Channel&      servo_2,
                        RC_Channel&      servo_3,
                        RC_Channel&      yaw_servo,
                        uint16_t         speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(rc_roll, rc_pitch, rc_throttle, rc_yaw, servo_rsc, speed_hz),
        _servo_aux(servo_aux),
        _servo_1(servo_1),
        _servo_2(servo_2),
        _servo_3(servo_3),
        _servo_4(yaw_servo),
        _tail_direct_drive_out(0),
        _delta_phase_angle(0)
    {
		AP_Param::setup_object_defaults(this, var_info);

        // initialise flags
        _heliflags.swash_initialised = 0;
        _heliflags.landing_collective = 0;
        _heliflags.motor_runup_complete = 0;
    };

    // init
    void Init();

    // set update rate to motors - a value in hertz
    // you must have setup_motors before calling this
    void set_update_rate( uint16_t speed_hz );

    // enable - starts allowing signals to be sent to motors
    void enable();

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    void output_test(uint8_t motor_seq, int16_t pwm);

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint16_t get_motor_mask();

    //
    // heli specific methods
    //

    // _tail_type - returns the tail type (servo, servo with ext gyro, direct drive var pitch, direct drive fixed pitch)
    int16_t tail_type() const { return _tail_type; }

    // ext_gyro_gain - gets and sets external gyro gain as a pwm (1000~2000)
    int16_t ext_gyro_gain() const { return _ext_gyro_gain; }
    void ext_gyro_gain(int16_t pwm) { _ext_gyro_gain = pwm; }

    // has_flybar - returns true if we have a mechical flybar
    bool has_flybar() const { return _flybar_mode; }

    // get_phase_angle - returns phase angle
    int16_t get_phase_angle() const { return _phase_angle; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];
    
    // set_delta_phase_angle for setting variable phase angle compensation and force
    // recalculation of collective factors
    void set_delta_phase_angle(int16_t angle);

protected:

protected:

    // init_servos
    void init_servos ();

    // reset_servos
    void reset_servos ();

    // calculate_swash_factors - calculate factors based on swash type and servo position
    void calculate_swash_factors ();

    // heli_move_swash - moves swash plate to attitude of parameters passed in
    void move_swash(int16_t roll_out, int16_t pitch_out, int16_t coll_in, int16_t yaw_out);

private:

    void rsc_control ();
    
    // tail_ramp - ramps tail motor towards target.  Only used for direct drive variable pitch tails
    // results put into _tail_direct_drive_out and sent to ESC
    void tail_ramp(int16_t tail_target);

    // return true if the tail rotor is up to speed
    bool tail_rotor_runup_complete();

    // write_aux - outputs pwm onto output aux channel (ch7). servo_out parameter is of the range 0 ~ 1000
    void write_aux(int16_t servo_out);

    // external objects we depend upon
    RC_Channel&     _servo_aux;                 // output to ext gyro gain and tail direct drive esc (ch7)
    RC_Channel&     _servo_1;                   // swash plate servo #1
    RC_Channel&     _servo_2;                   // swash plate servo #2
    RC_Channel&     _servo_3;                   // swash plate servo #3
    RC_Channel&     _servo_4;                   // tail servo

    // parameters
    AP_Int16        _servo1_pos;                // Angular location of swash servo #1
    AP_Int16        _servo2_pos;                // Angular location of swash servo #2
    AP_Int16        _servo3_pos;                // Angular location of swash servo #3
    AP_Int16        _tail_type;                 // Tail type used: Servo, Servo with external gyro, direct drive variable pitch or direct drive fixed pitch
    AP_Int8         _swash_type;                // Swash Type Setting - either 3-servo CCPM or H1 Mechanical Mixing
    AP_Int16        _ext_gyro_gain;             // PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro
    AP_Int16        _phase_angle;               // Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
    AP_Int8         _flybar_mode;               // Flybar present or not.  Affects attitude controller used during ACRO flight mode
    AP_Int16        _direct_drive_tailspeed;    // Direct Drive VarPitch Tail ESC speed (0 ~ 1000)

    // internal variables
    int16_t         _tail_direct_drive_out;     // current ramped speed of output on ch7 when using direct drive variable pitch tail type
    int16_t         _delta_phase_angle;         // phase angle dynamic compensation
};

#endif  // __AP_MOTORS_HELI_SINGLE_H__
