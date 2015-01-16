// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsHeliTandem.h
/// @brief	Motor control class for Tandem Heli

#ifndef __AP_MOTORS_HELI_TANDEM_H__
#define __AP_MOTORS_HELI_TANDEM_H__

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_MotorsHeli.h"

// maximum number of swashplate servos
#define AP_MOTORS_HELI_TANDEM_NUM_SWASHPLATE_SERVOS    6

// servo position defaults
#define AP_MOTORS_HELI_TANDEM_SERVO1_POS               -60
#define AP_MOTORS_HELI_TANDEM_SERVO2_POS                60
#define AP_MOTORS_HELI_TANDEM_SERVO3_POS               180
#define AP_MOTORS_HELI_TANDEM_SERVO4_POS               -60
#define AP_MOTORS_HELI_TANDEM_SERVO5_POS                60
#define AP_MOTORS_HELI_TANDEM_SERVO6_POS               180

// default swash limits
#define AP_MOTORS_HELI_TANDEM_SWASH_YAW_MAX            2500

// tandem mode
#define AP_MOTORS_HELI_TANDEM_MODE_LONGITUDINAL        0       // tandem mode (rotors front and aft)
#define AP_MOTORS_HELI_TANDEM_MODE_TRANSVERSE          1       // transverse mode (rotors side by side)

// differential-collective-pitch
#define AP_MOTORS_HELI_TANDEM_DCP_SCALER               1

/// @class      AP_MotorsHeliTandem
class AP_MotorsHeliTandem : public AP_MotorsHeli {
public:

    /// Constructor
    AP_MotorsHeliTandem(RC_Channel&      rc_roll,
                        RC_Channel&      rc_pitch,
                        RC_Channel&      rc_throttle,
                        RC_Channel&      rc_yaw,
                        RC_Channel&      servo_rsc,
                        RC_Channel&      swash_servo_1,
                        RC_Channel&      swash_servo_2,
                        RC_Channel&      swash_servo_3,
                        RC_Channel&      swash_servo_4,
                        RC_Channel&      swash_servo_5,
                        RC_Channel&      swash_servo_6,
                        uint16_t         speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(rc_roll, rc_pitch, rc_throttle, rc_yaw, servo_rsc, speed_hz),
        _servo_1(swash_servo_1),
        _servo_2(swash_servo_2),
        _servo_3(swash_servo_3),
        _servo_4(swash_servo_4),
        _servo_5(swash_servo_5),
        _servo_6(swash_servo_6)
    {
		AP_Param::setup_object_defaults(this, var_info);
    };

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

    // has_flybar - returns true if we have a mechical flybar
    bool has_flybar() const { return AP_MOTORS_HELI_NOFLYBAR; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];
protected:
    // init_swash
    void init_swash ();
    
    // init_servos
    void init_servos ();

    // reset_swash
    void reset_swash ();

    // reset_servos
    void reset_servos ();

    // calculate_swash_factors - calculate factors based on swash type and servo position
    void calculate_swash_factors ();

    // heli_move_swash - moves swash plate to attitude of parameters passed in
    void move_swash(int16_t roll_out, int16_t pitch_out, int16_t coll_in, int16_t yaw_out);

private:

    // external objects we depend upon
    RC_Channel&     _servo_1;                   // swash plate servo #1
    RC_Channel&     _servo_2;                   // swash plate servo #2
    RC_Channel&     _servo_3;                   // swash plate servo #3
    RC_Channel&     _servo_4;                   // swash plate servo #4
    RC_Channel&     _servo_5;                   // swash plate servo #5
    RC_Channel&     _servo_6;                   // swash plate servo #6

    // parameters
    AP_Int16        _servo1_pos;                // Angular location of swash servo #1
    AP_Int16        _servo2_pos;                // Angular location of swash servo #2
    AP_Int16        _servo3_pos;                // Angular location of swash servo #3
    AP_Int16        _servo4_pos;                // Angular location of swash servo #4
    AP_Int16        _servo5_pos;                // Angular location of swash servo #5
    AP_Int16        _servo6_pos;                // Angular location of swash servo #6
    AP_Int16        _yaw_max;                   // Maximum yaw angle of the swash plate in centi-degrees
    AP_Int16        _swash1_phase_angle;        // Phase angle correction for 1st swash.
    AP_Int16        _swash2_phase_angle;        // Phase angle correction for 2nd swash.
    AP_Int8         _tandem_mode;               // Which tandem mode the heli is
    AP_Float        _dcp_scaler;                // Scaling factor applied to the differential-collective-pitch  
    AP_Int8         _dcp_yaw_effect;            // Feed-forward compensation to automatically add yaw input when differential collective pitch is applied.

    // internal variables
    float           _yaw_factors[AP_MOTORS_HELI_MAX_NUM_SWASHPLATE_SERVOS];
    float           _yaw_scaler;                // scaler to convert pitch input from radio (i.e. -4500 ~ 4500) to max pitch range
    int16_t         _swash1_yaw_mode_factor;
    int16_t         _swash2_yaw_mode_factor;
};

#endif  // AP_MotorsHeliTandem
