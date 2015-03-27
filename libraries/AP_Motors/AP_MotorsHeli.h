// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsHeli.h
/// @brief	Motor control class for Traditional Heli

#ifndef __AP_MOTORS_HELI_H__
#define __AP_MOTORS_HELI_H__

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <RC_Channel.h>

#include "AP_Motors_Class.h"

// maximum number of swashplate servos
#define AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS    6

// servo output rates
#define AP_MOTORS_HELI_SPEED_DEFAULT            125     // default servo update rate for helicopters
#define AP_MOTORS_HELI_SPEED_DIGITAL_SERVOS     125     // update rate for digital servos
#define AP_MOTORS_HELI_SPEED_ANALOG_SERVOS      125     // update rate for analog servos

// default swash min and max angles and positions
#define AP_MOTORS_HELI_SWASH_ROLL_MAX           2500
#define AP_MOTORS_HELI_SWASH_PITCH_MAX          2500
#define AP_MOTORS_HELI_COLLECTIVE_MIN           1250
#define AP_MOTORS_HELI_COLLECTIVE_MAX           1750
#define AP_MOTORS_HELI_COLLECTIVE_MID           1500

// swash min and max position while in stabilize mode (as a number from 0 ~ 100)
#define AP_MOTORS_HELI_MANUAL_COLLECTIVE_MIN    0
#define AP_MOTORS_HELI_MANUAL_COLLECTIVE_MAX    100

// swash min while landed or landing (as a number from 0 ~ 1000
#define AP_MOTORS_HELI_LAND_COLLECTIVE_MIN      0

// main rotor speed control types (ch8 out)
#define AP_MOTORS_HELI_RSC_MODE_NONE            0       // main rotor ESC is directly connected to receiver, pilot controls ESC speed through transmitter directly
#define AP_MOTORS_HELI_RSC_MODE_CH8_PASSTHROUGH 1       // main rotor ESC is connected to RC8 (out), pilot desired rotor speed provided by CH8 input
#define AP_MOTORS_HELI_RSC_MODE_SETPOINT        2       // main rotor ESC is connected to RC8 (out), desired speed is held in RSC_SETPOINT parameter

// default main rotor speed (ch8 out) as a number from 0 ~ 1000
#define AP_MOTORS_HELI_RSC_SETPOINT             500

// default rotor ramp up time in seconds
#define AP_MOTORS_HELI_RSC_RAMP_TIME            1       // 1 second to ramp output to main rotor ESC to full power (most people use exterrnal govenors so we can ramp up quickly)
#define AP_MOTORS_HELI_RSC_RUNUP_TIME           10      // 10 seconds for rotor to reach full speed

// flybar types
#define AP_MOTORS_HELI_NOFLYBAR                 0
#define AP_MOTORS_HELI_FLYBAR                   1

/// @class      AP_MotorsHeli
class AP_MotorsHeli : public AP_Motors {
public:
    // constructor
    AP_MotorsHeli(RC_Channel&   rc_roll,
                  RC_Channel&   rc_pitch,
                  RC_Channel&   rc_throttle,
                  RC_Channel&   rc_yaw,
                  uint16_t      loop_rate,
                  uint16_t      speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_Motors(rc_roll, rc_pitch, rc_throttle, rc_yaw, loop_rate, speed_hz),
        _roll_scaler(1),
        _pitch_scaler(1),
        _collective_scaler(1),
        _collective_scaler_manual(1),
        _collective_out(0),
        _collective_mid_pwm(0)
    {
        AP_Param::setup_object_defaults(this, var_info);
        
        // initialise flags
        _heliflags.swash_initialised = 0;
        _heliflags.landing_collective = 0;
        _heliflags.motor_runup_complete = 0;
    };

    // init
    virtual void        Init();

    // set update rate to motors - a value in hertz
    // you must have setup_motors before calling this
    virtual void        set_update_rate( uint16_t speed_hz ) = 0;

    // enable - starts allowing signals to be sent to motors
    virtual void        enable() = 0;

    // output_min - sends minimum values out to the motors
    void                output_min();

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test(uint8_t motor_seq, int16_t pwm) = 0;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t    get_motor_mask() = 0;

    // set_dt for setting main loop rate time
    virtual void        set_dt(float dt) = 0;

    //
    // heli specific methods
    //

    // allow_arming - returns true if main rotor is spinning and it is ok to arm
    virtual bool        allow_arming() = 0;

    // has_flybar - returns true if we have a mechical flybar
    virtual bool        has_flybar() const = 0;

    // get_collective_mid - returns collective mid position as a number from 0 ~ 1000
    int16_t             get_collective_mid() const { return _collective_mid; }

    // get_collective_out - returns collective position from last output as a number from 0 ~ 1000
    int16_t             get_collective_out() const { return _collective_out; }

    // set_collective_for_landing - limits collective from going too low if we know we are landed
    void                set_collective_for_landing(bool landing) { _heliflags.landing_collective = landing; }

    // get_rsc_mode - gets the rotor speed control method (AP_MOTORS_HELI_RSC_MODE_NONE, AP_MOTORS_HELI_RSC_MODE_CH8_PASSTHROUGH or AP_MOTORS_HELI_RSC_MODE_SETPOINT)
    uint8_t             get_rsc_mode() const { return _rsc_mode; }

    // get_rsc_setpoint - gets contents of _rsc_setpoint parameter (0~1000)
    int16_t             get_rsc_setpoint() const { return _rsc_setpoint; }

    // set_speed_target - sets target rotor speed as a number from 0 ~ 1000
    virtual void        set_speed_target(int16_t speed_target) = 0;

    // motor_runup_complete - return true if the main rotor is up to speed
    bool                motor_runup_complete() const;

    // recalc_scalers - recalculates various scalers used. Should be called at about 1hz to allow users to see effect of changing parameters
    virtual void        recalc_scalers();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // output - sends commands to the motors
    virtual void        output_armed_stabilizing();
    void                output_armed_not_stabilizing();
    virtual void        output_disarmed();

private:

    // heli_move_swash - moves swash plate to attitude of parameters passed in
    virtual void        move_swash(int16_t roll_out, int16_t pitch_out, int16_t coll_in, int16_t yaw_out) = 0;

    // reset_swash - free up swash for maximum movements. Used for set-up
    virtual void        reset_swash();

    // reset_servos - should be implemented to reset all the swash servos
    virtual void        reset_servos() = 0;

    // reset_swash_servo - resets the range of a swash servo
    void                reset_swash_servo (RC_Channel& servo);

    // init_swash - initialise the swash plate
    virtual void        init_swash();

    // init_swash_servo - initializes a swash servo
    void                init_swash_servo (RC_Channel& servo);

    // init_servos - should be implemented to initialize all the servos
    virtual void        init_servos() = 0;

    // calculate_swash_factors - calculate factors based on swash type and servo position
    virtual void        calculate_swash_factors() = 0;

    // flags bitmask
    struct heliflags_type {
        uint8_t swash_initialised       : 1;    // true if swash has been initialised
        uint8_t landing_collective      : 1;    // true if collective is setup for landing which has much higher minimum
        uint8_t motor_runup_complete    : 1;    // true if the rotors have had enough time to wind up
    } _heliflags;

    // parameters
    AP_Int16        _roll_max;                  // Maximum roll angle of the swash plate in centi-degrees
    AP_Int16        _pitch_max;                 // Maximum pitch angle of the swash plate in centi-degrees
    AP_Int16        _collective_min;            // Lowest possible servo position for the swashplate
    AP_Int16        _collective_max;            // Highest possible servo position for the swashplate
    AP_Int16        _collective_mid;            // Swash servo position corresponding to zero collective pitch (or zero lift for Assymetrical blades)
    AP_Int8         _servo_manual;              // Pass radio inputs directly to servos during set-up through mission planner
    AP_Int16        _collective_yaw_effect;     // Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
    AP_Int16        _rsc_setpoint;              // rotor speed when RSC mode is set to is enabledv
    AP_Int8         _rsc_mode;                  // Which main rotor ESC control mode is active
    AP_Int8         _rsc_ramp_time;             // Time in seconds for the output to the main rotor's ESC to reach full speed
    AP_Int8         _rsc_runup_time;            // Time in seconds for the main rotor to reach full speed.  Must be longer than _rsc_ramp_time
    AP_Int16        _land_collective_min;       // Minimum collective when landed or landing

    // internal variables
    float           _roll_factors[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];
    float           _pitch_factors[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];
    float           _collective_factors[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];
    float           _roll_scaler;               // scaler to convert roll input from radio (i.e. -4500 ~ 4500) to max roll range
    float           _pitch_scaler;              // scaler to convert pitch input from radio (i.e. -4500 ~ 4500) to max pitch range
    float           _collective_scaler;         // collective scaler to convert pwm form (i.e. 0 ~ 1000) passed in to actual servo range (i.e 1250~1750 would be 500)
    float           _collective_scaler_manual;  // collective scaler to reduce the range of the collective movement while collective is being controlled manually (i.e. directly by the pilot)
    int16_t         _collective_out;            // actual collective pitch value.  Required by the main code for calculating cruise throttle
    int16_t         _collective_mid_pwm;        // collective mid parameter value converted to pwm form (i.e. 0 ~ 1000)
};

#endif  // AP_MOTORSHELI
