// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsHeli.h
/// @brief	Motor control class for Traditional Heli

#ifndef __AP_MOTORS_HELI_H__
#define __AP_MOTORS_HELI_H__

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_Motors.h"

// output channels
#define AP_MOTORS_HELI_EXT_GYRO                 CH_7    // tail servo uses channel 7
#define AP_MOTORS_HELI_EXT_RSC                  CH_8    // main rotor controlled with channel 8

// maximum number of swashplate servos
#define AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS    3

// servo output rates
#define AP_MOTORS_HELI_SPEED_DEFAULT            125     // default servo update rate for helicopters
#define AP_MOTORS_HELI_SPEED_DIGITAL_SERVOS     125     // update rate for digital servos
#define AP_MOTORS_HELI_SPEED_ANALOG_SERVOS      125     // update rate for analog servos

// TradHeli Aux Function Output Channels
#define AP_MOTORS_HELI_AUX                      CH_7
#define AP_MOTORS_HELI_RSC                      CH_8

// servo position defaults
#define AP_MOTORS_HELI_SERVO1_POS               -60
#define AP_MOTORS_HELI_SERVO2_POS                60
#define AP_MOTORS_HELI_SERVO3_POS               180

// swash type definitions
#define AP_MOTORS_HELI_SWASH_CCPM               0
#define AP_MOTORS_HELI_SWASH_H1                 1

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

// tail types
#define AP_MOTORS_HELI_TAILTYPE_SERVO                   0
#define AP_MOTORS_HELI_TAILTYPE_SERVO_EXTGYRO           1
#define AP_MOTORS_HELI_TAILTYPE_DIRECTDRIVE_VARPITCH    2
#define AP_MOTORS_HELI_TAILTYPE_DIRECTDRIVE_FIXEDPITCH  3

// default external gyro gain (ch7 out)
#define AP_MOTORS_HELI_EXT_GYRO_GAIN            350

// minimum outputs for direct drive motors
#define AP_MOTOR_HELI_DDTAIL_DEFAULT       500


// main rotor speed control types (ch8 out)
#define AP_MOTORS_HELI_RSC_MODE_NONE            0       // main rotor ESC is directly connected to receiver, pilot controls ESC speed through transmitter directly
#define AP_MOTORS_HELI_RSC_MODE_CH8_PASSTHROUGH 1       // main rotor ESC is connected to RC8 (out), pilot desired rotor speed provided by CH8 input
#define AP_MOTORS_HELI_RSC_MODE_SETPOINT        2       // main rotor ESC is connected to RC8 (out), desired speed is held in RSC_SETPOINT parameter

// default main rotor speed (ch8 out) as a number from 0 ~ 1000
#define AP_MOTORS_HELI_RSC_SETPOINT             500

// default main rotor ramp up time in seconds
#define AP_MOTORS_HELI_RSC_RAMP_TIME            1       // 1 second to ramp output to main rotor ESC to full power (most people use exterrnal govenors so we can ramp up quickly)
#define AP_MOTORS_HELI_RSC_RUNUP_TIME           10      // 10 seconds for rotor to reach full speed
#define AP_MOTORS_HELI_TAIL_RAMP_INCREMENT      5       // 5 is 2 seconds for direct drive tail rotor to reach to full speed (5 = (2sec*100hz)/1000)

// motor run-up time default in 100th of seconds
#define AP_MOTORS_HELI_MOTOR_RUNUP_TIME         500     // 500 = 5 seconds

// flybar types
#define AP_MOTORS_HELI_NOFLYBAR                 0
#define AP_MOTORS_HELI_FLYBAR                   1

class AP_HeliControls;

/// @class      AP_MotorsHeli
class AP_MotorsHeli : public AP_Motors {
public:

    /// Constructor
    AP_MotorsHeli( RC_Channel*      rc_roll,
                   RC_Channel*      rc_pitch,
                   RC_Channel*      rc_throttle,
                   RC_Channel*      rc_yaw,
                   RC_Channel*      servo_aux,
                   RC_Channel*      servo_rotor,
                   RC_Channel*      swash_servo_1,
                   RC_Channel*      swash_servo_2,
                   RC_Channel*      swash_servo_3,
                   RC_Channel*      yaw_servo,
                   uint16_t         speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_Motors(rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz),
        _servo_aux(servo_aux),
        _servo_rsc(servo_rotor),
        _servo_1(swash_servo_1),
        _servo_2(swash_servo_2),
        _servo_3(swash_servo_3),
        _servo_4(yaw_servo),
        _roll_scaler(1),
        _pitch_scaler(1),
        _collective_scalar(1),
        _collective_out(0),
        _collective_mid_pwm(0),
        _rotor_desired(0),
        _rotor_out(0),
        _rsc_ramp_increment(0.0f),
        _rsc_runup_increment(0.0f),
        _rotor_speed_estimate(0.0f),
        _tail_direct_drive_out(0)
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

    // output_min - sends minimum values out to the motors
    void output_min();

    // output_test - wiggle servos in order to show connections are correct
    void output_test();

    //
    // heli specific methods
    //

    // allow_arming - returns true if main rotor is spinning and it is ok to arm
    bool allow_arming();

    // _tail_type - returns the tail type (servo, servo with ext gyro, direct drive var pitch, direct drive fixed pitch)
    int16_t tail_type() { return _tail_type; }

    // ext_gyro_gain - gets and sets external gyro gain as a pwm (1000~2000)
    int16_t ext_gyro_gain() { return _ext_gyro_gain; }
    void ext_gyro_gain(int16_t pwm) { _ext_gyro_gain = pwm; }

    // has_flybar - returns true if we have a mechical flybar
    bool has_flybar() { return _flybar_mode; }

    // get_collective_mid - returns collective mid position as a number from 0 ~ 1000
    int16_t get_collective_mid() { return  _collective_mid; }

    // get_collective_out - returns collective position from last output as a number from 0 ~ 1000
    int16_t get_collective_out() { return _collective_out; }

    // set_collective_for_landing - limits collective from going too low if we know we are landed
    void set_collective_for_landing(bool landing) { _heliflags.landing_collective = landing; }

    // get_rsc_mode - gets the rotor speed control method (AP_MOTORS_HELI_RSC_MODE_NONE, AP_MOTORS_HELI_RSC_MODE_CH8_PASSTHROUGH or AP_MOTORS_HELI_RSC_MODE_SETPOINT)
    uint8_t get_rsc_mode() { return _rsc_mode; }

    // get_rsc_setpoint - gets contents of _rsc_setpoint parameter (0~1000)
    int16_t get_rsc_setpoint() { return _rsc_setpoint; }

    // set_desired_rotor_speed - sets target rotor speed as a number from 0 ~ 1000
    void set_desired_rotor_speed(int16_t desired_speed);

    // return true if the main rotor is up to speed
    bool motor_runup_complete();

    // recalc_scalers - recalculates various scalers used.  Should be called at about 1hz to allow users to see effect of changing parameters
    void recalc_scalers();

    // get_phase_angle - returns phase angle
    int16_t get_phase_angle() { return _phase_angle; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // output - sends commands to the motors
    void output_armed();
    void output_disarmed();

private:

    // heli_move_swash - moves swash plate to attitude of parameters passed in
    void move_swash(int16_t roll_out, int16_t pitch_out, int16_t coll_in, int16_t yaw_out);

    // reset_swash - free up swash for maximum movements. Used for set-up
    void reset_swash();

    // init_swash - initialise the swash plate
    void init_swash();

    // calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position
    void calculate_roll_pitch_collective_factors();

    // rsc_control - main function to update values to send to main rotor and tail rotor ESCs
    void rsc_control();

    // rotor_ramp - ramps rotor towards target. result put rotor_out and sent to ESC
    void rotor_ramp(int16_t rotor_target);

    // tail_ramp - ramps tail motor towards target.  Only used for direct drive variable pitch tails
    // results put into _tail_direct_drive_out and sent to ESC
    void tail_ramp(int16_t tail_target);

    // return true if the tail rotor is up to speed
    bool tail_rotor_runup_complete();

    // write_rsc - outputs pwm onto output rsc channel (ch8).  servo_out parameter is of the range 0 ~ 1000
    void write_rsc(int16_t servo_out);

    // write_aux - outputs pwm onto output aux channel (ch7). servo_out parameter is of the range 0 ~ 1000
    void write_aux(int16_t servo_out);

    // external objects we depend upon
    RC_Channel      *_servo_aux;                // output to ext gyro gain and tail direct drive esc (ch7)
    RC_Channel      *_servo_rsc;                // output to main rotor esc (ch8)
    RC_Channel      *_servo_1;                  // swash plate servo #1
    RC_Channel      *_servo_2;                  // swash plate servo #2
    RC_Channel      *_servo_3;                  // swash plate servo #3
    RC_Channel      *_servo_4;                  // tail servo

    // flags bitmask
    struct heliflags_type {
        uint8_t swash_initialised       : 1;    // true if swash has been initialised
        uint8_t landing_collective      : 1;    // true if collective is setup for landing which has much higher minimum
        uint8_t motor_runup_complete    : 1;    // true if the rotors have had enough time to wind up
    } _heliflags;

    // parameters
    AP_Int16        _servo1_pos;                // Angular location of swash servo #1
    AP_Int16        _servo2_pos;                // Angular location of swash servo #2
    AP_Int16        _servo3_pos;                // Angular location of swash servo #3
    AP_Int16        _roll_max;                  // Maximum roll angle of the swash plate in centi-degrees
    AP_Int16        _pitch_max;                 // Maximum pitch angle of the swash plate in centi-degrees
    AP_Int16        _collective_min;            // Lowest possible servo position for the swashplate
    AP_Int16        _collective_max;            // Highest possible servo position for the swashplate
    AP_Int16        _collective_mid;            // Swash servo position corresponding to zero collective pitch (or zero lift for Assymetrical blades)
    AP_Int16        _tail_type;                 // Tail type used: Servo, Servo with external gyro, direct drive variable pitch or direct drive fixed pitch
    AP_Int8         _swash_type;                // Swash Type Setting - either 3-servo CCPM or H1 Mechanical Mixing
    AP_Int16        _ext_gyro_gain;             // PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro
    AP_Int8         _servo_manual;              // Pass radio inputs directly to servos during set-up through mission planner
    AP_Int16        _phase_angle;               // Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
    AP_Int16        _collective_yaw_effect;     // Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.    
    AP_Int16        _rsc_setpoint;              // rotor speed when RSC mode is set to is enabledv
    AP_Int8         _rsc_mode;                  // Which main rotor ESC control mode is active
    AP_Int8         _rsc_ramp_time;             // Time in seconds for the output to the main rotor's ESC to reach full speed
    AP_Int8         _rsc_runup_time;            // Time in seconds for the main rotor to reach full speed.  Must be longer than _rsc_ramp_time
    AP_Int8         _flybar_mode;               // Flybar present or not.  Affects attitude controller used during ACRO flight mode
    AP_Int16        _land_collective_min;       // Minimum collective when landed or landing
    AP_Int16        _direct_drive_tailspeed;    // Direct Drive VarPitch Tail ESC speed (0 ~ 1000)

    // internal variables
    float           _rollFactor[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];
    float           _pitchFactor[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];
    float           _collectiveFactor[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];
    float           _roll_scaler;               // scaler to convert roll input from radio (i.e. -4500 ~ 4500) to max roll range
    float           _pitch_scaler;              // scaler to convert pitch input from radio (i.e. -4500 ~ 4500) to max pitch range
    float           _collective_scalar;         // collective scalar to convert pwm form (i.e. 0 ~ 1000) passed in to actual servo range (i.e 1250~1750 would be 500)
    float           _collective_scalar_manual;  // collective scalar to reduce the range of the collective movement while collective is being controlled manually (i.e. directly by the pilot)
    int16_t         _collective_out;            // actual collective pitch value.  Required by the main code for calculating cruise throttle
    int16_t         _collective_mid_pwm;        // collective mid parameter value converted to pwm form (i.e. 0 ~ 1000)
    int16_t         _rotor_desired;             // latest desired rotor speed from pilot
    float           _rotor_out;                 // latest output sent to the main rotor or an estimate of the rotors actual speed (whichever is higher) (0 ~ 1000)
    float           _rsc_ramp_increment;        // the amount we can increase the rotor output during each 100hz iteration
    float           _rsc_runup_increment;       // the amount we can increase the rotor's estimated speed during each 100hz iteration
    float           _rotor_speed_estimate;      // estimated speed of the main rotor (0~1000)
    int16_t         _tail_direct_drive_out;     // current ramped speed of output on ch7 when using direct drive variable pitch tail type
};

#endif  // AP_MOTORSHELI
