/// @file	AP_MotorsHeli.h
/// @brief	Motor control class for Traditional Heli
#pragma once

#include <inttypes.h>

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_Motors_Class.h"
#include "AP_MotorsHeli_RSC.h"

// servo output rates
#define AP_MOTORS_HELI_SPEED_DEFAULT            125     // default servo update rate for helicopters

// default swash min and max angles and positions
#define AP_MOTORS_HELI_SWASH_CYCLIC_MAX         2500
#define AP_MOTORS_HELI_COLLECTIVE_MIN           1250
#define AP_MOTORS_HELI_COLLECTIVE_MAX           1750
#define AP_MOTORS_HELI_COLLECTIVE_MID           1500

// swash min while landed or landing (as a number from 0 ~ 1000
#define AP_MOTORS_HELI_LAND_COLLECTIVE_MIN      0

// default main rotor speed (ch8 out) as a number from 0 ~ 1000
#define AP_MOTORS_HELI_RSC_SETPOINT             700

// default main rotor critical speed
#define AP_MOTORS_HELI_RSC_CRITICAL             500

// RSC output defaults
#define AP_MOTORS_HELI_RSC_IDLE_DEFAULT         0
#define AP_MOTORS_HELI_RSC_THRCRV_0_DEFAULT     250
#define AP_MOTORS_HELI_RSC_THRCRV_25_DEFAULT    320
#define AP_MOTORS_HELI_RSC_THRCRV_50_DEFAULT    380
#define AP_MOTORS_HELI_RSC_THRCRV_75_DEFAULT    500
#define AP_MOTORS_HELI_RSC_THRCRV_100_DEFAULT   1000

// default main rotor ramp up time in seconds
#define AP_MOTORS_HELI_RSC_RAMP_TIME            1       // 1 second to ramp output to main rotor ESC to setpoint
#define AP_MOTORS_HELI_RSC_RUNUP_TIME           10      // 10 seconds for rotor to reach full speed

// flybar types
#define AP_MOTORS_HELI_NOFLYBAR                 0

class AP_HeliControls;

/// @class      AP_MotorsHeli
class AP_MotorsHeli : public AP_Motors {
public:

    /// Constructor
    AP_MotorsHeli( uint16_t         loop_rate,
                   uint16_t         speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_Motors(loop_rate, speed_hz)
    {
        AP_Param::setup_object_defaults(this, var_info);

        // initialise flags
        _heliflags.landing_collective = 0;
        _heliflags.rotor_runup_complete = 0;
    };

    // init
    void init(motor_frame_class frame_class, motor_frame_type frame_type);

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type);

    // set update rate to motors - a value in hertz
    virtual void set_update_rate( uint16_t speed_hz ) = 0;

    // output_min - sets servos to neutral point with motors stopped
    void output_min();

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void output_test_seq(uint8_t motor_seq, int16_t pwm) override = 0;

    //
    // heli specific methods
    //

    // parameter_check - returns true if helicopter specific parameters are sensible, used for pre-arm check
    virtual bool parameter_check(bool display_msg) const;

    // has_flybar - returns true if we have a mechical flybar
    virtual bool has_flybar() const { return AP_MOTORS_HELI_NOFLYBAR; }

    // set_collective_for_landing - limits collective from going too low if we know we are landed
    void set_collective_for_landing(bool landing) { _heliflags.landing_collective = landing; }

    // set_inverted_flight - enables/disables inverted flight
    void set_inverted_flight(bool inverted) { _heliflags.inverted_flight = inverted; }

    // get_rsc_mode - gets the rotor speed control method (AP_MOTORS_HELI_RSC_MODE_CH8_PASSTHROUGH or AP_MOTORS_HELI_RSC_MODE_SETPOINT)
    uint8_t get_rsc_mode() const { return _rsc_mode; }

    // get_rsc_setpoint - gets contents of _rsc_setpoint parameter (0~1)
    float get_rsc_setpoint() const { return _rsc_setpoint / 1000.0f; }

    // set_desired_rotor_speed - sets target rotor speed as a number from 0 ~ 1
    virtual void set_desired_rotor_speed(float desired_speed) = 0;

    // get_desired_rotor_speed - gets target rotor speed as a number from 0 ~ 1
    virtual float get_desired_rotor_speed() const = 0;

    // get_main_rotor_speed - gets estimated or measured main rotor speed
    virtual float get_main_rotor_speed() const = 0;

    // return true if the main rotor is up to speed
    bool rotor_runup_complete() const { return _heliflags.rotor_runup_complete; }

    // rotor_speed_above_critical - return true if rotor speed is above that critical for flight
    virtual bool rotor_speed_above_critical() const = 0;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t get_motor_mask() = 0;

    virtual void set_acro_tail(bool set) {}

    // ext_gyro_gain - set external gyro gain in range 0 ~ 1
    virtual void ext_gyro_gain(float gain) {}

    // output - sends commands to the motors
    void output();

    // supports_yaw_passthrough
    virtual bool supports_yaw_passthrough() const { return false; }

    float get_throttle_hover() const { return 0.5f; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // manual servo modes (used for setup)
    enum ServoControlModes {
        SERVO_CONTROL_MODE_AUTOMATED = 0,
        SERVO_CONTROL_MODE_MANUAL_PASSTHROUGH,
        SERVO_CONTROL_MODE_MANUAL_MAX,
        SERVO_CONTROL_MODE_MANUAL_CENTER,
        SERVO_CONTROL_MODE_MANUAL_MIN,
        SERVO_CONTROL_MODE_MANUAL_OSCILLATE,
    };

    // output - sends commands to the motors
    void output_armed_stabilizing();
    void output_armed_zero_throttle();
    void output_disarmed();

    // update_motor_controls - sends commands to motor controllers
    virtual void update_motor_control(RotorControlState state) = 0;

    // reset_flight_controls - resets all controls and scalars to flight status
    void reset_flight_controls();

    // update the throttle input filter
    void update_throttle_filter();

    // move_actuators - moves swash plate and tail rotor
    virtual void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out) = 0;

    // reset_swash_servo - free up swash servo for maximum movement
    void reset_swash_servo(SRV_Channel *servo);

    // init_outputs - initialise Servo/PWM ranges and endpoints
    virtual bool init_outputs() = 0;

    // calculate_armed_scalars - must be implemented by child classes
    virtual void calculate_armed_scalars() = 0;

    // calculate_scalars - must be implemented by child classes
    virtual void calculate_scalars() = 0;

    // calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position
    virtual void calculate_roll_pitch_collective_factors() = 0;

    // servo_test - move servos through full range of movement
    // to be overloaded by child classes, different vehicle types would have different movement patterns
    virtual void servo_test() = 0;

    // convert input in -1 to +1 range to pwm output for swashplate servos. .  Special handling of trim is required 
    // to keep travel between the swashplate servos consistent.
    int16_t calc_pwm_output_1to1_swash_servo(float input, const SRV_Channel *servo);

    // flags bitmask
    struct heliflags_type {
        uint8_t landing_collective      : 1;    // true if collective is setup for landing which has much higher minimum
        uint8_t rotor_runup_complete    : 1;    // true if the rotors have had enough time to wind up
        uint8_t inverted_flight         : 1;    // true for inverted flight
    } _heliflags;

    // parameters
    AP_Int16        _cyclic_max;                // Maximum cyclic angle of the swash plate in centi-degrees
    AP_Int16        _collective_min;            // Lowest possible servo position for the swashplate
    AP_Int16        _collective_max;            // Highest possible servo position for the swashplate
    AP_Int16        _collective_mid;            // Swash servo position corresponding to zero collective pitch (or zero lift for Asymmetrical blades)
    AP_Int8         _servo_mode;              // Pass radio inputs directly to servos during set-up through mission planner
    AP_Int16        _rsc_setpoint;              // rotor speed when RSC mode is set to is enabledv
    AP_Int8         _rsc_mode;                  // Which main rotor ESC control mode is active
    AP_Int8         _rsc_ramp_time;             // Time in seconds for the output to the main rotor's ESC to reach setpoint
    AP_Int8         _rsc_runup_time;            // Time in seconds for the main rotor to reach full speed.  Must be longer than _rsc_ramp_time
    AP_Int16        _land_collective_min;       // Minimum collective when landed or landing
    AP_Int16        _rsc_critical;              // Rotor speed below which flight is not possible
    AP_Int16        _rsc_idle_output;           // Rotor control output while at idle
    AP_Int16        _rsc_thrcrv[5];             // throttle value sent to throttle servo at 0, 25, 50, 75 and 100 percent collective
    AP_Int16        _rsc_slewrate;              // throttle slew rate (percentage per second)
    AP_Int8         _servo_test;                // sets number of cycles to test servo movement on bootup

    // internal variables
    float           _collective_mid_pct = 0.0f;      // collective mid parameter value converted to 0 ~ 1 range
    uint8_t         _servo_test_cycle_counter = 0;   // number of test cycles left to run after bootup

    motor_frame_type _frame_type;
};
