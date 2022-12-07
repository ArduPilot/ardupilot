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
#define AP_MOTORS_HELI_COLLECTIVE_HOVER_DEFAULT 0.5f  // the estimated hover throttle, 0 ~ 1
#define AP_MOTORS_HELI_COLLECTIVE_HOVER_TC      10.0f // time constant used to update estimated hover throttle, 0 ~ 1
#define AP_MOTORS_HELI_COLLECTIVE_HOVER_MIN     0.3f  // minimum possible hover throttle
#define AP_MOTORS_HELI_COLLECTIVE_HOVER_MAX     0.8f // maximum possible hover throttle
#define AP_MOTORS_HELI_COLLECTIVE_MIN_DEG      -90.0f // minimum collective blade pitch angle in deg
#define AP_MOTORS_HELI_COLLECTIVE_MAX_DEG       90.0f // maximum collective blade pitch angle in deg
#define AP_MOTORS_HELI_COLLECTIVE_LAND_MIN      -2.0f // minimum landed collective blade pitch angle in deg for modes using althold


// flybar types
#define AP_MOTORS_HELI_NOFLYBAR                 0

// rsc function output channels.
#define AP_MOTORS_HELI_RSC                      CH_8

class AP_HeliControls;

/// @class      AP_MotorsHeli
class AP_MotorsHeli : public AP_Motors {
public:

    /// Constructor
    AP_MotorsHeli( uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_Motors(speed_hz),
        _main_rotor(SRV_Channel::k_heli_rsc, AP_MOTORS_HELI_RSC)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // init
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override {
        _frame_class = frame_class;
        _frame_type = frame_type;
    }

    // set update rate to motors - a value in hertz
    virtual void set_update_rate( uint16_t speed_hz ) override = 0;

    // output_min - sets servos to neutral point with motors stopped
    void output_min() override;

    //
    // heli specific methods
    //

    // parameter_check - returns true if helicopter specific parameters are sensible, used for pre-arm check
    virtual bool parameter_check(bool display_msg) const;
	
    //set turbine start flag on to initiaize starting sequence
    void set_turb_start(bool turb_start) { _heliflags.start_engine = turb_start; }

    // has_flybar - returns true if we have a mechical flybar
    virtual bool has_flybar() const { return AP_MOTORS_HELI_NOFLYBAR; }

    // set_collective_for_landing - limits collective from going too low if we know we are landed
    void set_collective_for_landing(bool landing) { _heliflags.landing_collective = landing; }

    // set_inverted_flight - enables/disables inverted flight
    void set_inverted_flight(bool inverted) { _heliflags.inverted_flight = inverted; }

    // get_rsc_mode - gets the current rotor speed control method
    uint8_t get_rsc_mode() const { return _main_rotor.get_control_mode(); }

    // get_rsc_setpoint - gets contents of _rsc_setpoint parameter (0~1)
    float get_rsc_setpoint() const { return _main_rotor._rsc_setpoint.get() * 0.01f; }

    // set_desired_rotor_speed - sets target rotor speed as a number from 0 ~ 1
    virtual void set_desired_rotor_speed(float desired_speed) = 0;

    // get_desired_rotor_speed - gets target rotor speed as a number from 0 ~ 1
    virtual float get_desired_rotor_speed() const = 0;

    // get_main_rotor_speed - estimated rotor speed when no governor or speed sensor used
    virtual float get_main_rotor_speed() const = 0;

    // return true if the main rotor is up to speed
    bool rotor_runup_complete() const { return _heliflags.rotor_runup_complete; }

    // rotor_speed_above_critical - return true if rotor speed is above that critical for flight
    virtual bool rotor_speed_above_critical() const = 0;

    //get rotor governor output
    virtual float get_governor_output() const = 0;

    //get engine throttle output
    virtual float get_control_output() const = 0;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint32_t get_motor_mask() override = 0;

    virtual void set_acro_tail(bool set) {}

    // ext_gyro_gain - set external gyro gain in range 0 ~ 1
    virtual void ext_gyro_gain(float gain) {}

    // output - sends commands to the motors
    void output() override;

    // supports_yaw_passthrough
    virtual bool supports_yaw_passthrough() const { return false; }

    // update estimated throttle required to hover
    void update_throttle_hover(float dt);
    float get_throttle_hover() const override { return constrain_float(_collective_hover, AP_MOTORS_HELI_COLLECTIVE_HOVER_MIN, AP_MOTORS_HELI_COLLECTIVE_HOVER_MAX); }

    // accessor to get the takeoff collective flag signifying that current collective is greater than collective required to indicate takeoff
    bool get_takeoff_collective() const { return _heliflags.takeoff_collective; }

    // accessor to get the land min collective flag signifying that current collective is lower than collective required for landing
    bool get_below_land_min_coll() const { return _heliflags.below_land_min_coll; }

    // support passing init_targets_on_arming flag to greater code
    bool init_targets_on_arming() const override { return _heliflags.init_targets_on_arming; }

    // set_in_autorotation - allows main code to set when aircraft is in autorotation.
    void set_in_autorotation(bool autorotation) { _heliflags.in_autorotation = autorotation; }

    // set_enable_bailout - allows main code to set when RSC can immediately ramp engine instantly
    void set_enable_bailout(bool bailout) { _heliflags.enable_bailout = bailout; }

    // set land complete flag
    void set_land_complete(bool landed) { _heliflags.land_complete = landed; }

    // enum for heli optional features
    enum class HeliOption {
        USE_LEAKY_I                     = (1<<0),   // 1
    };

    // use leaking integrator management scheme
    bool using_leaky_integrator() const { return heli_option(HeliOption::USE_LEAKY_I); }

    // Run arming checks
    bool arming_checks(size_t buflen, char *buffer) const override;

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
    void output_armed_stabilizing() override;
    void output_armed_zero_throttle();
    void output_disarmed();

    // external objects we depend upon
    AP_MotorsHeli_RSC   _main_rotor;            // main rotor

    // update_motor_controls - sends commands to motor controllers
    virtual void update_motor_control(RotorControlState state) = 0;

    // run spool logic
    void                output_logic();

    // output_to_motors - sends commands to the motors
    virtual void        output_to_motors() = 0;

    // reset_flight_controls - resets all controls and scalars to flight status
    void reset_flight_controls();

    // update the throttle input filter
    void update_throttle_filter() override;

    // move_actuators - moves swash plate and tail rotor
    virtual void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out) = 0;

    // reset_swash_servo - free up swash servo for maximum movement
    void reset_swash_servo(SRV_Channel::Aux_servo_function_t function);

    // init_outputs - initialise Servo/PWM ranges and endpoints.  This
    // method also updates the initialised flag.
    virtual bool init_outputs() = 0;

    // calculate_armed_scalars - must be implemented by child classes
    virtual void calculate_armed_scalars() = 0;

    // calculate_scalars - must be implemented by child classes
    virtual void calculate_scalars() = 0;

    // servo_test - move servos through full range of movement
    // to be overloaded by child classes, different vehicle types would have different movement patterns
    virtual void servo_test() = 0;

    // write to a swash servo. output value is pwm
    void rc_write_swash(uint8_t chan, float swash_in);

    // save parameters as part of disarming
    void save_params_on_disarm() override;

    // Determines if _heli_options bit is set
    bool heli_option(HeliOption opt) const;

    // updates the takeoff collective flag indicating that current collective is greater than collective required to indicate takeoff.
    void update_takeoff_collective_flag(float coll_out);

    const char* _get_frame_string() const override { return "HELI"; }

    // update turbine start flag
    void update_turbine_start();

    // enum values for HOVER_LEARN parameter
    enum HoverLearn {
        HOVER_LEARN_DISABLED = 0,
        HOVER_LEARN_ONLY = 1,
        HOVER_LEARN_AND_SAVE = 2
    };

    // flags bitmask
    struct heliflags_type {
        uint8_t landing_collective      : 1;    // true if collective is setup for landing which has much higher minimum
        uint8_t rotor_runup_complete    : 1;    // true if the rotors have had enough time to wind up
        uint8_t inverted_flight         : 1;    // true for inverted flight
        uint8_t init_targets_on_arming  : 1;    // 0 if targets were initialized, 1 if targets were not initialized after arming
        uint8_t save_rsc_mode           : 1;    // used to determine the rsc mode needs to be saved while disarmed
        uint8_t in_autorotation         : 1;    // true if aircraft is in autorotation
        uint8_t enable_bailout          : 1;    // true if allowing RSC to quickly ramp up engine
        uint8_t servo_test_running      : 1;    // true if servo_test is running
        uint8_t land_complete           : 1;    // true if aircraft is landed
        uint8_t takeoff_collective      : 1;    // true if collective is above 30% between H_COL_MID and H_COL_MAX
        uint8_t below_land_min_coll     : 1;    // true if collective is below H_COL_LAND_MIN
        uint8_t rotor_spooldown_complete : 1;    // true if the rotors have spooled down completely
        uint8_t start_engine            : 1;    // true if turbine start RC option is initiated
    } _heliflags;

    // parameters
    AP_Int16        _cyclic_max;                // Maximum cyclic angle of the swash plate in centi-degrees
    AP_Int16        _collective_min;            // Lowest possible servo position for the swashplate
    AP_Int16        _collective_max;            // Highest possible servo position for the swashplate
    AP_Int8         _servo_mode;                // Pass radio inputs directly to servos during set-up through mission planner
    AP_Int8         _servo_test;                // sets number of cycles to test servo movement on bootup
    AP_Float        _collective_hover;          // estimated collective required to hover throttle in the range 0 ~ 1
    AP_Int8         _collective_hover_learn;    // enable/disabled hover collective learning
    AP_Int8         _heli_options;              // bitmask for optional features
    AP_Float        _collective_zero_thrust_deg;// Zero thrust blade collective pitch in degrees
    AP_Float        _collective_land_min_deg;   // Minimum Landed collective blade pitch in degrees for non-manual collective modes (i.e. modes that use altitude hold)
    AP_Float        _collective_max_deg;        // Maximum collective blade pitch angle in deg that corresponds to the PWM set for maximum collective pitch (H_COL_MAX)
    AP_Float        _collective_min_deg;        // Minimum collective blade pitch angle in deg that corresponds to the PWM set for minimum collective pitch (H_COL_MIN)

    // internal variables
    float           _collective_zero_thrust_pct;      // collective zero thrutst parameter value converted to 0 ~ 1 range
    float           _collective_land_min_pct;      // collective land min parameter value converted to 0 ~ 1 range
    uint8_t         _servo_test_cycle_counter = 0;   // number of test cycles left to run after bootup

    motor_frame_type _frame_type;
    motor_frame_class _frame_class;
};
