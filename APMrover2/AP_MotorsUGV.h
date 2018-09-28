#pragma once

#include "defines.h"
#include "AP_Arming.h"
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>

class AP_MotorsUGV {
public:

    // Constructor
    AP_MotorsUGV(AP_ServoRelayEvents &relayEvents);

    enum pwm_type {
        PWM_TYPE_NORMAL = 0,
        PWM_TYPE_ONESHOT = 1,
        PWM_TYPE_ONESHOT125 = 2,
        PWM_TYPE_BRUSHED_WITH_RELAY = 3,
        PWM_TYPE_BRUSHED_BIPOLAR = 4,
     };

    enum motor_test_order {
        MOTOR_TEST_THROTTLE = 1,
        MOTOR_TEST_STEERING = 2,
        MOTOR_TEST_THROTTLE_LEFT = 3,
        MOTOR_TEST_THROTTLE_RIGHT = 4,
        MOTOR_TEST_MAINSAIL = 5
    };

    // supported custom motor configurations
    enum frame_type {
        FRAME_TYPE_UNDEFINED = 0,
        FRAME_TYPE_OMNI3 = 1,
        FRAME_TYPE_OMNIX = 2,
        FRAME_TYPE_OMNIPLUS = 3,
    };

    // initialise motors
    void init();

    // return true if motors are active
    bool active() const;

    // setup output in case of main CPU failure
    void setup_safety_output();

    // setup servo output ranges
    void setup_servo_output();

    // config for frames with vectored motors
    void setup_motors();

    // add motor using separate throttle, steering and lateral factors for frames with custom motor configuration
    void add_motor(int8_t motor_num, float throttle_factor, float steering_factor, float lateral_factor);

    // add a motor and set up output function
    void add_motor_num(int8_t motor_num);

    // disable motor and remove all throttle, steering and lateral factor for this motor
    void clear_motors(int8_t motor_num);

    // get or set steering as a value from -4500 to +4500
    //   apply_scaling should be set to false for manual modes where
    //   no scaling by speed or angle should e performed
    float get_steering() const { return _steering; }
    void set_steering(float steering, bool apply_scaling = true);

    // get or set throttle as a value from -100 to 100
    float get_throttle() const { return _throttle; }
    void set_throttle(float throttle);

    // set lateral input as a value from -100 to +100
    void set_lateral(float lateral);

    // set mainsail input as a value from 0 to 100
    void set_mainsail(float mainsail);
    float get_mainsail() const { return _mainsail; }

    // get slew limited throttle
    // used by manual mode to avoid bad steering behaviour during transitions from forward to reverse
    // same as private slew_limit_throttle method (see below) but does not update throttle state
    float get_slew_limited_throttle(float throttle, float dt) const;

    // true if vehicle is capable of skid steering
    bool have_skid_steering() const;

    // true if vehicle has vectored thrust (i.e. boat with motor on steering servo)
    bool have_vectored_thrust() const { return is_positive(_vector_throttle_base); }

    // true if the vehicle has a mainsail
    bool has_sail() const;

    // output to motors and steering servos
    // ground_speed should be the vehicle's speed over the surface in m/s
    // dt should be expected time between calls to this function
    void output(bool armed, float ground_speed, float dt);

    // test steering or throttle output as a percentage of the total (range -100 to +100)
    // used in response to DO_MOTOR_TEST mavlink command
    bool output_test_pct(motor_test_order motor_seq, float pct);

    // test steering or throttle output using a pwm value
    bool output_test_pwm(motor_test_order motor_seq, float pwm);

    //  returns true if checks pass, false if they fail.  display_failure argument should be true to send text messages to GCS
    bool pre_arm_check(bool report) const;

    // structure for holding motor limit flags
    struct AP_MotorsUGV_limit {
        uint8_t steer_left      : 1; // we have reached the steering controller's left most limit
        uint8_t steer_right     : 1; // we have reached the steering controller's right most limit
        uint8_t throttle_lower  : 1; // we have reached throttle's lower limit
        uint8_t throttle_upper  : 1; // we have reached throttle's upper limit
    } limit;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // sanity check parameters
    void sanity_check_parameters();

    // setup pwm output type
    void setup_pwm_type();

    // output to regular steering and throttle channels
    void output_regular(bool armed, float ground_speed, float steering, float throttle);

    // output to skid steering channels
    void output_skid_steering(bool armed, float steering, float throttle, float dt);

    // output for vectored and custom motors configuration
    void output_custom_config(bool armed, float steering, float throttle, float lateral);

    // output throttle (-100 ~ +100) to a throttle channel.  Sets relays if required
    // dt is the main loop time interval and is required when rate control is required
    void output_throttle(SRV_Channel::Aux_servo_function_t function, float throttle, float dt = 0.0f);

    // output for sailboat's mainsail in the range of 0 to 100
    void output_mainsail();

    // slew limit throttle for one iteration
    void slew_limit_throttle(float dt);

    // set limits based on steering and throttle input
    void set_limits_from_input(bool armed, float steering, float throttle);

    // scale a throttle using the _thrust_curve_expo parameter.  throttle should be in the range -100 to +100
    float get_scaled_throttle(float throttle) const;

    // use rate controller to achieve desired throttle
    float get_rate_controlled_throttle(SRV_Channel::Aux_servo_function_t function, float throttle, float dt);

    // external references
    AP_ServoRelayEvents &_relayEvents;

    static const int8_t AP_MOTORS_NUM_MOTORS_MAX = 4;

    // parameters
    AP_Int8 _pwm_type;  // PWM output type
    AP_Int8 _pwm_freq;  // PWM output freq for brushed motors
    AP_Int8 _disarm_disable_pwm;    // disable PWM output while disarmed
    AP_Int16 _slew_rate; // slew rate expressed as a percentage / second
    AP_Int8 _throttle_min; // throttle minimum percentage
    AP_Int8 _throttle_max; // throttle maximum percentage
    AP_Float _thrust_curve_expo; // thrust curve exponent from -1 to +1 with 0 being linear
    AP_Float _vector_throttle_base;  // throttle level above which steering is scaled down when using vector thrust.  zero to disable vectored thrust
    AP_Float _speed_scale_base;  // speed above which steering is scaled down when using regular steering/throttle vehicles.  zero to disable speed scaling

    // internal variables
    float   _steering;  // requested steering as a value from -4500 to +4500
    float   _throttle;  // requested throttle as a value from -100 to 100
    float   _throttle_prev; // throttle input from previous iteration
    bool    _scale_steering = true; // true if we should scale steering by speed or angle
    float   _lateral;  // requested lateral input as a value from -100 to +100
    float   _mainsail;  // requested mainsail input as a value from 0 to 100

    // custom config variables
    float   _throttle_factor[AP_MOTORS_NUM_MOTORS_MAX];
    float   _steering_factor[AP_MOTORS_NUM_MOTORS_MAX];
    float   _lateral_factor[AP_MOTORS_NUM_MOTORS_MAX];
    uint8_t   _motors_num;
};
