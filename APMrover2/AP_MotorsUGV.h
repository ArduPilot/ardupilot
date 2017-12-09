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
    };

    // initialise motors
    void init();

    // setup output in case of main CPU failure
    void setup_safety_output();

    // setup servo output ranges
    void setup_servo_output();

    // get or set steering as a value from -4500 to +4500
    float get_steering() const { return _steering; }
    void set_steering(float steering);

    // get or set throttle as a value from -100 to 100
    float get_throttle() const { return _throttle; }
    void set_throttle(float throttle);

    // true if vehicle is capable of skid steering
    bool have_skid_steering() const;

    // output to motors and steering servos
    void output(bool armed, float dt);

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

    // setup pwm output type
    void setup_pwm_type();

    // output to regular steering and throttle channels
    void output_regular(bool armed, float steering, float throttle);

    // output to skid steering channels
    void output_skid_steering(bool armed, float steering, float throttle);

    // output throttle (-100 ~ +100) to a throttle channel.  Sets relays if required
    void output_throttle(SRV_Channel::Aux_servo_function_t function, float throttle);

    // slew limit throttle for one iteration
    void slew_limit_throttle(float dt);

    // set limits based on steering and throttle input
    void set_limits_from_input(bool armed, float steering, float throttle);

    // scale a throttle using the _thrust_curve_expo parameter.  throttle should be in the range -100 to +100
    float get_scaled_throttle(float throttle) const;

    // external references
    AP_ServoRelayEvents &_relayEvents;

    // parameters
    AP_Int8 _pwm_type;  // PWM output type
    AP_Int8 _pwm_freq;  // PWM output freq for brushed motors
    AP_Int8 _disarm_disable_pwm;    // disable PWM output while disarmed
    AP_Int16 _slew_rate; // slew rate expressed as a percentage / second
    AP_Int8 _throttle_min; // throttle minimum percentage
    AP_Int8 _throttle_max; // throttle maximum percentage
    AP_Float _thrust_curve_expo; // thrust curve exponent from -1 to +1 with 0 being linear

    // internal variables
    float   _steering;  // requested steering as a value from -4500 to +4500
    float   _throttle;  // requested throttle as a value from -100 to 100
    float   _throttle_prev; // throttle input from previous iteration
};
