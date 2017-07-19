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
        PWM_TYPE_BRUSHED = 3,
        PWM_TYPE_BRUSHEDBIPOLAR = 4,
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

    void setup_servo_output();

    // set steering as a value from -4500 to +4500
    float get_steering() const { return _steering; }
    void set_steering(float steering) { _steering = steering; }

    // get or set throttle as a value from 0 to 100
    float get_throttle() const { return _throttle; }
    void set_throttle(float throttle) { _throttle = throttle; }

    // true if vehicle is capable of skid steering
    bool have_skid_steering() const;

    // output to motors and steering servos
    void output(bool armed, float dt);

    // set when to use slew rate limiter
    void slew_limit_throttle(bool value) { _use_slew_rate = value; }

    // test steering or throttle output as a percentage of the total (range -100 to +100)
    // used in response to DO_MOTOR_TEST mavlink command
    bool output_test_pct(motor_test_order motor_seq, float pct);

    // test steering or throttle output using a pwm value
    bool output_test_pwm(motor_test_order motor_seq, float pwm);

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

    // external references
    AP_ServoRelayEvents &_relayEvents;

    // parameters
    AP_Int8 _pwm_type;  // PWM output type
    AP_Int8 _pwm_freq;  // PWM output freq for brushed motors
    AP_Int8 _disarm_disable_pwm;    // disable PWM output while disarmed
    AP_Int8 _slew_rate; // slew rate expressed as a percentage / second

    // internal variables
    float   _steering;  // requested steering as a value from -4500 to +4500
    float   _throttle;  // requested throttle as a value from 0 to 100
    float   _last_throttle;
    bool    _use_slew_rate; // true if we should slew limit the throttle for one interation
};
