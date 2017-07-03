#pragma once

#include "defines.h"
#include "AP_Arming.h"

class AP_MotorsUGV {
public:

    // Constructor
    AP_MotorsUGV();

    enum pwm_type {
        PWM_TYPE_NORMAL = 0,
        PWM_TYPE_ONESHOT = 1,
        PWM_TYPE_ONESHOT125 = 2,
        PWM_TYPE_BRUSHED = 3
     };

    // initialise motors
    void init();

    // setup output in case of main CPU failure
    void setup_safety_output();

    // set steering as a value from -4500 to +4500
    float get_steering() const { return _steering; }
    void set_steering(float steering) { _steering = steering; }

    // get or set throttle as a value from 0 to 100
    float get_throttle() const { return _throttle; }
    void set_throttle(float throttle) { _throttle = throttle; }

    // slew limit throttle for one iteration
    void slew_limit_throttle(float slew_rate, float dt);

    // true if vehicle is capable of skid steering
    bool have_skid_steering() const;

    // output to motors and steering servos
    void output(bool armed);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // setup pwm output type
    void setup_pwm_type();

    // output to regular steering and throttle channels
    void output_regular(bool armed, float steering, float throttle);

    // output to skid steering channels
    void output_skid_steering(bool armed, float steering, float throttle);

    // parameters
    AP_Int8 _pwm_type;  // PWM output type
    AP_Int8 _pwm_freq;  // PWM output freq
    AP_Int8 _disarm_disable_pwm;    // disable PWM output while disarmed

    // internal variables
    float   _steering;  // requested steering as a value from -4500 to +4500
    float   _throttle;  // requested throttle as a value from 0 to 100
};
