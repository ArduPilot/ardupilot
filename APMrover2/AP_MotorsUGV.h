#pragma once

#include "defines.h"
#include "AP_Arming.h"

class AP_MotorsUGV {
public:

    // Constructor
    AP_MotorsUGV(AP_Arming_Rover &arming):
            _arming(arming)
    {
        AP_Param::setup_object_defaults(this, var_info);

    }
    enum pwm_type {
            PWM_TYPE_NORMAL = 0,
            PWM_TYPE_ONESHOT = 1,
            PWM_TYPE_ONESHOT125 = 2,
            PWM_TYPE_BRUSHED = 3
            };

    void setup_motors() const;
    void setup_motors_type() const;
    void set_safety_to_trim() const;
    bool have_skid_steering() const;
    void output_mixing() const;
    void failsafeThrottle(bool isFailsafe);
    void autoMode(enum mode currentMode);
    void output() const;
    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_Arming_Rover &_arming;
    bool _isFailsafeThrottle = false;
    bool _isAutoMode = false;
    AP_Int8 _pwm_type;  // PWM output type
    AP_Int8 _pwm_freq;   // PWM output freq

    bool check_status() const;
    // output_to_motors - sends values out to the motors
    void output_to_motors() const;
};
