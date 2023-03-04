#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_Camera_Params {

public:

    static const struct AP_Param::GroupInfo var_info[];

    AP_Camera_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Camera_Params);

    AP_Int8 type;               // camera type (see CameraType enum)
    AP_Float trigger_duration;  // duration in seconds that the camera shutter is held open
    AP_Int16 servo_on_pwm;      // PWM value to move servo to when shutter is activated
    AP_Int16 servo_off_pwm;     // PWM value to move servo to when shutter is deactivated
    AP_Float trigg_dist;        // distance between trigger points (meters)
    AP_Int8 relay_on;           // relay value to trigger camera
    AP_Float interval_min;      // minimum time (in seconds) between shots required by camera

    // pin number for accurate camera feedback messages
    AP_Int8 feedback_pin;
    AP_Int8 feedback_polarity;
};
