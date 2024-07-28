#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Camera/AP_Camera_config.h>

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
    AP_Int8 options;            // whether to start recording when armed and stop when disarmed
    AP_Int8 mount_instance;     // mount instance to which camera is associated with
    AP_Float hfov;              // horizontal field of view in degrees
    AP_Float vfov;              // vertical field of view in degrees
#if AP_CAMERA_TRACKING_ENABLED
    AP_Int8 track_enable;       // enable or disable camera tracking
    AP_Int16 track_sysid;        // system id for the tracking device
    AP_Int16 track_compid;       // component id for the tracking device
#endif
    // pin number for accurate camera feedback messages
    AP_Int8 feedback_pin;
    AP_Int8 feedback_polarity;
};
