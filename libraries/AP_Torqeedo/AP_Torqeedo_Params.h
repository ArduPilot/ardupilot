#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_Torqeedo_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_Torqeedo_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Torqeedo_Params);

    // parameters
    AP_Int8 type;          // connector type used (0:disabled, 1:tiller connector, 2:motor connector)
    AP_Int8 pin_onoff;     // Pin number connected to Torqeedo's on/off pin. -1 to disable turning motor on/off from autopilot
    AP_Int8 pin_de;        // Pin number connected to RS485 to Serial converter's DE pin. -1 to disable sending commands to motor
    AP_Int16 options;      // options bitmask
    AP_Int8 motor_power;   // motor power (0 ~ 100).  only applied when using motor connection
    AP_Float slew_time;    // slew rate specified as the minimum number of seconds required to increase the throttle from 0 to 100%.  A value of zero disables the limit
    AP_Float dir_delay;    // direction change delay.  output will remain at zero for this many seconds when transitioning between forward and backwards rotation
    AP_Int16 servo_fn;     // servo output function number
};
