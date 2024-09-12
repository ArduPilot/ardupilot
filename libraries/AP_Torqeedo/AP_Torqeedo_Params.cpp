#include "AP_Torqeedo_Params.h"
#include <SRV_Channel/SRV_Channel.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_Torqeedo_Params::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Torqeedo connection type
    // @Description: Torqeedo connection type
    // @Values: 0:Disabled, 1:Tiller, 2:Motor
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_Torqeedo_Params, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ONOFF_PIN
    // @DisplayName: Torqeedo ON/Off pin
    // @Description: Pin number connected to Torqeedo's on/off pin. -1 to use serial port's RTS pin if available
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("ONOFF_PIN", 2, AP_Torqeedo_Params, pin_onoff, -1),

    // @Param: DE_PIN
    // @DisplayName: Torqeedo DE pin
    // @Description: Pin number connected to RS485 to Serial converter's DE pin. -1 to use serial port's CTS pin if available
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("DE_PIN", 3, AP_Torqeedo_Params, pin_de, -1),

    // @Param: OPTIONS
    // @DisplayName: Torqeedo Options
    // @Description: Torqeedo Options Bitmask
    // @Bitmask: 0:Log,1:Send debug to GCS
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 4, AP_Torqeedo_Params, options, 1),

    // @Param: POWER
    // @DisplayName: Torqeedo Motor Power
    // @Description: Torqeedo motor power.  Only applied when using motor connection type (e.g. TRQD_TYPE=2)
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POWER", 5, AP_Torqeedo_Params, motor_power, 100),

    // @Param: SLEW_TIME
    // @DisplayName: Torqeedo Throttle Slew Time
    // @Description: Torqeedo slew rate specified as the minimum number of seconds required to increase the throttle from 0 to 100%.  Higher values cause a slower response, lower values cause a faster response.  A value of zero disables the limit
    // @Units: s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SLEW_TIME", 6, AP_Torqeedo_Params, slew_time, 2.0),

    // @Param: DIR_DELAY
    // @DisplayName: Torqeedo Direction Change Delay
    // @Description: Torqeedo direction change delay.  Output will remain at zero for this many seconds when transitioning between forward and backwards rotation
    // @Units: s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DIR_DELAY", 7, AP_Torqeedo_Params, dir_delay, 1.0),

    // @Param: SERVO_FN
    // @DisplayName: Torqeedo Servo Output Function
    // @Description: Torqeedo Servo Output Function
    // @Values: 70:Throttle,73:ThrottleLeft,74:ThrottleRight
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("SERVO_FN", 8, AP_Torqeedo_Params, servo_fn, (int16_t)SRV_Channel::k_throttle),

    AP_GROUPEND
};

AP_Torqeedo_Params::AP_Torqeedo_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}
