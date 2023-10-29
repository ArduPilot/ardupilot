#include "AP_Camera_Params.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_Camera_Params::var_info[] = {

    // 0 should not be used

    // @Param: _TYPE
    // @DisplayName: Camera shutter (trigger) type
    // @Description: how to trigger the camera to take a picture
    // @Values: 0:None, 1:Servo, 2:Relay, 3:GoPro in Solo Gimbal, 4:Mount (Siyi), 5:MAVLink, 6:MAVLinkCamV2, 7:Scripting
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE",  1, AP_Camera_Params, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _DURATION
    // @DisplayName: Camera shutter duration held open
    // @Description: Duration in seconds that the camera shutter is held open
    // @Units: s
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("_DURATION", 2, AP_Camera_Params, trigger_duration, 0.1),

    // @Param: _SERVO_ON
    // @DisplayName: Camera servo ON PWM value
    // @Description: PWM value in microseconds to move servo to when shutter is activated
    // @Units: PWM
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("_SERVO_ON", 3, AP_Camera_Params, servo_on_pwm, 1300),

    // @Param: _SERVO_OFF
    // @DisplayName: Camera servo OFF PWM value
    // @Description: PWM value in microseconds to move servo to when shutter is deactivated
    // @Units: PWM
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("_SERVO_OFF", 4, AP_Camera_Params, servo_off_pwm, 1100),

    // @Param: _TRIGG_DIST
    // @DisplayName: Camera trigger distance
    // @Description: Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight.
    // @User: Standard
    // @Units: m
    // @Range: 0 1000
    AP_GROUPINFO("_TRIGG_DIST", 5, AP_Camera_Params, trigg_dist, 0),

    // @Param: _RELAY_ON
    // @DisplayName: Camera relay ON value
    // @Description: This sets whether the relay goes high or low when it triggers. Note that you should also set RELAY_DEFAULT appropriately for your camera
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("_RELAY_ON", 6, AP_Camera_Params, relay_on, 1),

    // @Param: _INTRVAL_MIN
    // @DisplayName: Camera minimum time interval between photos
    // @Description: Postpone shooting if previous picture was taken less than this many seconds ago
    // @Units: s
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("_INTRVAL_MIN", 7, AP_Camera_Params, interval_min, 0),

    // @Param: _FEEDBAK_PIN
    // @DisplayName: Camera feedback pin
    // @Description: pin number to use for save accurate camera feedback messages. If set to -1 then don't use a pin flag for this, otherwise this is a pin number which if held high after a picture trigger order, will save camera messages when camera really takes a picture. A universal camera hot shoe is needed. The pin should be held high for at least 2 milliseconds for reliable trigger detection.  Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot. See also the CAMx_FEEDBCK_POL option.
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_FEEDBAK_PIN", 8, AP_Camera_Params, feedback_pin, -1),

    // @Param: _FEEDBAK_POL
    // @DisplayName: Camera feedback pin polarity
    // @Description: Polarity for feedback pin. If this is 1 then the feedback pin should go high on trigger. If set to 0 then it should go low
    // @Values: 0:TriggerLow,1:TriggerHigh
    // @User: Standard
    AP_GROUPINFO("_FEEDBAK_POL", 9, AP_Camera_Params, feedback_polarity, 1),

    // @Param: _OPTIONS
    // @DisplayName: Camera options
    // @Description: Camera options bitmask
    // @Bitmask: 0:None,1: Recording Starts at arming and stops at disarming
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 10, AP_Camera_Params, options, 0),

    // @Param: _MNT_INST
    // @DisplayName: Camera Mount instance
    // @Description: Mount instance camera is associated with. 0 means camera and mount have identical instance numbers e.g. camera1 and mount1
    // @User: Standard
    AP_GROUPINFO("_MNT_INST", 11, AP_Camera_Params, mount_instance, 0),

    AP_GROUPEND

};

AP_Camera_Params::AP_Camera_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
