#include "AP_CustomControl/AP_CustomControl.h"
#include "AP_CustomControl_config.h"

#if AP_CUSTOMCONTROL_EMPTY_ENABLED

#include "AP_CustomControl_Empty.h"

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_CustomControl_Empty::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: Empty param1
    // @Description: Dummy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM1", 1, AP_CustomControl_Empty, param1, 0.0f),

    // @Param: PARAM2
    // @DisplayName: Empty param2
    // @Description: Dummy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM2", 2, AP_CustomControl_Empty, param2, 0.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dummy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM3", 3, AP_CustomControl_Empty, param3, 0.0f),

    AP_GROUPEND
};

// initialize in the constructor
AP_CustomControl_Empty::AP_CustomControl_Empty(AP_CustomControl& frontend, float dt) :
    AP_CustomControl_Backend(frontend, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update controller
// return controller output
void AP_CustomControl_Empty::update(void)
{
    // Reset controllers when on the ground, e.g. to avoid windup.
    if (_frontend.is_flying) {
        reset();
    }

    // ArduPilot main attitude controller already ran,
    // we don't need to do anything else.

    uint32_t now = AP_HAL::millis();
    if (now - statustest_last_t_ms > 5000.0f) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "empty custom controller working");
        statustest_last_t_ms = now;
    }

    // return what ArduPlane main controller outputted
    _frontend.set_output_scaled(SRV_Channel::k_aileron,SRV_Channels::get_output_scaled(SRV_Channel::k_aileron));
    _frontend.set_output_scaled(SRV_Channel::k_elevator,SRV_Channels::get_output_scaled(SRV_Channel::k_elevator));
    _frontend.set_output_scaled(SRV_Channel::k_throttle,SRV_Channels::get_output_scaled(SRV_Channel::k_throttle));
    _frontend.set_output_scaled(SRV_Channel::k_rudder,SRV_Channels::get_output_scaled(SRV_Channel::k_rudder));
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arduplane main controller
void AP_CustomControl_Empty::reset(void)
{
}

#endif  // AP_CUSTOMCONTROL_EMPTY_ENABLED
