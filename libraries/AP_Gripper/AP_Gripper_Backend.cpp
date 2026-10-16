#include "AP_Gripper_Backend.h"

#if AP_GRIPPER_ENABLED

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

void AP_Gripper_Backend::init()
{
    init_gripper();
}

// update - should be called at at least 10hz
void AP_Gripper_Backend::update()
{
    update_gripper();

    // close the gripper again if autoclose_time > 0.0 and no hold since the release
    if (config.state == AP_Gripper::STATE_RELEASED && (_last_grab_or_release > 0) &&
        !_hold_requested &&
        (is_positive(config.autoclose_time)) &&
        (AP_HAL::millis() - _last_grab_or_release > (config.autoclose_time * 1000.0))) {
        grab();
    }
}

// hold - leave the output where it is; a grab or release in progress
// still completes. Cutting an EPM pulse short would leave the magnet in
// an unknown state, and moving a position servo could drop the load
void AP_Gripper_Backend::hold()
{
    _hold_requested = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Gripper holding");
    LOGGER_WRITE_EVENT(LogEvent::GRIPPER_HOLD);
}

#endif  // AP_GRIPPER_ENABLED
