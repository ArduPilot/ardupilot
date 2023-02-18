#include "AP_Gripper_Backend.h"

#if AP_GRIPPER_ENABLED

#include <AP_Math/AP_Math.h>

void AP_Gripper_Backend::init()
{
    init_gripper();
}

// update - should be called at at least 10hz
void AP_Gripper_Backend::update()
{
    update_gripper();

    // close the gripper again if autoclose_time > 0.0
    if (config.state == AP_Gripper::STATE_RELEASED && (_last_grab_or_release > 0) &&
        (is_positive(config.autoclose_time)) &&
        (AP_HAL::millis() - _last_grab_or_release > (config.autoclose_time * 1000.0))) {
        grab();
    }
}

#endif  // AP_GRIPPER_ENABLED
