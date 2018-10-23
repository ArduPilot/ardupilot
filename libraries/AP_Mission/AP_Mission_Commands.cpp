#include "AP_Mission.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Gripper/AP_Gripper.h>

bool AP_Mission::start_command_do_gripper(const AP_Mission::Mission_Command& cmd)
{
    AP_Gripper *gripper = AP::gripper();
    if (gripper == nullptr) {
        return true;
    }

    // Note: we ignore the gripper num parameter because we only
    // support one gripper
    switch (cmd.content.gripper.action) {
    case GRIPPER_ACTION_RELEASE:
        gripper->release();
        // Log_Write_Event(DATA_GRIPPER_RELEASE);
        gcs().send_text(MAV_SEVERITY_INFO, "Gripper Released");
        break;
    case GRIPPER_ACTION_GRAB:
        gripper->grab();
        // Log_Write_Event(DATA_GRIPPER_GRAB);
        gcs().send_text(MAV_SEVERITY_INFO, "Gripper Grabbed");
        break;
    default:
        // do nothing
        break;
    }

    return true;
}
