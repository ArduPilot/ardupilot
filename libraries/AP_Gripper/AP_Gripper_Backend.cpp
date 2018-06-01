#include "AP_Gripper_Backend.h"

extern const AP_HAL::HAL& hal;

void AP_Gripper_Backend::init()
{
    init_gripper();
}

// update - should be called at at least 10hz
void AP_Gripper_Backend::update()
{
    update_gripper();
}
