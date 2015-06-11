/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

// called by update navigation at 10Hz
// --------------------
void Plane::update_commands(void)
{
    if(control_mode == AUTO) {
        if (home_is_set != HOME_UNSET) {
            if(mission.state() == AP_Mission::MISSION_RUNNING) {
                mission.update();
            } else {
                // auto_rtl_command should have been set to MAV_CMD_NAV_LOITER_UNLIM by exit_mission
                verify_command(auto_rtl_command);
            }
        }
    }
}

