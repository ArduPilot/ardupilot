#include "mode.h"
#include "Plane.h"

bool ModeAuto::_enter()
{
    return true;
}

void ModeAuto::_exit()
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();

        if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND &&
            !plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id))
        {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}
