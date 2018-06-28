#include "mode.h"
#include "Plane.h"

bool ModeAuto::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    } else {
        plane.auto_state.vtol_mode = false;
    }
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();

    plane.g2.soaring_controller.init_cruising();

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

