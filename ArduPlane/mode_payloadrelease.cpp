#include "mode.h"
#include "Plane.h"

bool ModePayloadRelease::_enter()
{
    gcs().send_text(MAV_SEVERITY_INFO,"inside: ModePayloadRelease::_enter");
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    // if (plane.quadplane.available() && plane.quadplane.enable == 2) {
    //     plane.auto_state.vtol_mode = true;
    // } else {
    //     plane.auto_state.vtol_mode = false;
    // }
    // plane.prev_WP_loc = plane.current_loc;
    // plane.next_WP_loc = plane.mission.get_current_nav_cmd().content.location;
    // plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    // plane.mission.start_or_resume();
    // plane.do_payload_release(plane.mission.get_current_nav_cmd());

    // if (hal.util->was_watchdog_armed()) {
    //     if (hal.util->persistent_data.waypoint_num != 0) {
    //         gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
    //         plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
    //         hal.util->persistent_data.waypoint_num = 0;
    //     }
    // }

// #if SOARING_ENABLED == ENABLED
//     plane.g2.soaring_controller.init_cruising();
// #endif

    return true;
}

void ModePayloadRelease::_exit()
{
    gcs().send_text(MAV_SEVERITY_INFO,"inside: ModeModePayloadRelease::_exit");


    // if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
    //     plane.mission.stop();

    //     if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND &&
    //         !plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id))
    //     {
    //         plane.landing.restart_landing_sequence();
    //     }
    // }
    // plane.auto_state.started_flying_in_auto_ms = 0;
}

void ModePayloadRelease::update()
{
    // Location tar_loc = plane.mission.get_current_nav_cmd().content.location;
    gcs().send_text(MAV_SEVERITY_INFO,"inside: ModePayloadRelease::_update");
    plane.steer_state.hold_course_cd = -1;
    // gcs().send_text(MAV_SEVERITY_INFO,"target lat: %d long: %d", tar_loc.lat, tar_loc.lng);
    plane.calc_nav_pitch();
    plane.calc_nav_roll();
    plane.calc_throttle();
    // if(plane.verify_payload_release(plane.mission.get_current_nav_cmd())){
    //     gcs().send_text(MAV_SEVERITY_INFO,"reached the intended wp");
    //     // plane.set_mode(plane.mode_auto, ModeReason::UNKNOWN);
    //     // plane.mission.start_or_resume();

    // }

}

