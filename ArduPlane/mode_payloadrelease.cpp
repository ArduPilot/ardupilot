#include "mode.h"
#include "Plane.h"

bool ModePayloadRelease::_enter()
{
    gcs().send_text(MAV_SEVERITY_INFO,"inside: ModePayloadRelease::_enter");
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    return true;
}

void ModePayloadRelease::_exit()
{
    gcs().send_text(MAV_SEVERITY_INFO,"inside: ModeModePayloadRelease::_exit");

}

void ModePayloadRelease::update()
{
    // Location tar_loc = plane.mission.get_current_nav_cmd().content.location;
    gcs().send_text(MAV_SEVERITY_INFO,"inside: ModePayloadRelease::_update");

    //These calculations is not required since mode_auto.cpp's update function calls this everytime in normal auto mission.
    //plane.calc_nav_pitch();
    //plane.calc_nav_roll();
    //plane.calc_throttle();
    ////////////////////////////////////////////////////////////////////////////////////////////////

}

