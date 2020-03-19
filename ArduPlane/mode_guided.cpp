#include "mode.h"
#include "Plane.h"

bool ModeGuided::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    plane.guided_WP_loc = plane.current_loc;
    plane.set_guided_WP();

    return true;
}

void ModeGuided::update()
{
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
    } else {

        if (plane.follow_target.lat != 0 && plane.follow_target.lng != 0) {
            plane.next_WP_loc.lat = plane.follow_target.lat;
            plane.next_WP_loc.lng = plane.follow_target.lng;
        }

        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

