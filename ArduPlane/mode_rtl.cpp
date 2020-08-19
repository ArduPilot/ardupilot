#include "mode.h"
#include "Plane.h"

bool ModeRTL::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.prev_WP_loc = plane.current_loc;
    plane.do_RTL(plane.get_RTL_altitude());
    plane.rtl.done_climb = false;

    return true;
}

void ModeRTL::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();

    if (plane.g2.rtl_climb_min > 0) {
        /*
          when RTL first starts limit bank angle to LEVEL_ROLL_LIMIT
          until we have climbed by RTL_CLIMB_MIN meters
         */
        if (!plane.rtl.done_climb && (plane.current_loc.alt - plane.prev_WP_loc.alt)*0.01 > plane.g2.rtl_climb_min) {
            plane.rtl.done_climb = true;
        }
        if (!plane.rtl.done_climb) {
            plane.roll_limit_cd = MIN(plane.roll_limit_cd, plane.g.level_roll_limit*100);
            plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -plane.roll_limit_cd, plane.roll_limit_cd);
        }
    }
}

