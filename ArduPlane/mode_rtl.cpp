#include "mode.h"
#include "Plane.h"

bool ModeRTL::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.prev_WP_loc = plane.current_loc;
    plane.do_RTL(plane.get_RTL_altitude());

    return true;
}

