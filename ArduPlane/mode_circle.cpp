#include "mode.h"
#include "Plane.h"

bool ModeCircle::_enter()
{
    // the altitude to circle at is taken from the current altitude
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    plane.next_WP_loc.alt = plane.current_loc.alt;

    return true;
}

