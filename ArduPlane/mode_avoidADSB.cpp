#include "mode.h"
#include "Plane.h"

bool ModeAvoidADSB::_enter()
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

