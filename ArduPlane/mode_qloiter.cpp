#include "mode.h"
#include "Plane.h"

bool ModeQLoiter::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_navigation_mode = false;
    if (!plane.quadplane.init_mode()) {
        plane.control_mode = plane.previous_mode;
    } else {
        plane.auto_throttle_mode = false;
        plane.auto_state.vtol_mode = true;
    }

    return true;
}

