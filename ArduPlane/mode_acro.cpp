#include "mode.h"
#include "Plane.h"

bool ModeAcro::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;
    plane.acro_state.locked_roll = false;
    plane.acro_state.locked_pitch = false;

    return true;
}

