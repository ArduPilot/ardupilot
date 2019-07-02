#include "mode.h"
#include "Plane.h"

bool ModeStabilize::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    return true;
}

void ModeStabilize::update()
{
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
}

