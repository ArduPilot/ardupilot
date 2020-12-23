#include "mode.h"
#include "Plane.h"

bool ModeStabilize::_enter()
{
    plane.auto_throttle_mode = false;

    return true;
}

void ModeStabilize::update()
{
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
}

