#include "mode.h"
#include "Plane.h"

bool ModeManual::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    return true;
}

void ModeManual::_exit()
{
    if (plane.g.auto_trim > 0) {
        plane.trim_radio();
    }
}

