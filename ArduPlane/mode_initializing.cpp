#include "mode.h"
#include "Plane.h"

bool ModeInitializing::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = false;

    return true;
}

