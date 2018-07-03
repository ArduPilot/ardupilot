#include "mode.h"
#include "Plane.h"

bool ModeFBWA::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    return true;
}

void ModeFBWA::update()
{
    fbwa_update();
}
