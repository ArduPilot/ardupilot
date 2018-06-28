#include "mode.h"
#include "Plane.h"

bool ModeFBWB::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = false;

    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();

    plane.set_target_altitude_current();

    return true;
}

