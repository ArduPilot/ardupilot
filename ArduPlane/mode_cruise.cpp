#include "mode.h"
#include "Plane.h"

bool ModeCruise::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = false;
    plane.cruise_state.locked_heading = false;
    plane.cruise_state.lock_timer_ms = 0;

    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();

    plane.set_target_altitude_current();

    return true;
}

