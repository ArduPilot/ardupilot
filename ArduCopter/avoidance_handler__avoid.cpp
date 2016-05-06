#include "avoidance_handler.h"

#include <AP_Notify/AP_Notify.h>
#include "Copter.h"

bool AvoidanceHandler__AVOID::update()
{
    if (!AvoidanceHandler__ModeChange::update()) {
        return false;
    }

    // only update our destination once per second
    uint32_t now = AP_HAL::millis();
    if (now - _last_wp_update > 1000) {
        _last_wp_update = now;
        if (!do_avoid_navigation()) {
            return false;
        }
    }

    return true;
}


bool AvoidanceHandler__AVOID::do_avoid_navigation()
{
    Vector3f dest;
    if (!new_destination(dest)) {
        return false;
    }

    // avoid_set_destination takes NEU!
    if (!copter.avoid_set_destination(dest)) {
        return false;
    }

    return true;
}
