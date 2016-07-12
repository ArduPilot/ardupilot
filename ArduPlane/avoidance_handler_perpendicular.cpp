#include "avoidance_handler.h"

#include <stdio.h>
#include <AP_Notify/AP_Notify.h>

#include "Plane.h"

bool AvoidanceHandler_PERPENDICULAR::new_destination(Location &newdest)
{
    if (! _ahrs.get_position(newdest)) {
        return false;
    }

    Vector3f newdest_neu;
    if (!new_destination_perpendicular(newdest_neu, _ahrs, plane.aparm.airspeed_max * 100, plane.aparm.airspeed_max * 100, _minimum_avoid_height * 100)) {
        return false;
    }

    location_offset(newdest, newdest_neu[0]/100.0f, newdest_neu[1]/100.0f);
    newdest.alt += newdest_neu[2];

    return true;
}


bool AvoidanceHandler_PERPENDICULAR::update()
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


bool AvoidanceHandler_PERPENDICULAR::do_avoid_navigation()
{
    if (!new_destination(guided_WP_loc)) {
        return false;
    }
    plane.guided_WP_loc.lat = guided_WP_loc.lat;
    plane.guided_WP_loc.lng = guided_WP_loc.lng;
    plane.guided_WP_loc.alt = guided_WP_loc.alt;
    plane.set_guided_WP();

    return true;
}
