#include "avoidance_handler.h"

#include <stdio.h>
#include <AP_Notify/AP_Notify.h>

#include "Copter.h"

bool AvoidanceHandler_PERPENDICULAR::new_destination(Vector3f &newdest_neu)
{
    return new_destination_perpendicular(newdest_neu, _ahrs, copter.wp_nav.get_speed_xy(), copter.wp_nav.get_speed_up(), _minimum_avoid_height);
}
