#include "Plane.h"
#include "avoidance.h"
#include <AP_Notify/AP_Notify.h>

void Plane::avoidance_update(void)
{
    avoidance.update();
}

#include <stdio.h>

AvoidanceHandler &AP_Avoidance_Plane::handler_for_action(MAV_COLLISION_ACTION action)
{
    if (!plane.is_flying()) {
        // do nothing if we are not flying
        if (action == MAV_COLLISION_ACTION_REPORT) {
            return avoidance_handler_report;
        }
        return avoidance_handler_none;
    }

    switch(action) {
    case MAV_COLLISION_ACTION_NONE:
        return avoidance_handler_none;
    case MAV_COLLISION_ACTION_REPORT:
        return avoidance_handler_report;
    case MAV_COLLISION_ACTION_MOVE_PERPENDICULAR:
        return avoidance_handler_perpendicular;
    default:
        ::fprintf(stderr, "Avoidance action %d not known\n", action);
        internal_error();
    };
    return avoidance_handler_none;
}
