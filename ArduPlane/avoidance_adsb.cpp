
#include <stdio.h>
#include "Plane.h"

void Plane::avoidance_adsb_update(void)
{
    adsb.update();
    avoidance_adsb.update();
}


MAV_COLLISION_ACTION AP_Avoidance_Plane::handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action)
{
    MAV_COLLISION_ACTION actual_action = requested_action;

    // return with action taken
    return actual_action;
}

void AP_Avoidance_Plane::handle_recovery(uint8_t recovery_action)
{
}

// check flight mode is avoid_adsb
bool AP_Avoidance_Plane::check_flightmode(bool allow_mode_change)
{
    // check flight mode
    return (plane.control_mode == AVOID_ADSB);
}

bool AP_Avoidance_Plane::handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // if we got this far we failed to set the new target
    return false;
}

bool AP_Avoidance_Plane::handle_avoidance_perpendicular(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // if we got this far we failed to set the new target
    return false;
}
