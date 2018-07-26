#include "Copter.h"

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    update_super_simple_bearing(false);

    flightmode->update_navigation();
}

// distance between vehicle and home in cm
uint32_t Copter::home_distance()
{
    if (position_ok()) {
        _home_distance = get_distance_cm(current_loc, ahrs.get_home());
    }
    return _home_distance;
}

// The location of home in relation to the vehicle in centi-degrees
int32_t Copter::home_bearing()
{
    if (position_ok()) {
        _home_bearing = get_bearing_cd(current_loc, ahrs.get_home());
    }
    return _home_bearing;
}
