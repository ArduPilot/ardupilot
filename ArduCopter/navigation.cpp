#include "Copter.h"

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    update_super_simple_bearing(false);
}

// distance between vehicle and home in m
float Copter::home_distance_m()
{
    if (position_ok()) {
        _home_distance_m = current_loc.get_distance(ahrs.get_home());
    }
    return _home_distance_m;
}

// The location of home in relation to the vehicle in centi-degrees
float Copter::home_bearing_rad()
{
    if (position_ok()) {
        _home_bearing_rad = current_loc.get_bearing(ahrs.get_home());
    }
    return _home_bearing_rad;
}
