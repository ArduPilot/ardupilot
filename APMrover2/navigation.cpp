#include "Rover.h"

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
void Rover::navigate()
{
    // do not navigate with corrupt data
    // ---------------------------------
    if (!have_position) {
        return;
    }

    if ((next_WP.lat == 0 && next_WP.lng == 0) || (home_is_set == HOME_UNSET)){
        return;
    }

    // waypoint distance from rover
    // ----------------------------
    wp_distance = get_distance(current_loc, next_WP);

    // control mode specific updates to nav_bearing
    // --------------------------------------------
    update_navigation();
}


