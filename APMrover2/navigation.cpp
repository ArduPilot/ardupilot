// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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

	if ((next_WP.lat == 0)||(home_is_set==false)){
		return;
	}

	// waypoint distance from rover
	// ----------------------------
	wp_distance = get_distance(current_loc, next_WP);

	if (wp_distance < 0){
		gcs_send_text_P(MAV_SEVERITY_CRITICAL,PSTR("<navigate> WP error - distance < 0"));
		return;
	}

	// control mode specific updates to nav_bearing
	// --------------------------------------------
	update_navigation();
}


