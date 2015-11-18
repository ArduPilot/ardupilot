// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

/* Functions in this file:
	void set_next_WP(const AP_Mission::Mission_Command& cmd)
	void set_guided_WP(void)
	void init_home()
	void restart_nav()
************************************************************ 
*/


/*
 *  set_next_WP - sets the target location the vehicle should fly to
 */
void Rover::set_next_WP(const struct Location& loc)
{
	// copy the current WP into the OldWP slot
	// ---------------------------------------
	prev_WP = next_WP;

	// Load the next_WP slot
	// ---------------------
	next_WP = loc;

    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text(MAV_SEVERITY_NOTICE, "Resetting previous WP");
        prev_WP = current_loc;
    }

	// this is handy for the groundstation
	wp_totalDistance 	= get_distance(current_loc, next_WP);
	wp_distance 		= wp_totalDistance;
}

void Rover::set_guided_WP(void)
{
	// copy the current location into the OldWP slot
	// ---------------------------------------
	prev_WP = current_loc;

	// Load the next_WP slot
	// ---------------------
	next_WP = guided_WP;

	// this is handy for the groundstation
	wp_totalDistance 	= get_distance(current_loc, next_WP);
	wp_distance 		= wp_totalDistance;
}

// run this at setup on the ground
// -------------------------------
void Rover::init_home()
{
    if (!have_position) {
        // we need position information
        return;
    }

	gcs_send_text(MAV_SEVERITY_INFO, "Init HOME");

    ahrs.set_home(gps.location());
	home_is_set = HOME_SET_NOT_LOCKED;
	Log_Write_Home_And_Origin();
    GCS_MAVLINK::send_home_all(gps.location());

	// Save Home to EEPROM
	mission.write_home_to_storage();

	// Save prev loc
	// -------------
	next_WP = prev_WP = home;

	// Load home for a default guided_WP
	// -------------
	guided_WP = home;
}

void Rover::restart_nav()
{  
    g.pidSpeedThrottle.reset_I();
    prev_WP = current_loc;
    mission.start_or_resume();
}

/*
  update home location from GPS
  this is called as long as we have 3D lock and the arming switch is
  not pushed
*/
void Rover::update_home()
{
    if (home_is_set == HOME_SET_NOT_LOCKED) {
        ahrs.set_home(gps.location());
        Log_Write_Home_And_Origin();
        GCS_MAVLINK::send_home_all(gps.location());
    }
    barometer.update_calibration();
}
