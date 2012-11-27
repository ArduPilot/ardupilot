// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	if (g_gps->fix == 0)
	{
		g_gps->new_data = false;
		return;
	}

	if ((next_WP.lat == 0)||(home_is_set==false)){
		return;
	}

	// waypoint distance from plane
	// ----------------------------
	wp_distance = get_distance(&current_loc, &next_WP);

	if (wp_distance < 0){
		gcs_send_text_P(SEVERITY_HIGH,PSTR("<navigate> WP error - distance < 0"));
		return;
	}

	// target_bearing is where we should be heading
	// --------------------------------------------
	target_bearing 	= get_bearing_cd(&current_loc, &next_WP);

	// nav_bearing will includes xtrac correction
	// ------------------------------------------
	nav_bearing = target_bearing;

	// control mode specific updates to nav_bearing
	// --------------------------------------------
	update_navigation();
}


static void calc_gndspeed_undershoot()
{
    if (g_gps->status() == GPS::GPS_OK) {
        // Function is overkill, but here in case we want to add filtering later
        groundspeed_undershoot = (g.min_gndspeed > 0) ? (g.min_gndspeed - ground_speed) : 0;
    }
}

static void calc_bearing_error()
{    
	bearing_error = nav_bearing - ahrs.yaw_sensor;
	bearing_error = wrap_180(bearing_error);
}

static long wrap_360(long error)
{
	if (error > 36000)	error -= 36000;
	if (error < 0)		error += 36000;
	return error;
}

static long wrap_180(long error)
{
	if (error > 18000)	error -= 36000;
	if (error < -18000)	error += 36000;
	return error;
}

static void update_crosstrack(void)
{
	// Crosstrack Error
	// ----------------
	if (abs(wrap_180(target_bearing - crosstrack_bearing)) < 4500) {	 // If we are too far off or too close we don't do track following
		crosstrack_error = sin(radians((target_bearing - crosstrack_bearing) / (float)100)) * (float)wp_distance;	 // Meters we are off track line
		nav_bearing += constrain(crosstrack_error * g.crosstrack_gain, -g.crosstrack_entry_angle.get(), g.crosstrack_entry_angle.get());
		nav_bearing = wrap_360(nav_bearing);
	}
}

static void reset_crosstrack()
{
	crosstrack_bearing 	= get_bearing_cd(&prev_WP, &next_WP);	// Used for track following
}

void reached_waypoint()
{       

}

