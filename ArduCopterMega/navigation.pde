// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
void navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	if (g_gps->fix == 0){
		g_gps->new_data = false;
		return;
	}

	if(next_WP.lat == 0){
		return;
	}

	// waypoint distance from plane
	// ----------------------------
	wp_distance = get_distance(&current_loc, &next_WP);

	if (wp_distance < 0){
		gcs.send_text_P(SEVERITY_HIGH,PSTR("<navigate> WP error - distance < 0"));
		//Serial.println(wp_distance,DEC);
		//print_current_waypoints();
		return;
	}

	// target_bearing is where we should be heading
	// --------------------------------------------
	target_bearing 	= get_bearing(&current_loc, &next_WP);

	// nav_bearing will includes xtrac correction
	// ------------------------------------------
	nav_bearing = target_bearing;
}

bool check_missed_wp()
{
	long temp 	= target_bearing - saved_target_bearing;
	temp 		= wrap_180(temp);
	return (abs(temp) > 10000);	//we pased the waypoint by 10 °
}

#define DIST_ERROR_MAX 1800
void calc_loiter_nav()
{
	/*
	Becuase we are using lat and lon to do our distance errors here's a quick chart:
	100 	= 1m
	1000 	= 11m
	1800 	= 19.80m = 60 feet
	3000 	= 33m
	10000 	= 111m
	pitch_max = 22° (2200)
	*/

	// X ROLL
	long_error	= (float)(next_WP.lng - current_loc.lng) * scaleLongDown;   // 500 - 0 = 500 roll EAST

	// Y PITCH
	lat_error	= current_loc.lat - next_WP.lat;							// 0 - 500 = -500 pitch NORTH


	long_error	= constrain(long_error, -DIST_ERROR_MAX, DIST_ERROR_MAX); // +- 20m max error
	lat_error	= constrain(lat_error,  -DIST_ERROR_MAX, DIST_ERROR_MAX); // +- 20m max error

	// Convert distance into ROLL X
	//nav_lon		= long_error * g.pid_nav_lon.kP();						// 1800 * 2 = 3600 or 36°
	nav_lon		= g.pid_nav_lon.get_pid(long_error, dTnav2, 1.0);

	// PITCH	Y
	//nav_lat		= lat_error * g.pid_nav_lat.kP();						// 1800 * 2 = 3600 or 36°
	nav_lat 	= g.pid_nav_lat.get_pid(lat_error, dTnav2, 1.0);			// invert lat (for pitch)

	// rotate the vector
	nav_roll 	=   (float)nav_lon * sin_yaw_y 	- (float)nav_lat * -cos_yaw_x;
					// BAD
					//NORTH  -1000 *  1			- 1000 *  0 	= -1000	// roll left
					//WEST   -1000 *  0			- 1000 * -1 	=  1000	// roll right  - Backwards
					//EAST   -1000 *  0			- 1000 *  1		= -1000	// roll left   - Backwards
					//SOUTH  -1000 * -1			- 1000 *  0 	=  1000	// roll right

					// GOOD
					//NORTH  -1000 *  1			- 1000 *  0 	= -1000	// roll left
					//WEST   -1000 *  0			- 1000 *  1 	= -1000	// roll right
					//EAST   -1000 *  0			- 1000 * -1		=  1000	// roll left
					//SOUTH  -1000 * -1			- 1000 *  0 	=  1000	// roll right

	nav_pitch 	=  ((float)nav_lon * -cos_yaw_x + (float)nav_lat * sin_yaw_y);
					// BAD
					//NORTH  -1000 *  0			+ 1000 *  1 	=  1000	// pitch back
					//WEST   -1000 * -1			+ 1000 *  0 	=  1000	// pitch back     - Backwards
					//EAST   -1000 *  1			+ 1000 *  0 	= -1000	// pitch forward  - Backwards
 					//SOUTH  -1000 *  0			+ 1000 * -1 	= -1000	// pitch forward

					// GOOD
					//NORTH  -1000 *  0			+ 1000 *  1 	=  1000	// pitch back
					//WEST   -1000 *  1			+ 1000 *  0 	= -1000	// pitch forward
					//EAST   -1000 * -1			+ 1000 *  0 	=  1000	// pitch back
					//SOUTH  -1000 *  0			+ 1000 * -1 	= -1000	// pitch forward

}

void calc_simple_nav()
{
	// no dampening here in SIMPLE mode
	nav_lat	= constrain((wp_distance * 100), -1800, 1800); // +- 20m max error
	// Scale response by kP
	nav_lat	*= g.pid_nav_lat.kP();	// 1800 * 2 = 3600 or 36°
}

void calc_nav_output()
{
	// get the sin and cos of the bearing error - rotated 90°
	sin_nav_y 	= sin(radians((float)(9000 - bearing_error) / 100));
	cos_nav_x 	= cos(radians((float)(bearing_error - 9000) / 100));

	// rotate the vector
	nav_roll 	=  (float)nav_lat * cos_nav_x;
	nav_pitch 	= -(float)nav_lat * sin_nav_y;
}

#define WAYPOINT_SPEED 450

#if NAV_TEST == 0

void calc_rate_nav()
{
	// calc distance error
	nav_lat	= constrain((wp_distance * 100), -1800, 1800); // +- 20m max error

	// Scale response by kP
	nav_lat	*= g.pid_nav_lat.kP();	// 1800 * 2 = 3600 or 36°

	// Scale response by kP
	//long output 	= g.pid_nav_wp.kP() * error;
	int dampening	= g.pid_nav_wp.kD() * (g_gps->ground_speed - last_ground_speed);

	// remember our old speed
	last_ground_speed = g_gps->ground_speed;


	// dampen our response
	nav_lat -= constrain(dampening, 	-1800, 1800); // +- 20m max error
}

#else

// called after we get GPS read
void calc_rate_nav()
{
	// which direction are we moving?
	long target_error 	= target_bearing - g_gps->ground_course;
	target_error 		= wrap_180(target_error);

	// calc the cos of the error to tell how fast we are moving towards the target
	int groundspeed 	= (float)g_gps->ground_speed * cos(radians((float)target_error/100));

	// change to rate error
	// we want to be going 450cm/s
	int error = WAYPOINT_SPEED - groundspeed;

	// Scale response by kP
	long nav_lat 	= g.pid_nav_wp.kP() * error;
	int dampening	= g.pid_nav_wp.kD() * (groundspeed - last_ground_speed);

	// remember our old speed
	last_ground_speed = groundspeed;

	// dampen our response
	nav_lat -= constrain(dampening, 	-1800, 1800); // +- max error

	// limit our output
	nav_lat	= constrain(nav_lat, 	-2500, 2500); // +- max error
}

#endif

void calc_bearing_error()
{
	bearing_error 	= nav_bearing - dcm.yaw_sensor;
	bearing_error 	= wrap_180(bearing_error);
}

void calc_altitude_error()
{
	altitude_error 	= next_WP.alt - current_loc.alt;
}

void calc_altitude_smoothing_error()
{
	// limit climb rates - we draw a straight line between first location and edge of waypoint_radius
	target_altitude = next_WP.alt - ((wp_distance * (next_WP.alt - prev_WP.alt)) / (wp_totalDistance - g.waypoint_radius));

	// stay within a certain range
	if(prev_WP.alt > next_WP.alt){
		target_altitude = constrain(target_altitude, next_WP.alt, prev_WP.alt);
	}else{
		target_altitude = constrain(target_altitude, prev_WP.alt, next_WP.alt);
	}

	altitude_error 	= target_altitude - current_loc.alt;
}

long wrap_360(long error)
{
	if (error > 36000)	error -= 36000;
	if (error < 0)		error += 36000;
	return error;
}

long wrap_180(long error)
{
	if (error > 18000)	error -= 36000;
	if (error < -18000)	error += 36000;
	return error;
}

void update_crosstrack(void)
{
	// Crosstrack Error
	// ----------------
	if (cross_track_test() < 5000) {	 // If we are too far off or too close we don't do track following
		crosstrack_error = sin(radians((target_bearing - crosstrack_bearing) / 100)) * wp_distance;	 // Meters we are off track line
		nav_bearing += constrain(crosstrack_error * g.crosstrack_gain, -g.crosstrack_entry_angle.get(), g.crosstrack_entry_angle.get());
		nav_bearing = wrap_360(nav_bearing);
	}
}

long cross_track_test()
{
	long temp 	= target_bearing - crosstrack_bearing;
	temp 		= wrap_180(temp);
	return abs(temp);
}

void reset_crosstrack()
{
	crosstrack_bearing 	= get_bearing(&current_loc, &next_WP);	// Used for track following
}

long get_altitude_above_home(void)
{
	// This is the altitude above the home location
	// The GPS gives us altitude at Sea Level
	// if you slope soar, you should see a negative number sometimes
	// -------------------------------------------------------------
	return current_loc.alt - home.alt;
}

// distance is returned in meters
long get_distance(struct Location *loc1, struct Location *loc2)
{
	//if(loc1->lat == 0 || loc1->lng == 0)
	//	return -1;
	//if(loc2->lat == 0 || loc2->lng == 0)
	//	return -1;
	float dlat 		= (float)(loc2->lat - loc1->lat);
	float dlong		= ((float)(loc2->lng - loc1->lng)) * scaleLongDown;
	return sqrt(sq(dlat) + sq(dlong)) * .01113195;
}

long get_alt_distance(struct Location *loc1, struct Location *loc2)
{
	return abs(loc1->alt - loc2->alt);
}

long get_bearing(struct Location *loc1, struct Location *loc2)
{
	long off_x = loc2->lng - loc1->lng;
	long off_y = (loc2->lat - loc1->lat) * scaleLongUp;
	long bearing =	9000 + atan2(-off_y, off_x) * 5729.57795;
	if (bearing < 0) bearing += 36000;
	return bearing;
}
