// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
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
		//gcs.send_text_P(SEVERITY_HIGH,PSTR("<navigate> WP error - distance < 0"));
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

static bool check_missed_wp()
{
	long temp 	= target_bearing - saved_target_bearing;
	temp 		= wrap_180(temp);
	return (abs(temp) > 10000);	//we pased the waypoint by 10 °
}

static int
get_nav_throttle(long error)
{
	int throttle;

	// limit error to prevent I term run up
	error = constrain(error, -600,600);

	throttle = g.pid_throttle.get_pid(error, delta_ms_medium_loop, 1.0);
	throttle = g.throttle_cruise + constrain(throttle, -80, 80);

	// failed experiment
	//int tem = alt_hold_velocity();
	//throttle -= tem;

	return throttle;
}

// ------------------------------

// long_error, lat_error
static void calc_loiter_nav2()
{
	/*
	Becuase we are using lat and lon to do our distance errors here's a quick chart:
	100 	= 1m
	1000 	= 11m	 = 36 feet
	1800 	= 19.80m = 60 feet
	3000 	= 33m
	10000 	= 111m
	pitch_max = 22° (2200)
	*/

	// X ROLL
	long_error	= (float)(next_WP.lng - current_loc.lng) * scaleLongDown;   // 500 - 0 = 500 roll EAST

	// Y PITCH
	lat_error	= current_loc.lat - next_WP.lat;							// 0 - 500 = -500 pitch NORTH

	// constrain input, not output to let I term ramp up and do it's job again wind
	long_error	= constrain(long_error, -loiter_error_max, loiter_error_max); // +- 20m max error
	lat_error	= constrain(lat_error,  -loiter_error_max, loiter_error_max); // +- 20m max error
}

// sets nav_lon, nav_lat
static void calc_rate_nav2(int target_x_speed, int target_y_speed)
{
	// find the rates:
	// calc the cos of the error to tell how fast we are moving towards the target in cm
	int y_speed 	= (float)g_gps->ground_speed * cos(radians((float)g_gps->ground_course/100.0));
	int y_error 	= constrain(target_y_speed - y_speed, -1000, 1000);

	// calc the sin of the error to tell how fast we are moving laterally to the target in cm
	int x_speed 	= (float)g_gps->ground_speed  * sin(radians((float)g_gps->ground_course/100.0));
	int x_error 	= constrain(target_x_speed - x_speed, -1000, 1000);

	// how fast should we be going?
	nav_lat 		+= g.pid_nav_lat.get_pid(y_error, dTnav, 1.0);
	nav_lat 		>>= 1; // divide by two for smooting

	nav_lon 		+= g.pid_nav_lon.get_pid(x_error, dTnav, 1.0);
	nav_lon			>>= 1; // divide by two for smooting

	//Serial.printf("dTnav: %ld, gs: %d, err: %d, int: %d, pitch: %ld", dTnav,  targetspeed, error, (int)g.pid_nav_wp.get_integrator(), (long)nav_lat);

	// limit our output
	nav_lat	= constrain(nav_lat, 	-3500, 3500); // +- max error
	nav_lon	= constrain(nav_lon, 	-3500, 3500); // +- max error
}


// ------------------------------

//nav_lon, nav_lat
static void calc_loiter_nav()
{
	/*
	Becuase we are using lat and lon to do our distance errors here's a quick chart:
	100 	= 1m
	1000 	= 11m	 = 36 feet
	1800 	= 19.80m = 60 feet
	3000 	= 33m
	10000 	= 111m
	pitch_max = 22° (2200)
	*/

	// X ROLL
	long_error	= (float)(next_WP.lng - current_loc.lng) * scaleLongDown;   // 500 - 0 = 500 roll EAST

	// Y PITCH
	lat_error	= current_loc.lat - next_WP.lat;							// 0 - 500 = -500 pitch NORTH

	// constrain input, not output to let I term ramp up and do it's job again wind
	long_error	= constrain(long_error, -loiter_error_max, loiter_error_max); // +- 20m max error
	lat_error	= constrain(lat_error,  -loiter_error_max, loiter_error_max); // +- 20m max error

	nav_lon		= g.pid_nav_lon.get_pid(long_error, dTnav, 1.0);		// X 700 * 2.5 = 1750,
	nav_lat 	= g.pid_nav_lat.get_pid(lat_error,  dTnav, 1.0);		// Y invert lat (for pitch)
}

//nav_lat
static void calc_simple_nav()
{
	// no dampening here in SIMPLE mode
	nav_lat	= constrain((wp_distance * 100), -4500, 4500); // +- 20m max error
	// Scale response by kP
	//nav_lat	*= g.pid_nav_lat.kP();	// 1800 * 2 = 3600 or 36°
}

// sets nav_lon, nav_lat
static void calc_rate_nav(int speed)
{
	// which direction are we moving?
	long heading_error 	= nav_bearing - g_gps->ground_course;
	heading_error 		= wrap_180(heading_error);

	// calc the cos of the error to tell how fast we are moving towards the target in cm
	int targetspeed 	= (float)g_gps->ground_speed * cos(radians((float)heading_error/100));

	// calc the sin of the error to tell how fast we are moving laterally to the target in cm
	int lateralspeed 	= (float)g_gps->ground_speed  * sin(radians((float)heading_error/100));
	//targetspeed			= max(targetspeed, 0);

	// Reduce speed on RTL
	if(control_mode == RTL){
		int tmp 			= min(wp_distance, 80) * 50;
		waypoint_speed 		= min(tmp, speed);
		//waypoint_speed		= max(waypoint_speed, 50);
	}else{
		int tmp 			= min(wp_distance, 200) * 90;
		waypoint_speed 		= min(tmp, speed);
		waypoint_speed		= max(waypoint_speed, 50);
		//waypoint_speed 		= g.waypoint_speed_max.get();
	}

	int error 		= constrain(waypoint_speed - targetspeed, -1000, 1000);

	nav_lat 		+= g.pid_nav_wp.get_pid(error, dTnav, 1.0);
	nav_lat 		>>= 1; // divide by two for smooting

	nav_lon			+= lateralspeed * 2; // 2 is our fake PID gain
	nav_lon			>>= 1; // divide by two for smooting

	//Serial.printf("dTnav: %ld, gs: %d, err: %d, int: %d, pitch: %ld", dTnav,  targetspeed, error, (int)g.pid_nav_wp.get_integrator(), (long)nav_lat);

	// limit our output
	nav_lat	= constrain(nav_lat, 	-3500, 3500); // +- max error
}


// output pitch and roll
// ------------------------------

// nav_roll, nav_pitch
static void calc_loiter_output()
{
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

// nav_roll, nav_pitch
static void calc_nav_output()
{
	// get the sin and cos of the bearing error - rotated 90°
	float sin_nav_y 	= sin(radians((float)(9000 - bearing_error) / 100));
	float cos_nav_x 	= cos(radians((float)(bearing_error - 9000) / 100));

	// rotate the vector
	//nav_roll 	=  (float)nav_lat * cos_nav_x;
	//nav_pitch = -(float)nav_lat * sin_nav_y;
	nav_roll 	=	(float)nav_lon * sin_nav_y	- (float)nav_lat * -cos_nav_x;
	nav_pitch 	=	(float)nav_lon * cos_nav_x 	- (float)nav_lat * sin_nav_y;
}

// ------------------------------
static void calc_bearing_error()
{
	//				  83			99 Yaw  = -16
	bearing_error 	= nav_bearing - dcm.yaw_sensor;
	bearing_error 	= wrap_180(bearing_error);
}

static void calc_altitude_error()
{
	altitude_error 	= next_WP.alt - current_loc.alt;
}

static void calc_altitude_smoothing_error()
{
	// limit climb rates - we draw a straight line between first location and edge of waypoint_radius
	target_altitude = next_WP.alt - ((float)(wp_distance * (next_WP.alt - prev_WP.alt)) / (float)(wp_totalDistance - g.waypoint_radius));

	// stay within a certain range
	if(prev_WP.alt > next_WP.alt){
		target_altitude = constrain(target_altitude, next_WP.alt, prev_WP.alt);
	}else{
		target_altitude = constrain(target_altitude, prev_WP.alt, next_WP.alt);
	}

	altitude_error 	= target_altitude - current_loc.alt;
}

static void update_loiter()
{
	float power;

	if(wp_distance <= g.loiter_radius){
		power = float(wp_distance) / float(g.loiter_radius);
		power = constrain(power, 0.5, 1);
		nav_bearing += (int)(9000.0 * (2.0 + power));
	}else if(wp_distance < (g.loiter_radius + LOITER_RANGE)){
		power = -((float)(wp_distance - g.loiter_radius - LOITER_RANGE) / LOITER_RANGE);
		power = constrain(power, 0.5, 1);			//power = constrain(power, 0, 1);
		nav_bearing -= power * 9000;

	}else{
		update_crosstrack();
		loiter_time = millis();			// keep start time for loiter updating till we get within LOITER_RANGE of orbit

	}
	nav_bearing = wrap_360(nav_bearing);
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
	if (cross_track_test() < 9000) {	 // If we are too far off or too close we don't do track following
		// Meters we are off track line
		crosstrack_error = sin(radians((target_bearing - crosstrack_bearing) / (float)100)) * (float)wp_distance;

		// take meters * 100 to get adjustment to nav_bearing
		long xtrack = g.pid_crosstrack.get_pid(crosstrack_error, dTnav, 1.0) * 100;
		nav_bearing += constrain(xtrack, -g.crosstrack_entry_angle.get(), g.crosstrack_entry_angle.get());
		nav_bearing = wrap_360(nav_bearing);
	}
}

static long cross_track_test()
{
	long temp 	= target_bearing - crosstrack_bearing;
	temp 		= wrap_180(temp);
	return abs(temp);
}

static void reset_crosstrack()
{
	crosstrack_bearing 	= get_bearing(&current_loc, &next_WP);	// Used for track following
}

static long get_altitude_above_home(void)
{
	// This is the altitude above the home location
	// The GPS gives us altitude at Sea Level
	// if you slope soar, you should see a negative number sometimes
	// -------------------------------------------------------------
	return current_loc.alt - home.alt;
}

// distance is returned in meters
static long get_distance(struct Location *loc1, struct Location *loc2)
{
	//if(loc1->lat == 0 || loc1->lng == 0)
	//	return -1;
	//if(loc2->lat == 0 || loc2->lng == 0)
	//	return -1;
	float dlat 		= (float)(loc2->lat - loc1->lat);
	float dlong		= ((float)(loc2->lng - loc1->lng)) * scaleLongDown;
	return sqrt(sq(dlat) + sq(dlong)) * .01113195;
}

static long get_alt_distance(struct Location *loc1, struct Location *loc2)
{
	return abs(loc1->alt - loc2->alt);
}

static long get_bearing(struct Location *loc1, struct Location *loc2)
{
	long off_x = loc2->lng - loc1->lng;
	long off_y = (loc2->lat - loc1->lat) * scaleLongUp;
	long bearing =	9000 + atan2(-off_y, off_x) * 5729.57795;
	if (bearing < 0) bearing += 36000;
	return bearing;
}
