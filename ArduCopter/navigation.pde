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

	// nav_bearing will include xtrac correction
	// -----------------------------------------
	//xtrack_enabled = false;
	if(xtrack_enabled){
		nav_bearing = wrap_360(target_bearing + get_crosstrack_correction());
	}else{
		nav_bearing = target_bearing;
	}
}

static bool check_missed_wp()
{
	long temp 	= target_bearing - saved_target_bearing;
	temp 		= wrap_180(temp);
	return (abs(temp) > 10000);	//we pased the waypoint by 10 °
}

// ------------------------------

// long_error, lat_error
static void calc_location_error(struct Location *next_loc)
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
	long_error	= (float)(next_loc->lng - current_loc.lng) * scaleLongDown;   // 500 - 0 = 500 roll EAST

	// Y PITCH
	lat_error	= next_loc->lat - current_loc.lat;							// 0 - 500 = -500 pitch NORTH
}

#define NAV_ERR_MAX 400
static void calc_nav_rate(int x_error, int y_error, int max_speed, int min_speed)
{
	// moved to globals for logging
	//int x_actual_speed, y_actual_speed;
	//int x_rate_error, y_rate_error;
	x_error = constrain(x_error, -NAV_ERR_MAX, NAV_ERR_MAX);
	y_error = constrain(y_error, -NAV_ERR_MAX, NAV_ERR_MAX);

	float scaler = (float)max_speed/(float)NAV_ERR_MAX;
	g.pi_loiter_lat.kP(scaler);
	g.pi_loiter_lon.kP(scaler);

	int x_target_speed = g.pi_loiter_lon.get_pi(x_error, dTnav);
	int y_target_speed = g.pi_loiter_lat.get_pi(y_error, dTnav);

	//Serial.printf("scaler: %1.3f, y_target_speed %d",scaler,y_target_speed);

	if(x_target_speed > 0){
		x_target_speed	= max(x_target_speed, min_speed);
	}else{
		x_target_speed	= min(x_target_speed, -min_speed);
	}

	if(y_target_speed > 0){
		y_target_speed	= max(y_target_speed, min_speed);
	}else{
		y_target_speed	= min(y_target_speed, -min_speed);
	}

	// find the rates:
	// calc the cos of the error to tell how fast we are moving towards the target in cm
	y_actual_speed 	= (float)g_gps->ground_speed * cos(radians((float)g_gps->ground_course/100.0));
	y_rate_error 	= y_target_speed - y_actual_speed; // 413
	y_rate_error 	= constrain(y_rate_error, -250, 250);	// added a rate error limit to keep pitching down to a minimum
	nav_lat		 	= constrain(g.pi_nav_lat.get_pi(y_rate_error, dTnav), -3500, 3500);

	//Serial.printf("yr: %d, nav_lat: %d, int:%d \n",y_rate_error, nav_lat, g.pi_nav_lat.get_integrator());

	// calc the sin of the error to tell how fast we are moving laterally to the target in cm
	x_actual_speed 	= (float)g_gps->ground_speed  * sin(radians((float)g_gps->ground_course/100.0));
	x_rate_error 	= x_target_speed - x_actual_speed;
	x_rate_error 	= constrain(x_rate_error, -250, 250);
	nav_lon		 	= constrain(g.pi_nav_lon.get_pi(x_rate_error, dTnav), -3500, 3500);
}

// nav_roll, nav_pitch
static void calc_nav_pitch_roll()
{
	// rotate the vector
	nav_roll 	=  (float)nav_lon * sin_yaw_y - (float)nav_lat * cos_yaw_x;
	nav_pitch 	=  (float)nav_lon * cos_yaw_x + (float)nav_lat * sin_yaw_y;

	// flip pitch because forward is negative
	nav_pitch = -nav_pitch;
}

// ------------------------------
static void calc_bearing_error()
{
	bearing_error 	= nav_bearing - dcm.yaw_sensor;
	bearing_error 	= wrap_180(bearing_error);
}

static long get_altitude_error()
{
	return next_WP.alt - current_loc.alt;
}

/*
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
*/

static int get_loiter_angle()
{
	float power;
	int angle;

	if(wp_distance <= g.loiter_radius){
		power = float(wp_distance) / float(g.loiter_radius);
		power = constrain(power, 0.5, 1);
		angle = 90.0 * (2.0 + power);
	}else if(wp_distance < (g.loiter_radius + LOITER_RANGE)){
		power = -((float)(wp_distance - g.loiter_radius - LOITER_RANGE) / LOITER_RANGE);
		power = constrain(power, 0.5, 1);			//power = constrain(power, 0, 1);
		angle = power * 90;
	}

	return angle;
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

static long get_crosstrack_correction(void)
{
	// Crosstrack Error
	// ----------------
	if (cross_track_test() < 9000) {	 // If we are too far off or too close we don't do track following

		// Meters we are off track line
		float error = sin(radians((target_bearing - crosstrack_bearing) / (float)100)) * (float)wp_distance;

		// take meters * 100 to get adjustment to nav_bearing
		long _crosstrack_correction = g.pi_crosstrack.get_pi(error, dTnav) * 100;

		// constrain answer to 30° to avoid overshoot
		return constrain(_crosstrack_correction, -g.crosstrack_entry_angle.get(), g.crosstrack_entry_angle.get());
	}
    return 0;
}


static long cross_track_test()
{
	long temp = wrap_180(target_bearing - crosstrack_bearing);
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
