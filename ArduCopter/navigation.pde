// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static byte navigate()
{

	// waypoint distance from plane
	// ----------------------------
	wp_distance = get_distance(&current_loc, &next_WP);
	home_distance = get_distance(&current_loc, &home);

	if (wp_distance < 0){
		//gcs_send_text_P(SEVERITY_HIGH,PSTR("<navigate> WP error - distance < 0"));
		//Serial.println(wp_distance,DEC);
		//print_current_waypoints();
		return 0;
	}

	// target_bearing is where we should be heading
	// --------------------------------------------
	target_bearing 	= get_bearing(&current_loc, &next_WP);
	home_to_copter_bearing 	= get_bearing(&home, &current_loc);
	return 1;
}

static bool check_missed_wp()
{
	int32_t temp 	= target_bearing - original_target_bearing;
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

#define NAV_ERR_MAX 800

#if LOITER_METHOD == 0
static void calc_loiter(int x_error, int y_error)
{
	x_error = constrain(x_error, -NAV_ERR_MAX, NAV_ERR_MAX);
	y_error = constrain(y_error, -NAV_ERR_MAX, NAV_ERR_MAX);

	int x_target_speed = g.pi_loiter_lon.get_p(x_error);
	int y_target_speed = g.pi_loiter_lat.get_p(y_error);
	int x_iterm = g.pi_loiter_lon.get_i(x_error, dTnav);
	int y_iterm = g.pi_loiter_lat.get_i(y_error, dTnav);

	// find the rates:
	float temp		= g_gps->ground_course * RADX100;

	#ifdef OPTFLOW_ENABLED
		// calc the cos of the error to tell how fast we are moving towards the target in cm
		if(g.optflow_enabled && current_loc.alt < 500 &&  g_gps->ground_speed < 150){
			x_actual_speed 	= optflow.vlon * 10;
			y_actual_speed 	= optflow.vlat * 10;
		}else{
			x_actual_speed 	= (float)g_gps->ground_speed * sin(temp);
			y_actual_speed 	= (float)g_gps->ground_speed * cos(temp);
		}
	#else
		x_actual_speed 	= (float)g_gps->ground_speed * sin(temp);
		y_actual_speed 	= (float)g_gps->ground_speed * cos(temp);
	#endif

	y_rate_error 	= y_target_speed - y_actual_speed; // 413
	y_rate_error 	= constrain(y_rate_error, -1000, 1000);	// added a rate error limit to keep pitching down to a minimum
	nav_lat		 	= g.pi_nav_lat.get_pi(y_rate_error, dTnav);
	nav_lat			= constrain(nav_lat, -3500, 3500);
	nav_lat			+= y_iterm;

	/*Serial.printf("loiter x_actual_speed %d,\tx_rate_error: %d,\tnav_lon: %d,\ty_actual_speed %d,\ty_rate_error: %d,\tnav_lat: %d,\t",
					x_actual_speed,
					x_rate_error,
					nav_lon,
					y_actual_speed,
					y_rate_error,
					nav_lat);
	*/
	x_rate_error 	= x_target_speed - x_actual_speed;
	x_rate_error 	= constrain(x_rate_error, -1000, 1000);
	nav_lon		 	= g.pi_nav_lon.get_pi(x_rate_error, dTnav);
	nav_lon			= constrain(nav_lon, -3500, 3500);
	nav_lon			+= x_iterm;
}

#else
static void calc_loiter2(int x_error, int y_error)
{
	static int last_x_error = 0;
	static int last_y_error = 0;

	x_error = constrain(x_error, -NAV_ERR_MAX, NAV_ERR_MAX); //1200
	y_error = constrain(y_error, -NAV_ERR_MAX, NAV_ERR_MAX);

	int x_target_speed = g.pi_loiter_lon.get_p(x_error);
	int y_target_speed = g.pi_loiter_lat.get_p(y_error);

	int x_iterm = g.pi_loiter_lon.get_i(x_error, dTnav);
	int y_iterm = g.pi_loiter_lat.get_i(y_error, dTnav);

	// find the rates:
	x_actual_speed 	= (float)(last_x_error - x_error)/dTnav;
	y_actual_speed 	= (float)(last_y_error - y_error)/dTnav;

	// save speeds
	last_x_error	= x_error;
	last_y_error	= y_error;

	y_rate_error 	= y_target_speed - y_actual_speed;
	nav_lat		 	= g.pi_nav_lat.get_pi(y_rate_error, dTnav);
	nav_lat			= constrain(nav_lat, -2200, 2200);
	nav_lat			+= y_iterm;

	x_rate_error 	= x_target_speed - x_actual_speed;
	nav_lon		 	= g.pi_nav_lon.get_pi(x_rate_error, dTnav);
	nav_lon			= constrain(nav_lon, -2200, 2200);
	nav_lon			+= x_iterm;
}
#endif

// nav_roll, nav_pitch
static void calc_loiter_pitch_roll()
{

	//float temp  	 = radians((float)(9000 - (dcm.yaw_sensor))/100.0);
	//float _cos_yaw_x = cos(temp);
	//float _sin_yaw_y = sin(temp);

	//Serial.printf("ys %ld, cx %1.4f, _cx %1.4f | sy %1.4f, _sy %1.4f\n", dcm.yaw_sensor, cos_yaw_x, _cos_yaw_x, sin_yaw_y, _sin_yaw_y);

	// rotate the vector
	nav_roll 	=  (float)nav_lon * sin_yaw_y - (float)nav_lat * cos_yaw_x;
	nav_pitch 	=  (float)nav_lon * cos_yaw_x + (float)nav_lat * sin_yaw_y;

	// flip pitch because forward is negative
	nav_pitch = -nav_pitch;

	//Serial.printf("nav_roll %d, nav_pitch %d\n",
	//				nav_roll, nav_pitch);
}

static void calc_nav_rate(int max_speed)
{
	/*
			   |< WP Radius
	0  1   2   3   4   5   6   7   8m
	...|...|...|...|...|...|...|...|
		  100  |  200	  300	  400cm/s
	           |  		 		            +|+
	           |< we should slow to 1.5 m/s as we hit the target
	*/

	// max_speed is default 400 or 4m/s
	// (wp_distance * 50) = 1/2 of the distance converted to speed
	// wp_distance is always in m/s and not cm/s - I know it's stupid that way
	// for example 4m from target = 200cm/s speed
	// we choose the lowest speed based on disance
	max_speed 		= min(max_speed, (wp_distance * 50));

	// limit the ramp up of the speed
	// waypoint_speed_gov is reset to 0 at each new WP command
	if(waypoint_speed_gov < max_speed){
		waypoint_speed_gov += (int)(100.0 * dTnav); // increase at 1.5/ms

		// go at least 50cm/s
		max_speed 		= max(50, waypoint_speed_gov);
		// limit with governer
		max_speed 		= min(max_speed, waypoint_speed_gov);
	}

	//float temp		= radians((target_bearing - g_gps->ground_course)/100.0);
	float temp 		= (target_bearing - g_gps->ground_course) * RADX100;

	// push us towards the original track
	update_crosstrack();

	// heading laterally, we want a zero speed here
	x_actual_speed 	= -sin(temp) * (float)g_gps->ground_speed;
	x_rate_error 	= crosstrack_error -x_actual_speed;
	x_rate_error 	= constrain(x_rate_error, -1400, 1400);
	nav_lon		 	= constrain(g.pi_nav_lon.get_pi(x_rate_error, dTnav), -3500, 3500);
	/*Serial.printf("max_speed %d,\tx_actual_speed %d,\tx_rate_error: %d,\tnav_lon: %d,\ty_actual_speed %d,\ty_rate_error: %d,\tnav_lat: %d,\t",
					max_speed,
					x_actual_speed,
					x_rate_error,
					nav_lon,
					y_actual_speed,
					y_rate_error,
					nav_lat);
	*/

	// heading towards target
	y_actual_speed 	= cos(temp) * (float)g_gps->ground_speed;
	y_rate_error 	= max_speed - y_actual_speed; // 413
	y_rate_error 	= constrain(y_rate_error, -1400, 1400);	// added a rate error limit to keep pitching down to a minimum
	nav_lat		 	= constrain(g.pi_nav_lat.get_pi(y_rate_error, dTnav), -3500, 3500);


	// nav_lat and nav_lon will be rotated to the angle of the quad in calc_nav_pitch_roll()

	/*Serial.printf("max_speed: %d, xspeed: %d, yspeed: %d, x_re: %d, y_re: %d, nav_lon: %ld, nav_lat: %ld  ",
					max_speed,
					x_actual_speed,
					y_actual_speed,
					x_rate_error,
					y_rate_error,
					nav_lon,
					nav_lat);*/
}

static void update_crosstrack(void)
{
	// Crosstrack Error
	// ----------------
	if (cross_track_test() < 4000) {	 // If we are too far off or too close we don't do track following
		float temp = (target_bearing - original_target_bearing) * RADX100;
		//radians((target_bearing - original_target_bearing) / 100)
		crosstrack_error = sin(temp) * wp_distance;	 // Meters we are off track line

		crosstrack_error = constrain(crosstrack_error * g.crosstrack_gain, -1200, 1200);
	}
}

static int32_t cross_track_test()
{
	int32_t temp 	= target_bearing - original_target_bearing;
	temp 		= wrap_180(temp);
	return abs(temp);
}


// nav_roll, nav_pitch
static void calc_nav_pitch_roll()
{
	int32_t angle = wrap_360(dcm.yaw_sensor - target_bearing);
	float temp	 = (9000l - angle) * RADX100;
	//t: 1.5465, t1: -10.9451, t2: 1.5359, t3: 1.5465
	float _cos_yaw_x = cos(temp);
	float _sin_yaw_y = sin(temp);

	// rotate the vector
	nav_roll 	=  (float)nav_lon * _sin_yaw_y - (float)nav_lat * _cos_yaw_x;
	nav_pitch 	=  (float)nav_lon * _cos_yaw_x + (float)nav_lat * _sin_yaw_y;

	// flip pitch because forward is negative
	nav_pitch = -nav_pitch;

	/*Serial.printf("Yaw %d, Tbear:%d, \tangle: %d, \t_cos_yaw_x:%1.4f, _sin_yaw_y:%1.4f, nav_roll:%d, nav_pitch:%d\n",
					dcm.yaw_sensor,
					target_bearing,
					angle,
					_cos_yaw_x,
					_sin_yaw_y,
					nav_roll,
					nav_pitch);*/
}

static int32_t get_altitude_error()
{
	return next_WP.alt - current_loc.alt;
}

/*
//static int get_loiter_angle()
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
}*/

static int32_t wrap_360(int32_t error)
{
	if (error > 36000)	error -= 36000;
	if (error < 0)		error += 36000;
	return error;
}

static int32_t wrap_180(int32_t error)
{
	if (error > 18000)	error -= 36000;
	if (error < -18000)	error += 36000;
	return error;
}

/*
//static int32_t get_crosstrack_correction(void)
{
	// Crosstrack Error
	// ----------------
	if (cross_track_test() < 9000) {	 // If we are too far off or too close we don't do track following

		// Meters we are off track line
		float error = sin(radians((target_bearing - crosstrack_bearing) / (float)100)) * (float)wp_distance;

		// take meters * 100 to get adjustment to nav_bearing
		int32_t _crosstrack_correction = g.pi_crosstrack.get_pi(error, dTnav) * 100;

		// constrain answer to 30° to avoid overshoot
		return constrain(_crosstrack_correction, -g.crosstrack_entry_angle.get(), g.crosstrack_entry_angle.get());
	}
    return 0;
}
*/
/*
//static int32_t cross_track_test()
{
	int32_t temp = wrap_180(target_bearing - crosstrack_bearing);
	return abs(temp);
}
*/
/*
//static void reset_crosstrack()
{
	crosstrack_bearing 	= get_bearing(&current_loc, &next_WP);	// Used for track following
}
*/
/*
//static int32_t get_altitude_above_home(void)
{
	// This is the altitude above the home location
	// The GPS gives us altitude at Sea Level
	// if you slope soar, you should see a negative number sometimes
	// -------------------------------------------------------------
	return current_loc.alt - home.alt;
}
*/
// distance is returned in meters
static int32_t get_distance(struct Location *loc1, struct Location *loc2)
{
	//if(loc1->lat == 0 || loc1->lng == 0)
	//	return -1;
	//if(loc2->lat == 0 || loc2->lng == 0)
	//	return -1;
	float dlat 		= (float)(loc2->lat - loc1->lat);
	float dlong		= ((float)(loc2->lng - loc1->lng)) * scaleLongDown;
	return sqrt(sq(dlat) + sq(dlong)) * .01113195;
}
/*
static int32_t get_alt_distance(struct Location *loc1, struct Location *loc2)
{
	return abs(loc1->alt - loc2->alt);
}
*/
static int32_t get_bearing(struct Location *loc1, struct Location *loc2)
{
	int32_t off_x = loc2->lng - loc1->lng;
	int32_t off_y = (loc2->lat - loc1->lat) * scaleLongUp;
	int32_t bearing =	9000 + atan2(-off_y, off_x) * 5729.57795;
	if (bearing < 0) bearing += 36000;
	return bearing;
}
