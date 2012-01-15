// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static byte navigate()
{
	// waypoint distance from plane in meters
	// ---------------------------------------
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
	target_bearing 			= get_bearing(&current_loc, &next_WP);
	home_to_copter_bearing 	= get_bearing(&home, &current_loc);

	// nav_bearing will includes xtrac correction
	// ------------------------------------------
	nav_bearing = target_bearing;

	return 1;
}

static bool check_missed_wp()
{
	int32_t temp;
	temp = target_bearing - original_target_bearing;
	temp = wrap_180(temp);
	return (abs(temp) > 10000);	//we pased the waypoint by 10 °
}

// ------------------------------

static void calc_XY_velocity(){
	// offset calculation of GPS speed:
	// used for estimations below 1.5m/s
	// our GPS is about 1m per
	static int32_t last_longitude = 0;
	static int32_t last_latitude  = 0;

	//static int16_t x_speed_old = 0;
	//static int16_t y_speed_old = 0;

	// y_GPS_speed positve = Up
	// x_GPS_speed positve = Right

	// this speed is ~ in cm because we are using 10^7 numbers from GPS
	float tmp = 1.0/dTnav;

	// straightforward approach:
	///*
	int16_t	x_estimate	= (float)(g_gps->longitude - last_longitude) * tmp;
	int16_t	y_estimate	= (float)(g_gps->latitude  - last_latitude)  * tmp;

	// slight averaging filter
	x_GPS_speed = (x_GPS_speed  + x_estimate) >> 1;
	y_GPS_speed = (y_GPS_speed  + y_estimate) >> 1;
	//*/

	/*
	// Ryan Beall's forward estimator:
	int16_t	x_speed_new = (float)(g_gps->longitude - last_longitude) * tmp;
	int16_t	y_speed_new = (float)(g_gps->latitude  - last_latitude)  * tmp;

	x_GPS_speed 	= x_speed_new + (x_speed_new - x_speed_old);
	y_GPS_speed 	= y_speed_new + (y_speed_new - y_speed_old);

	x_speed_old 	= x_speed_new;
	y_speed_old 	= y_speed_new;
	*/

	last_longitude 	= g_gps->longitude;
	last_latitude 	= g_gps->latitude;
}

static void calc_location_error(struct Location *next_loc)
{
	/*
	Becuase we are using lat and lon to do our distance errors here's a quick chart:
	100 	= 1m
	1000 	= 11m	 = 36 feet
	1800 	= 19.80m = 60 feet
	3000 	= 33m
	10000 	= 111m
	*/

	// X Error
	long_error	= (float)(next_loc->lng - current_loc.lng) * scaleLongDown;   // 500 - 0 = 500 Go East

	// Y Error
	lat_error	= next_loc->lat - current_loc.lat;							// 500 - 0 = 500 Go North
}

/*
//static void calc_loiter3(int x_error, int y_error)
{
	static int32_t	gps_lat_I = 0;
	static int32_t	gps_lon_I = 0;

	// If close to goal <1m reset the I term
	if (abs(x_error) < 50)
		gps_lon_I = 0;
	if (abs(y_error) < 50)
		gps_lat_I = 0;

	gps_lon_I += x_error;
	gps_lat_I += y_error;

	gps_lon_I = constrain(gps_lon_I,-3000,3000);
	gps_lat_I = constrain(gps_lat_I,-3000,3000);

	int16_t lon_P = 1.2 	* (float)x_error;
	int16_t lon_I = 0.1 	* (float)gps_lon_I;  //.1
	int16_t lon_D = 3 		* x_GPS_speed ; // this controls the small bumps

	int16_t lat_P = 1.2 	* (float)y_error;
	int16_t lat_I = 0.1 	* (float)gps_lat_I;
	int16_t lat_D = 3 		* y_GPS_speed ;

	//limit of terms
	lon_I = constrain(lon_I,-3000,3000);
	lat_I = constrain(lat_I,-3000,3000);
	lon_D = constrain(lon_D,-500,500);  //this controls the long distance dampimg
	lat_D = constrain(lat_D,-500,500);  //this controls the long distance dampimg

	nav_lon 	= lon_P  + lon_I - lon_D;
	nav_lat 	= lat_P  + lat_I - lat_D;

	Serial.printf("%d, %d, %d, %d, %d, %d\n",
				lon_P, lat_P,
				lon_I, lat_I,
				lon_D, lat_D);

}
*/

#define NAV_ERR_MAX 800
static void calc_loiter(int x_error, int y_error)
{
	int16_t lon_PI 	= g.pi_loiter_lon.get_pi(x_error, dTnav);
	int16_t lon_D 	= 3 * x_actual_speed ; // this controls the small bumps

	int16_t lat_PI 	= g.pi_loiter_lat.get_pi(y_error, dTnav);
	int16_t lat_D 	= 3 * y_actual_speed ;

	//limit of terms
	lon_D = constrain(lon_D,-500,500);
	lat_D = constrain(lat_D,-500,500);

	nav_lon		= constrain(lon_PI - lon_D, -2500, 2500);
	nav_lat		= constrain(lat_PI - lat_D, -2500, 2500);
}


static void calc_loiter1(int x_error, int y_error)
{
	// East/West
	x_error 				= constrain(x_error, -NAV_ERR_MAX, NAV_ERR_MAX);	//800
	int16_t x_target_speed 	= g.pi_loiter_lon.get_p(x_error);
	int16_t x_iterm 		= g.pi_loiter_lon.get_i(x_error, dTnav);
	x_rate_error 			= x_target_speed - x_actual_speed;
	nav_lon_p		 		= g.pi_nav_lon.get_p(x_rate_error);
	nav_lon_p				= constrain(nav_lon_p, -1200, 1200);
	nav_lon					= nav_lon_p + x_iterm;
	nav_lon					= constrain(nav_lon, -2500, 2500);

	// North/South
	y_error 				= constrain(y_error, -NAV_ERR_MAX, NAV_ERR_MAX);
	int16_t y_target_speed 	= g.pi_loiter_lat.get_p(y_error);
	int16_t y_iterm 		= g.pi_loiter_lat.get_i(y_error, dTnav);
	y_rate_error 			= y_target_speed - y_actual_speed;
	nav_lat_p	 			= g.pi_nav_lat.get_p(y_rate_error);
	nav_lat_p				= constrain(nav_lat_p, -1200, 1200);
	nav_lat					= nav_lat_p + y_iterm;
	nav_lat					= constrain(nav_lat, -2500, 2500);

	/*
	int8_t ttt = 1.0/dTnav;
	int16_t t2 = g.pi_nav_lat.get_integrator();

				// 1   2   3      4   5   6   7   8   9   10
	Serial.printf("%d, %d, %d,    %d, %d, %d, %d, %d, %d, %d\n",
					wp_distance, 	//1
					y_error,		//2
					y_GPS_speed,	//3

					y_actual_speed,	//4  ;
					y_target_speed,	//5
					y_rate_error,	//6
					nav_lat_p,		//7
					nav_lat,		//8
					y_iterm,		//9
					t2);			//10
	//*/

	/*
	int16_t t1 = g.pi_nav_lon.get_integrator(); // X
	Serial.printf("%d, %1.4f, %d, %d, %d, %d, %d, %d, %d, %d\n",
					wp_distance, 	//1
					dTnav,			//2
					x_error,		//3
					x_GPS_speed,	//4
					x_actual_speed,	//5
					x_target_speed,	//6
					x_rate_error,	//7
					nav_lat,		//8
					x_iterm,		//9
					t1);			//10
	//*/
}

//wp_distance,ttt, y_error, y_GPS_speed, y_actual_speed, y_target_speed, y_rate_error, nav_lat, y_iterm, t2


#define ERR_GAIN .01
// called at 50hz
static void estimate_velocity()
{
	// we need to extimate velocity when below GPS threshold of 1.5m/s
	//if(g_gps->ground_speed < 120){
		// some smoothing to prevent bumpy rides
		x_actual_speed = (x_actual_speed * 15 + x_GPS_speed) / 16;
		y_actual_speed = (y_actual_speed * 15 + y_GPS_speed) / 16;

		// integration of nav_p angle
		//x_actual_speed += (nav_lon_p >>2);
		//y_actual_speed += (nav_lat_p >>2);

		// this is just what worked best in SIM
		//x_actual_speed = (x_actual_speed * 2 + x_GPS_speed * 1) / 4;
		//y_actual_speed = (y_actual_speed * 2 + y_GPS_speed * 1) / 4;

	//}else{
		// less smoothing needed since the GPS already filters
	//	x_actual_speed = (x_actual_speed * 3 + x_GPS_speed) / 4;
	//	y_actual_speed = (y_actual_speed * 3 + y_GPS_speed) / 4;
	//}
}

// this calculation rotates our World frame of reference to the copter's frame of reference
// We use the DCM's matrix to precalculate these trig values at 50hz
static void calc_loiter_pitch_roll()
{
	//Serial.printf("ys %ld, cx %1.4f, _cx %1.4f | sy %1.4f, _sy %1.4f\n", dcm.yaw_sensor, cos_yaw_x, _cos_yaw_x, sin_yaw_y, _sin_yaw_y);
	// rotate the vector
	nav_roll 	= (float)nav_lon * sin_yaw_y - (float)nav_lat * cos_yaw_x;
	nav_pitch 	= (float)nav_lon * cos_yaw_x + (float)nav_lat * sin_yaw_y;

	// flip pitch because forward is negative
	nav_pitch = -nav_pitch;
}

static int16_t calc_desired_speed(int16_t max_speed)
{
	/*
			   |< WP Radius
	0  1   2   3   4   5   6   7   8m
	...|...|...|...|...|...|...|...|
		  100  |  200	  300	  400cm/s
	           |  		 		            +|+
	           |< we should slow to 1.5 m/s as we hit the target
	*/

	// max_speed is default 600 or 6m/s
	// (wp_distance * 50) = 1/2 of the distance converted to speed
	// wp_distance is always in m/s and not cm/s - I know it's stupid that way
	// for example 4m from target = 200cm/s speed
	// we choose the lowest speed based on disance
	max_speed 		= min(max_speed, (wp_distance * 50));

	// limit the ramp up of the speed
	// waypoint_speed_gov is reset to 0 at each new WP command
	if(waypoint_speed_gov < max_speed){
		waypoint_speed_gov += (int)(50.0 * dTnav); // increase at .5/ms

		// go at least 50cm/s
		max_speed 		= max(50, waypoint_speed_gov);
		// limit with governer
		max_speed 		= min(max_speed, waypoint_speed_gov);
	}

	return max_speed;
}

static void calc_nav_rate(int max_speed)
{
	// push us towards the original track
	update_crosstrack();

	// nav_bearing includes crosstrack
	float temp 		= (9000 - nav_bearing) * RADX100;

	x_rate_error 		= (cos(temp) * max_speed) - x_actual_speed; // 413
	x_rate_error 		= constrain(x_rate_error, -1000, 1000);
	int16_t x_iterm 	= g.pi_loiter_lon.get_i(x_rate_error, dTnav);
	nav_lon_p		 	= g.pi_nav_lon.get_p(x_rate_error);
	nav_lon				= nav_lon_p + x_iterm;
	nav_lon				= constrain(nav_lon, -3000, 3000);


	y_rate_error 		= (sin(temp) * max_speed) - y_actual_speed; // 413
	y_rate_error 		= constrain(y_rate_error, -1000, 1000);	// added a rate error limit to keep pitching down to a minimum
	int16_t y_iterm 	= g.pi_loiter_lat.get_i(y_rate_error, dTnav);
	nav_lat_p		 	= g.pi_nav_lat.get_p(y_rate_error);
	nav_lat				= nav_lat_p + y_iterm;
	nav_lat				= constrain(nav_lat, -3000, 3000);

	/*
	Serial.printf("max_sp %d,\t x_sp %d, y_sp %d,\t x_re: %d, y_re: %d, \tnav_lon: %d, nav_lat: %d, Xi:%d, Yi:%d, \t XE %d \n",
					max_speed,
					x_actual_speed,
					y_actual_speed,
					x_rate_error,
					y_rate_error,
					nav_lon,
					nav_lat,
					x_iterm,
					y_iterm,
					crosstrack_error);
	//*/


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
	if (abs(wrap_180(target_bearing - original_target_bearing)) < 4500) {	 // If we are too far off or too close we don't do track following
		float temp = (target_bearing - original_target_bearing) * RADX100;
		crosstrack_error = sin(temp) * (wp_distance * g.crosstrack_gain);	 // Meters we are off track line
		nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
		nav_bearing = wrap_360(nav_bearing);
	}else{
		nav_bearing = target_bearing;
	}
}

static int32_t get_altitude_error()
{
	// Next_WP alt is our target alt
	// It changes based on climb rate
	// until it reaches the target_altitude
	return next_WP.alt - current_loc.alt;
}

static void clear_new_altitude()
{
	alt_change_flag = REACHED_ALT;
}

static void set_new_altitude(int32_t _new_alt)
{
	// just to be clear
	next_WP.alt = current_loc.alt;

	// for calculating the delta time
	alt_change_timer = millis();

	// save the target altitude
	target_altitude = _new_alt;

	// reset our altitude integrator
	alt_change = 0;

	// save the original altitude
	original_altitude = current_loc.alt;

	// to decide if we have reached the target altitude
	if(target_altitude > original_altitude){
		// we are below, going up
		alt_change_flag = ASCENDING;
		Serial.printf("go up\n");
	}else if(target_altitude < original_altitude){
		// we are above, going down
		alt_change_flag = DESCENDING;
		Serial.printf("go down\n");
	}else{
		// No Change
		alt_change_flag = REACHED_ALT;
		Serial.printf("reached alt\n");
	}

	//Serial.printf("new alt: %d Org alt: %d\n", target_altitude, original_altitude);
}

static int32_t get_new_altitude()
{
	// returns a new next_WP.alt

	if(alt_change_flag == ASCENDING){
		// we are below, going up
		if(current_loc.alt >=  target_altitude){
			alt_change_flag = REACHED_ALT;
		}

		// we shouldn't command past our target
		if(next_WP.alt >=  target_altitude){
			return target_altitude;
		}
	}else if (alt_change_flag == DESCENDING){
		// we are above, going down
		if(current_loc.alt <=  target_altitude)
			alt_change_flag = REACHED_ALT;

		// we shouldn't command past our target
		if(next_WP.alt <=  target_altitude){
			return target_altitude;
		}
	}

	// if we have reached our target altitude, return the target alt
	if(alt_change_flag == REACHED_ALT){
		return target_altitude;
	}

	int32_t diff 	= abs(next_WP.alt - target_altitude);
	int8_t			_scale 	= 3;

	if (next_WP.alt < target_altitude){
		// we are below the target alt
		if(diff < 200){
			_scale = 5;
		} else {
			_scale = 4;
		}
	}else {
		// we are above the target, going down
		if(diff < 600){
			_scale = 4;
		}
		if(diff < 300){
			_scale = 5;
		}
	}

	int32_t change = (millis() - alt_change_timer) >> _scale;

	if(alt_change_flag == ASCENDING){
		alt_change += change;
	}else{
		alt_change -= change;
	}
	// for generating delta time
	alt_change_timer = millis();

	return original_altitude + alt_change;
}


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
//static int32_t get_alt_distance(struct Location *loc1, struct Location *loc2)
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
