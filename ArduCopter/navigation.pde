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
	static int32_t last_latutude  = 0;

	// y_GPS_speed positve = Up
	// x_GPS_speed positve = Right

	// this speed is ~ in cm because we are using 10^7 numbers from GPS
	float tmp = 1.0/dTnav;
	//int8_t tmp = 5;

	int16_t	x_diff	= (g_gps->longitude - last_longitude) * tmp;
	int16_t	y_diff	= (g_gps->latitude  - last_latutude) * tmp;

	// filter
	x_GPS_speed = (x_GPS_speed * 3 + x_diff) / 4;
	y_GPS_speed = (y_GPS_speed * 3 + y_diff) / 4;

	// Above simply works better than GPS groundspeed
	// which is proving to be problematic

	/*if(g_gps->ground_speed > 120){
		// Derive X/Y speed from GPS
		// this is far more accurate when traveling about 1.5m/s
		float temp		= g_gps->ground_course * RADX100;
		x_GPS_speed 	= sin(temp) * (float)g_gps->ground_speed;
		y_GPS_speed 	= cos(temp) * (float)g_gps->ground_speed;
	}*/

	last_longitude 	= g_gps->longitude;
	last_latutude 	= g_gps->latitude;

	//Serial.printf("GS: %d  \tx:%d \ty:%d\n", g_gps->ground_speed, x_GPS_speed, y_GPS_speed);
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


#define NAV_ERR_MAX 800
static void calc_loiter(int x_error, int y_error)
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

#if WIND_COMP_STAB == 1
static void calc_wind_compensation()
{
	// this idea is a function that converts user input into I term for position hold
	// the concept is simple. The iterms always act upon flight no matter what mode were in.
	// when our velocity is 0, we call this function to update our iterms
	// otherwise we slowly reduce out iterms to 0

	// take the pitch and roll of the copter and,
	float roll 	=  dcm.roll_sensor;
	float pitch = -dcm.pitch_sensor; // flip pitch to make positive pitch forward

	// rotate it to eliminate yaw of Copter
	int32_t roll_tmp 	= roll *  sin_yaw_y  - pitch * -cos_yaw_x;
	int32_t pitch_tmp 	= roll * -cos_yaw_x  + pitch *  sin_yaw_y;

	roll_tmp  = constrain(roll_tmp, -2000, 2000);
	pitch_tmp = constrain(pitch_tmp, -2000, 2000);

	// filter the input and apply it to out integrator value
	// nav_lon and nav_lat will be applied to normal flight

	// This filter is far too fast!!!
	//nav_lon = ((int32_t)g.pi_loiter_lon.get_integrator() * 15 + roll_tmp) / 16;
	//nav_lat = ((int32_t)g.pi_loiter_lat.get_integrator() * 15 + pitch_tmp) / 16;

	nav_lon = g.pi_loiter_lon.get_integrator();
	nav_lat = g.pi_loiter_lat.get_integrator();

	// Maybe a slower filter would work?
	if(g.pi_loiter_lon.get_integrator() > roll_tmp){
		g.pi_loiter_lon.set_integrator(nav_lon - 5);
	}else if(g.pi_loiter_lon.get_integrator() < roll_tmp){
		g.pi_loiter_lon.set_integrator(nav_lon + 5);
	}
	if(g.pi_loiter_lat.get_integrator() > pitch_tmp){
		g.pi_loiter_lat.set_integrator(nav_lat - 5);
	}else if(g.pi_loiter_lat.get_integrator() < pitch_tmp){
		g.pi_loiter_lat.set_integrator(nav_lat + 5);
	}

	// save smoothed input to integrator
	g.pi_loiter_lon.set_integrator(nav_lon);		// X
	g.pi_loiter_lat.set_integrator(nav_lat);		// Y

	//Serial.printf("build wind iterm X:%d  Y:%d, r:%d, p:%d\n",
	//				nav_lon,
	//				nav_lat,
	//				nav_roll,
	//				nav_pitch);
}

static void reduce_wind_compensation()
{
	//slow degradation of iterms
	float tmp;

	tmp = g.pi_loiter_lon.get_integrator();
	tmp *= .98;
	g.pi_loiter_lon.set_integrator(tmp);		// X

	tmp = g.pi_loiter_lat.get_integrator();
	tmp *= .98;
	g.pi_loiter_lat.set_integrator(tmp);		// Y

	// debug
	//int16_t t1 = g.pi_loiter_lon.get_integrator();
	//int16_t t2 = g.pi_loiter_lon.get_integrator();

	//Serial.printf("reduce wind iterm X:%d  Y:%d \n",
	//				t1,
	//				t2);
}
#endif

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
	return next_WP.alt - current_loc.alt;
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
