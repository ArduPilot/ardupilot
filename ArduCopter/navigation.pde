// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
{
	// waypoint distance from plane in cm
	// ---------------------------------------
	wp_distance 	= get_distance(&current_loc, &next_WP);
	home_distance 	= get_distance(&current_loc, &home);

	// target_bearing is where we should be heading
	// --------------------------------------------
	target_bearing 			= get_bearing(&current_loc, &next_WP);
	home_to_copter_bearing 	= get_bearing(&home, &current_loc);
}

static bool check_missed_wp()
{
	int32_t temp;
	temp = target_bearing - original_target_bearing;
	temp = wrap_180(temp);
	return (abs(temp) > 10000);	// we passed the waypoint by 100 degrees
}

// ------------------------------
static void calc_XY_velocity(){
	static int32_t last_longitude = 0;
	static int32_t last_latitude  = 0;

	// called after GPS read
	// offset calculation of GPS speed:
	// used for estimations below 1.5m/s
	// y_GPS_speed positve = Up
	// x_GPS_speed positve = Right

	// initialise last_longitude and last_latitude
	if( last_longitude == 0 && last_latitude == 0 ) {
		last_longitude = g_gps->longitude;
		last_latitude = g_gps->latitude;
	}

	// this speed is ~ in cm because we are using 10^7 numbers from GPS
	float tmp = 1.0/dTnav;

	x_actual_speed 	= (float)(g_gps->longitude - last_longitude)  * scaleLongDown * tmp;
	y_actual_speed	= (float)(g_gps->latitude  - last_latitude)  * tmp;

	last_longitude 	= g_gps->longitude;
	last_latitude 	= g_gps->latitude;

	/*if(g_gps->ground_speed > 150){
		float temp = radians((float)g_gps->ground_course/100.0);
		x_actual_speed = (float)g_gps->ground_speed * sin(temp);
		y_actual_speed = (float)g_gps->ground_speed * cos(temp);
	}*/


	#if INERTIAL_NAV == ENABLED
	// inertial_nav
	xy_error_correction();
	#endif

	current_loc.lng = xLeadFilter.get_position(g_gps->longitude, x_actual_speed);
	current_loc.lat = yLeadFilter.get_position(g_gps->latitude,  y_actual_speed);
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

#define NAV_ERR_MAX 600
#define NAV_RATE_ERR_MAX 250
static void calc_loiter(int x_error, int y_error)
{
	int32_t p,i,d;						// used to capture pid values for logging
	int32_t output;
	int32_t x_target_speed, y_target_speed;

	// East / West
	x_target_speed 	= g.pi_loiter_lon.get_p(x_error);			// calculate desired speed from lon error

#if LOGGING_ENABLED == ENABLED
	// log output if PID logging is on and we are tuning the yaw
	if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_LOITER_KP || g.radio_tuning == CH6_LOITER_KI) ) {
		Log_Write_PID(CH6_LOITER_KP, x_error, x_target_speed, 0, 0, x_target_speed, tuning_value);
	}
#endif


	// calculate rate error
	#if INERTIAL_NAV == ENABLED
	x_rate_error	= x_target_speed - accels_velocity.x;		// calc the speed error
	#else
	x_rate_error	= x_target_speed - x_actual_speed;			// calc the speed error
	#endif


	p				= g.pid_loiter_rate_lon.get_p(x_rate_error);
	i				= g.pid_loiter_rate_lon.get_i(x_rate_error + x_error, dTnav);
	d				= g.pid_loiter_rate_lon.get_d(x_error, dTnav);
	d				= constrain(d, -2000, 2000);

	// get rid of noise
	if(abs(x_actual_speed) < 50){
		d = 0;
	}

	output			= p + i + d;
	nav_lon			= constrain(output, -3000, 3000); 			// 30°

#if LOGGING_ENABLED == ENABLED
	// log output if PID logging is on and we are tuning the yaw
	if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_LOITER_RATE_KP || g.radio_tuning == CH6_LOITER_RATE_KI || g.radio_tuning == CH6_LOITER_RATE_KD) ) {
		Log_Write_PID(CH6_LOITER_RATE_KP, x_rate_error, p, i, d, nav_lon, tuning_value);
	}
#endif

	// North / South
	y_target_speed 	= g.pi_loiter_lat.get_p(y_error);			// calculate desired speed from lat error

#if LOGGING_ENABLED == ENABLED
	// log output if PID logging is on and we are tuning the yaw
	if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_LOITER_KP || g.radio_tuning == CH6_LOITER_KI) ) {
		Log_Write_PID(CH6_LOITER_KP+100, y_error, y_target_speed, 0, 0, y_target_speed, tuning_value);
	}
#endif

	// calculate rate error
	#if INERTIAL_NAV == ENABLED
	y_rate_error	= y_target_speed - accels_velocity.y;		// calc the speed error
	#else
	y_rate_error	= y_target_speed - y_actual_speed;			// calc the speed error
	#endif

	p				= g.pid_loiter_rate_lat.get_p(y_rate_error);
	i				= g.pid_loiter_rate_lat.get_i(y_rate_error + y_error, dTnav);
	d				= g.pid_loiter_rate_lat.get_d(y_error, dTnav);
	d				= constrain(d, -2000, 2000);

	// get rid of noise
	if(abs(y_actual_speed) < 50){
		d = 0;
	}

	output			= p + i + d;
	nav_lat			= constrain(output, -3000, 3000); // 30°

#if LOGGING_ENABLED == ENABLED
	// log output if PID logging is on and we are tuning the yaw
	if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_LOITER_RATE_KP || g.radio_tuning == CH6_LOITER_RATE_KI || g.radio_tuning == CH6_LOITER_RATE_KD) ) {
		Log_Write_PID(CH6_LOITER_RATE_KP+100, y_rate_error, p, i, d, nav_lat, tuning_value);
	}
#endif

	// copy over I term to Nav_Rate
	g.pid_nav_lon.set_integrator(g.pid_loiter_rate_lon.get_integrator());
	g.pid_nav_lat.set_integrator(g.pid_loiter_rate_lat.get_integrator());
}

static void calc_nav_rate(int max_speed)
{
	float temp, temp_x, temp_y;

	// push us towards the original track
	update_crosstrack();

	int16_t cross_speed = crosstrack_error * -g.crosstrack_gain; // scale down crosstrack_error in cm
	// XXX replace above with crosstrack gain.

	cross_speed	= constrain(cross_speed, -200, 200);

	// rotate by 90 to deal with trig functions
	temp 			= (9000l - target_bearing) * RADX100;
	temp_x 			= cos(temp);
	temp_y 			= sin(temp);

	// rotate desired spped vector:
	int16_t x_target_speed = max_speed   * temp_x - cross_speed * temp_y;
	int16_t y_target_speed = cross_speed * temp_x + max_speed   * temp_y;

	// East / West
	x_rate_error 	= x_target_speed - x_actual_speed; // 413
	x_rate_error 	= constrain(x_rate_error, -1000, 1000);
	nav_lon			= g.pid_nav_lon.get_pid(x_rate_error, dTnav);
	temp			= x_target_speed * x_target_speed;
	temp			*= g.tilt_comp;

	if(x_target_speed < 0)
		temp = -temp;
	nav_lon			+= temp;
	nav_lon			= constrain(nav_lon, -3000, 3000);


	// North / South
	y_rate_error 	= y_target_speed - y_actual_speed; // 413
	y_rate_error 	= constrain(y_rate_error, -1000, 1000);	// added a rate error limit to keep pitching down to a minimum
	nav_lat			= g.pid_nav_lat.get_pid(y_rate_error, dTnav);
	temp			= y_target_speed * y_target_speed;
	temp			*= g.tilt_comp;

	if(y_target_speed < 0)
		temp = -temp;
	nav_lat			+= temp;
	nav_lat			= constrain(nav_lat, -3000, 3000);

	// copy over I term to Loiter_Rate
	g.pid_loiter_rate_lon.set_integrator(g.pid_nav_lon.get_integrator());
	g.pid_loiter_rate_lat.set_integrator(g.pid_nav_lat.get_integrator());
}


// this calculation rotates our World frame of reference to the copter's frame of reference
// We use the DCM's matrix to precalculate these trig values at 50hz
static void calc_loiter_pitch_roll()
{
	//Serial.printf("ys %ld, cx %1.4f, _cx %1.4f | sy %1.4f, _sy %1.4f\n", dcm.yaw_sensor, cos_yaw_x, _cos_yaw_x, sin_yaw_y, _sin_yaw_y);
	// rotate the vector
	auto_roll 	= (float)nav_lon * sin_yaw_y - (float)nav_lat * cos_yaw_x;
	auto_pitch 	= (float)nav_lon * cos_yaw_x + (float)nav_lat * sin_yaw_y;

	// flip pitch because forward is negative
	auto_pitch = -auto_pitch;
}

static int16_t calc_desired_speed(int16_t max_speed, bool _slow)
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
	if(_slow){
		max_speed 		= min(max_speed, wp_distance / 2);
		max_speed 		= max(max_speed, 0);
	}else{
		max_speed 		= min(max_speed, wp_distance);
		max_speed 		= max(max_speed, WAYPOINT_SPEED_MIN);	// go at least 100cm/s
	}

	// limit the ramp up of the speed
	// waypoint_speed_gov is reset to 0 at each new WP command
	if(max_speed > waypoint_speed_gov){
		waypoint_speed_gov += (int)(200.0 * dTnav); // increase at .5/ms
		max_speed = waypoint_speed_gov;
	}

	return max_speed;
}


static void update_crosstrack(void)
{
	// Crosstrack Error
	// ----------------
	// If we are too far off or too close we don't do track following
	float temp = (target_bearing - original_target_bearing) * RADX100;
	crosstrack_error = sin(temp) * wp_distance;	 // Meters we are off track line
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

static void force_new_altitude(int32_t _new_alt)
{
	next_WP.alt 	= _new_alt;
	target_altitude = _new_alt;
	alt_change_flag = REACHED_ALT;
}

static void set_new_altitude(int32_t _new_alt)
{
	if(_new_alt == current_loc.alt){
		force_new_altitude(_new_alt);
		return;
	}

	// We start at the current location altitude and gradually change alt
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
		//Serial.printf("go up\n");
	}else if(target_altitude < original_altitude){
		// we are above, going down
		alt_change_flag = DESCENDING;
		//Serial.printf("go down\n");
	}else{
		// No Change
		alt_change_flag = REACHED_ALT;
		//Serial.printf("reached alt\n");
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
	// scale is how we generate a desired rate from the elapsed time
	// a smaller scale means faster rates
	int8_t			_scale 	= 4;

	if (next_WP.alt < target_altitude){
		// we are below the target alt
		if(diff < 200){
			_scale = 4;
		} else {
			_scale = 3;
		}
	}else {
		// we are above the target, going down
		if(diff < 400){
			_scale = 5;
		}
		if(diff < 100){
			_scale = 6;
		}
	}

	// we use the elapsed time as our altitude offset
	// 1000 = 1 sec
	// 1000 >> 4 = 64cm/s descent by default
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

