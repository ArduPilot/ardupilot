// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// update_navigation - checks for new GPS updates and invokes navigation routines
static void update_navigation()
{
    static uint32_t nav_last_gps_update = 0;    // the system time of the last gps update
    static uint32_t nav_last_gps_time = 0;      // the time according to the gps
    bool log_output = false;


    // check for new gps data
    if( g_gps->fix && g_gps->time != nav_last_gps_time ) {

        // used to calculate speed in X and Y, iterms
        // ------------------------------------------
        dTnav = (float)(millis() - nav_last_gps_update)/ 1000.0;
        nav_last_gps_update = millis();

        // prevent runup from bad GPS
        dTnav = min(dTnav, 1.0);

        // save GPS time
        nav_last_gps_time = g_gps->time;

        // calculate velocity
        calc_velocity_and_position();

        // signal to create log entry
        log_output = true;
    }

    // calc various navigation values and run controllers if we've received a position update
	// calculate distance, angles to target
	calc_distance_and_bearing();

	calc_location_error(&next_WP);


	// run navigation controllers
	run_navigation_contollers();
	//cliSerial->printf_P(PSTR("nc, "));

	// update log
	if (log_output && (g.log_bitmask & MASK_LOG_NTUN)) {
		Log_Write_Nav_Tuning();
	}

    // reduce nav outputs to zero if we have not received a gps update in 2 seconds
    if( millis() - nav_last_gps_update > 2000 ) {
        // after 12 reads we guess we may have lost GPS signal, stop navigating
        // we have lost GPS signal for a moment. Reduce our error to avoid flyaways
        //auto_roll  >>= 1;
        auto_pitch >>= 1;
    }
}

//*******************************************************************************************************
// calc_velocity_and_filtered_position - velocity in lon and lat directions calculated from GPS position
//       and accelerometer data
// lon_speed expressed in cm/s.  positive numbers mean moving east
// lat_speed expressed in cm/s.  positive numbers when moving north
// Note: we use gps locations directly to calculate velocity instead of asking gps for velocity because
//       this is more accurate below 1.5m/s
// Note: even though the positions are projected using a lead filter, the velocities are calculated
//       from the unaltered gps locations.  We do not want noise from our lead filter affecting velocity
//*******************************************************************************************************
static void calc_velocity_and_position(){

    // this speed is ~ in cm because we are using 10^7 numbers from GPS
    //float tmp = 1.0/dTnav;

    // calculate position from gps + expected travel during gps_lag
    //current_loc.lng = xLeadFilter.get_position(g_gps->longitude, lon_speed, g_gps->get_lag());
    //current_loc.lat = yLeadFilter.get_position(g_gps->latitude,  lat_speed, g_gps->get_lag());

	// need to blend the GPS into the more accurate wheel encoders with a complementary filter
}

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void calc_distance_and_bearing()
{
    // waypoint distance from plane in cm
    // ---------------------------------------
    wp_distance     = get_distance_cm(&current_loc, &next_WP);
    home_distance   = get_distance_cm(&current_loc, &home);

	//cliSerial->printf_P(PSTR("c:%ld, %ld\tn:%ld, %ld\td:%ld\n"), current_loc.lat, current_loc.lng, next_WP.lat, next_WP.lng, wp_distance);

    // wp_bearing is bearing to next waypoint
    // --------------------------------------------
    wp_bearing      = get_bearing_cd(&current_loc, &next_WP);
    home_bearing    = get_bearing_cd(&current_loc, &home);

    // bearing to target (used when yaw_mode = YAW_LOOK_AT_LOCATION)
    //yaw_look_at_WP_bearing = get_bearing_cd(&current_loc, &yaw_look_at_WP);
}

static void calc_location_error(struct Location *next_loc)
{
    /*
     *  Becuase we are using lat and lon to do our distance errors here's a quick chart:
     *  100     = 1m
     *  1000    = 11m	 = 36 feet
     *  1800    = 19.80m = 60 feet
     *  3000    = 33m
     *  10000   = 111m
     */

    // X Error
    long_error      = (float)(next_loc->lng - current_loc.lng) * scaleLongDown;       // 500 - 0 = 500 Go East

    // Y Error
    lat_error       = next_loc->lat - current_loc.lat;                                                          // 500 - 0 = 500 Go North
}

// called after a GPS read
static void run_navigation_contollers()
{
    // wp_distance is in CM
    // --------------------
	switch(control_mode) {
		case AUTO:
			// note: wp_control is handled by commands_logic
			verify_commands();
			break;

		case RTL:
			// execute the RTL state machine
			verify_RTL();
			break;

		case GUIDED:
			wp_control = WP_MODE;
			// check if we are close to point > loiter
			wp_verify_byte = 0;
			verify_nav_wp();

			if (wp_control != WP_MODE) {
				set_mode(STABILIZE);
			}
			break;

		case STABILIZE:
			wp_control = NO_NAV_MODE;
			break;
	}
}

static bool check_missed_wp()
{
    int32_t temp;
    temp = wp_bearing - original_wp_bearing;
    temp = wrap_180(temp);
    return (labs(temp) > 9000);         // we passed the waypoint by 100 degrees
}

static int16_t
get_nav_pitch(int16_t minSpeed, int16_t dist_err)
{
	static int16_t desired_speed_old = 0;
	int16_t nav_out, ff_out, speed_error;

	//dist_err		= (float)long_error * cos_yaw_x + (float)lat_error * sin_yaw_y;
    //dist_err 		= constrain(dist_err, 0, 45);

    // this is the speed of the wheels: 1000 = 1 rotation of the wheels.
    // We convert cm to rpm based on wheel diamter and encoder ticks per revolution
    desired_speed 	= convert_distance_to_encoder_speed(dist_err);

	// accleration is limited to prevent wobbly starts towards waypoints
	desired_speed 	= min(desired_speed, desired_speed_old + 10);// limit going faster
	desired_speed 	= max(desired_speed, minSpeed);
	desired_speed_old = desired_speed;

	// We simply rotated the wheels at the desired speed times a proportional value
	// stabilizer manages uprightness
    ff_out          = (float)desired_speed * g.throttle;

	// grab the wheel speed error
	speed_error 	= wheel.speed - desired_speed;
	nav_out      	= g.pid_nav.get_pid(speed_error, G_Dt);

    return constrain((nav_out - ff_out), -2000, 2000);
}

static int16_t get_dist_err()
{
	int16_t dist_err;
	dist_err		= (float)long_error * cos_yaw_x + (float)lat_error * sin_yaw_y;
    return constrain(dist_err, 0, 45);
}

static int16_t get_desired_speed(int16_t max_speed)
{
    /*
    Based on Equation by Bill Premerlani & Robert Lefebvre
    	(sq(V2)-sq(V1))/2 = A(X2-X1)
        derives to:
        V1 = sqrt(sq(V2) - 2*A*(X2-X1))
     */

    if(ap.fast_corner) {
        // don't slow down
    }else{
        if(wp_distance < 20000){ // limit the size of numbers we're dealing with to avoid overflow
            // go slower
    	 	int32_t temp 	= 2 * 100 * (int32_t)(wp_distance - g.waypoint_radius * 100);
    	 	int32_t s_min 	= WAYPOINT_SPEED_MIN;
    	 	temp 			+= s_min * s_min;
    		max_speed 		= sqrt((float)temp);
            max_speed 		= min(max_speed, g.waypoint_speed_max);
        }
    }

    max_speed 		= min(max_speed, max_speed_old + (100 * dTnav));// limit going faster
    max_speed 		= max(max_speed, WAYPOINT_SPEED_MIN); 	// don't go too slow
    max_speed_old 	= max_speed;
    return max_speed;
}

static void reset_desired_speed()
{
    max_speed_old = 0;
}

static void update_crosstrack(void)
{
    // Crosstrack Error
    // ----------------
    if (abs(wrap_180(wp_bearing - original_wp_bearing)) < 4500) {    // If we are too far off or too close we don't do track following
        float temp = (wp_bearing - original_wp_bearing) * RADX100;
        crosstrack_error = sin(temp) * (wp_distance * g.crosstrack_gain);    // Meters we are off track line
        nav_bearing = wp_bearing + constrain(crosstrack_error, -3000, 3000);
        nav_bearing = wrap_360(nav_bearing);
    }else{
        nav_bearing = wp_bearing;
    }
}

// Keeps old data out of our calculation / logs
static void reset_nav_params(void)
{
    // always start Circle mode at same angle
    circle_angle                    = 0;

    // We must be heading to a new WP, so XTrack must be 0
    crosstrack_error                = 0;

    // Will be set by new command
    wp_bearing                      = 0;

    // Will be set by new command
    wp_distance                     = 0;

    // Will be set by new command, used by loiter
    long_error                      = 0;
    lat_error                       = 0;
    nav_lon 						= 0;
    nav_lat 						= 0;
    //nav_roll 						= 0;
    nav_pitch 						= 0;
    //auto_roll 						= 0;
    auto_pitch 						= 0;
}

static int32_t wrap_360(int32_t error)
{
    if (error > 36000) error -= 36000;
    if (error < 0) error += 36000;
    return error;
}

static int32_t wrap_180(int32_t error)
{
    if (error > 18000) error -= 36000;
    if (error < -18000) error += 36000;
    return error;
}
