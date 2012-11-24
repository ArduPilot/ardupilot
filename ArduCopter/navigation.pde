// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// update_navigation - checks for new GPS updates and invokes navigation routines
static void update_navigation()
{
    static uint32_t nav_last_gps_update = 0;    // the system time of the last gps update
    static uint32_t nav_last_gps_time = 0;      // the time according to the gps
    bool pos_updated = false;
    bool log_output = false;

    // check for new gps data
    if( g_gps->fix && g_gps->time != nav_last_gps_time ) {

        // used to calculate speed in X and Y, iterms
        // ------------------------------------------
        dTnav               = (float)(millis() - nav_last_gps_update)/ 1000.0;
        nav_last_gps_update = millis();

        // prevent runup from bad GPS
        dTnav = min(dTnav, 1.0);

        // save GPS time
        nav_last_gps_time = g_gps->time;

        // signal to run nav controllers
        pos_updated = true;

        // signal to create log entry
        log_output = true;
    }

#if INERTIAL_NAV == ENABLED
    // TO-DO: clean this up because inertial nav is overwriting the dTnav and pos_updated from above
    // check for inertial nav updates
    if( inertial_nav.position_ok() ) {
        // 50hz
        dTnav = 0.02;   // To-Do: calculate the time from the mainloop or INS readings?

        // signal to run nav controllers
        pos_updated = true;
    }
#endif

    // calc various navigation values and run controllers if we've received a position update
    if( pos_updated ) {

        // calculate velocity
        calc_velocity_and_position();
		
		// calculate ground bearing
		calc_ground_bearing();

        // calculate distance, angles to target
        calc_distance_and_bearing();

        // run navigation controllers
        run_navigation_contollers();

        // Rotate the nav_lon and nav_lat vectors based on Yaw
        calc_nav_pitch_roll();

        // update log
        if (log_output && (g.log_bitmask & MASK_LOG_NTUN) && motors.armed()) {
            Log_Write_Nav_Tuning();
        }
    }

    // reduce nav outputs to zero if we have not received a gps update in 2 seconds
    if( millis() - nav_last_gps_update > 2000 ) {
        // after 12 reads we guess we may have lost GPS signal, stop navigating
        // we have lost GPS signal for a moment. Reduce our error to avoid flyaways
        auto_roll  >>= 1;
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
    static int32_t last_gps_longitude = 0;
    static int32_t last_gps_latitude  = 0;

    // initialise last_longitude and last_latitude
    if( last_gps_longitude == 0 && last_gps_latitude == 0 ) {
        last_gps_longitude = g_gps->longitude;
        last_gps_latitude = g_gps->latitude;
    }

    // this speed is ~ in cm because we are using 10^7 numbers from GPS
    float tmp = 1.0/dTnav;

#if INERTIAL_NAV == ENABLED
    if( inertial_nav.position_ok() ) {
        // pull velocity from interial nav library
        lon_speed = inertial_nav.get_longitude_velocity();
        lat_speed = inertial_nav.get_latitude_velocity();

        // pull position from interial nav library
        current_loc.lng = inertial_nav.get_longitude();
        current_loc.lat = inertial_nav.get_latitude();
    }else{
        // calculate velocity
        lon_speed  = (float)(g_gps->longitude - last_gps_longitude)  * scaleLongDown * tmp;
        lat_speed  = (float)(g_gps->latitude  - last_gps_latitude)  * tmp;

        // calculate position from gps + expected travel during gps_lag
        current_loc.lng = xLeadFilter.get_position(g_gps->longitude, lon_speed, g_gps->get_lag());
        current_loc.lat = yLeadFilter.get_position(g_gps->latitude,  lat_speed, g_gps->get_lag());
    }
#else
    // calculate velocity
    lon_speed  = (float)(g_gps->longitude - last_gps_longitude)  * scaleLongDown * tmp;
    lat_speed  = (float)(g_gps->latitude  - last_gps_latitude)  * tmp;

    // calculate position from gps + expected travel during gps_lag
    current_loc.lng = xLeadFilter.get_position(g_gps->longitude, lon_speed, g_gps->get_lag());
    current_loc.lat = yLeadFilter.get_position(g_gps->latitude,  lat_speed, g_gps->get_lag());
#endif

    // store gps lat and lon values for next iteration
    last_gps_longitude  = g_gps->longitude;
    last_gps_latitude   = g_gps->latitude;
}

static void calc_ground_bearing(){
	ground_bearing = atan2( lat_speed , lon_speed ) * DEGX100;
	ground_bearing = wrap_360(ground_bearing);						// atan2 returns a value of -pi to +pi, so we need to wrap this.
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

    // target_bearing is where we should be heading
    // --------------------------------------------
    target_bearing          = get_bearing_cd(&current_loc, &next_WP);
    home_to_copter_bearing  = get_bearing_cd(&home, &current_loc);
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

        // calculates desired Yaw
        update_auto_yaw();

        // calculates the desired Roll and Pitch
        update_nav_wp();
        break;

    case GUIDED:
        wp_control = WP_MODE;
        // check if we are close to point > loiter
        wp_verify_byte = 0;
        verify_nav_wp();

        if (wp_control == WP_MODE) {
            update_auto_yaw();
        } else {
            set_mode(LOITER);
        }
        update_nav_wp();
        break;

    case RTL:
        // have we reached the desired Altitude?
        if(alt_change_flag == REACHED_ALT || alt_change_flag == DESCENDING) {
            // we are at or above the target alt
            if(false == ap.rtl_reached_alt) {
                set_rtl_reached_alt(true);
                do_RTL();
            }
            wp_control = WP_MODE;
            // checks if we have made it to home
            update_nav_RTL();
        } else{
            // we need to loiter until we are ready to come home
            wp_control = LOITER_MODE;
        }

        // calculates desired Yaw
#if FRAME_CONFIG ==     HELI_FRAME
        update_auto_yaw();
#endif

        // calculates the desired Roll and Pitch
        update_nav_wp();
        break;

    // switch passthrough to LOITER
    case LOITER:
    case POSITION:
        // This feature allows us to reposition the quad when the user lets
        // go of the sticks

        if((abs(g.rc_2.control_in) + abs(g.rc_1.control_in)) > 500) {
            if(wp_distance > 500){
                ap.loiter_override = true;
            }
        }

        // Allow the user to take control temporarily,
        if(ap.loiter_override) {
            // this sets the copter to not try and nav while we control it
            wp_control      = NO_NAV_MODE;

            // reset LOITER to current position
            next_WP.lat = current_loc.lat;
            next_WP.lng = current_loc.lng;

            if(g.rc_2.control_in == 0 && g.rc_1.control_in == 0) {
                wp_control          = LOITER_MODE;
                ap.loiter_override  = false;
            }
        }else{
            wp_control = LOITER_MODE;
        }

        if(loiter_timer != 0) {
            // If we have a safe approach alt set and we have been loitering for 20 seconds(default), begin approach
            if((millis() - loiter_timer) > (uint32_t)g.auto_land_timeout.get()) {
                // just to make sure we clear the timer
                loiter_timer = 0;
                if(g.rtl_approach_alt == 0) {
                    set_mode(LAND);
                    if(home_distance < 300) {
                        next_WP.lat = home.lat;
                        next_WP.lng = home.lng;
                    }
                }else{
                    if(g.rtl_approach_alt < current_loc.alt) {
                        set_new_altitude(g.rtl_approach_alt);
                    }
                }
            }
        }

        // calculates the desired Roll and Pitch
        update_nav_wp();
        break;

    case LAND:
        verify_land();
        // calculates the desired Roll and Pitch
        update_nav_wp();
        break;

    case CIRCLE:
        wp_control              = CIRCLE_MODE;

        // calculates desired Yaw
        update_auto_yaw();
        update_nav_wp();
        break;

    case STABILIZE:
    case TOY_A:
    case TOY_M:
        wp_control = NO_NAV_MODE;
        update_nav_wp();
        break;
    }

    // are we in SIMPLE mode?
    if(ap.simple_mode && g.super_simple) {
        // get distance to home
        if(home_distance > SUPER_SIMPLE_RADIUS) {        // 10m from home
            // we reset the angular offset to be a vector from home to the quad
            initial_simple_bearing = home_to_copter_bearing;
            //cliSerial->printf("ISB: %d\n", initial_simple_bearing);
        }
    }

    if(yaw_mode == YAW_LOOK_AT_HOME) {
        if(ap.home_is_set) {
            nav_yaw = get_bearing_cd(&current_loc, &home);
        } else {
            nav_yaw = 0;
        }
    }
}

static void update_nav_RTL()
{
    // Have we have reached Home?
    if(wp_distance <= 200 || check_missed_wp()) {
        // if loiter_timer value > 0, we are set to trigger auto_land or approach
        set_mode(LOITER);

        // just in case we arrive and we aren't at the lower RTL alt yet.
        set_new_altitude(get_RTL_alt());

        // force loitering above home
        next_WP.lat = home.lat;
        next_WP.lng = home.lng;

        // If failsafe OR auto approach altitude is set
        // we will go into automatic land, (g.rtl_approach_alt) is the lowest point
        // -1 means disable feature
        if(ap.failsafe || g.rtl_approach_alt >= 0)
            loiter_timer = millis();
        else
            loiter_timer = 0;
    }
}

static bool check_missed_wp()
{
    int32_t temp;
    temp = target_bearing - original_target_bearing;
    temp = wrap_180(temp);
    return (labs(temp) > 9000);         // we passed the waypoint by 100 degrees
}

#define NAV_ERR_MAX 600
#define NAV_RATE_ERR_MAX 250
static void calc_loiter(int16_t x_error, int16_t y_error)
{
    int32_t p,i,d;                                              // used to capture pid values for logging
    int32_t output;
    int32_t x_target_speed, y_target_speed;

    // East / West
    x_target_speed  = g.pi_loiter_lon.get_p(x_error);                           // calculate desired speed from lon error

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_LOITER_KP || g.radio_tuning == CH6_LOITER_KI) ) {
        Log_Write_PID(CH6_LOITER_KP, x_error, x_target_speed, 0, 0, x_target_speed, tuning_value);
    }
#endif

    // calculate rate error
    x_rate_error    = x_target_speed - lon_speed;                           // calc the speed error

    p                               = g.pid_loiter_rate_lon.get_p(x_rate_error);
    i                               = g.pid_loiter_rate_lon.get_i(x_rate_error + x_error, dTnav);
    d                               = g.pid_loiter_rate_lon.get_d(x_error, dTnav);
    d                               = constrain(d, -2000, 2000);

    // get rid of noise
    if(abs(lon_speed) < 50) {
        d = 0;
    }

    output                  = p + i + d;
    nav_lon                 = constrain(output, -32000, 32000); // constraint to remove chance of overflow when adding int32_t to int16_t

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_LOITER_RATE_KP || g.radio_tuning == CH6_LOITER_RATE_KI || g.radio_tuning == CH6_LOITER_RATE_KD) ) {
        Log_Write_PID(CH6_LOITER_RATE_KP, x_rate_error, p, i, d, nav_lon, tuning_value);
    }
#endif

    // North / South
    y_target_speed  = g.pi_loiter_lat.get_p(y_error);                           // calculate desired speed from lat error

#if LOGGING_ENABLED == ENABLED
    // log output if PID logging is on and we are tuning the yaw
    if( g.log_bitmask & MASK_LOG_PID && (g.radio_tuning == CH6_LOITER_KP || g.radio_tuning == CH6_LOITER_KI) ) {
        Log_Write_PID(CH6_LOITER_KP+100, y_error, y_target_speed, 0, 0, y_target_speed, tuning_value);
    }
#endif

    // calculate rate error
    y_rate_error    = y_target_speed - lat_speed;                          // calc the speed error

    p                               = g.pid_loiter_rate_lat.get_p(y_rate_error);
    i                               = g.pid_loiter_rate_lat.get_i(y_rate_error + y_error, dTnav);
    d                               = g.pid_loiter_rate_lat.get_d(y_error, dTnav);
    d                               = constrain(d, -2000, 2000);

    // get rid of noise
    if(abs(lat_speed) < 50) {
        d = 0;
    }

    output                  = p + i + d;
    nav_lat                 = constrain(output, -32000, 32000); // constraint to remove chance of overflow when adding int32_t to int16_t

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

static void calc_nav_rate(int16_t max_speed)
{
    float temp, temp_x, temp_y;

    // push us towards the original track
    update_crosstrack();

    int16_t cross_speed = crosstrack_error * -g.crosstrack_gain;     // scale down crosstrack_error in cm
    cross_speed     = constrain(cross_speed, -150, 150);

    // rotate by 90 to deal with trig functions
    temp                    = (9000l - target_bearing) * RADX100;
    temp_x                  = cos(temp);
    temp_y                  = sin(temp);

    // rotate desired spped vector:
    int32_t x_target_speed = max_speed   * temp_x - cross_speed * temp_y;
    int32_t y_target_speed = cross_speed * temp_x + max_speed   * temp_y;

    // East / West
    // calculate rate error
    x_rate_error    = x_target_speed - lon_speed;

    x_rate_error    = constrain(x_rate_error, -500, 500);
    nav_lon         = g.pid_nav_lon.get_pid(x_rate_error, dTnav);
    int32_t tilt    = (x_target_speed * x_target_speed * (int32_t)g.tilt_comp) / 10000;

    if(x_target_speed < 0) tilt = -tilt;
    nav_lon                 += tilt;


    // North / South
    // calculate rate error
    y_rate_error    = y_target_speed - lat_speed;

    y_rate_error    = constrain(y_rate_error, -500, 500);       // added a rate error limit to keep pitching down to a minimum
    nav_lat         = g.pid_nav_lat.get_pid(y_rate_error, dTnav);
    tilt            = (y_target_speed * y_target_speed * (int32_t)g.tilt_comp) / 10000;

    if(y_target_speed < 0) tilt = -tilt;
    nav_lat                 += tilt;

    // copy over I term to Loiter_Rate
    g.pid_loiter_rate_lon.set_integrator(g.pid_nav_lon.get_integrator());
    g.pid_loiter_rate_lat.set_integrator(g.pid_nav_lat.get_integrator());
}


// this calculation rotates our World frame of reference to the copter's frame of reference
// We use the DCM's matrix to precalculate these trig values at 50hz
static void calc_nav_pitch_roll()
{
    //cliSerial->printf("ys %ld, cx %1.4f, _cx %1.4f | sy %1.4f, _sy %1.4f\n", dcm.yaw_sensor, cos_yaw_x, _cos_yaw_x, sin_yaw_y, _sin_yaw_y);
    // rotate the vector
    auto_roll       = (float)nav_lon * sin_yaw_y - (float)nav_lat * cos_yaw_x;
    auto_pitch      = (float)nav_lon * cos_yaw_x + (float)nav_lat * sin_yaw_y;

    // flip pitch because forward is negative
    auto_pitch = -auto_pitch;
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
    if (wp_distance >= (g.crosstrack_min_distance * 100) &&
        abs(wrap_180(target_bearing - original_target_bearing)) < 4500) {

	    float temp = (target_bearing - original_target_bearing) * RADX100;
    	crosstrack_error = sin(temp) * wp_distance;          // Meters we are off track line
    }else{
        // fade out crosstrack
        crosstrack_error >>= 1;
    }
}

static int32_t get_altitude_error()
{
    // Next_WP alt is our target alt
    // It changes based on climb rate
    // until it reaches the target_altitude

#if INERTIAL_NAV == ENABLED
    // use inertial nav for altitude error
    return next_WP.alt - inertial_nav._position.z;
#else
    return next_WP.alt - current_loc.alt;
#endif
}

static void clear_new_altitude()
{
    set_alt_change(REACHED_ALT);
}

static void force_new_altitude(int32_t new_alt)
{
    next_WP.alt     = new_alt;
    set_alt_change(REACHED_ALT);
}

static void set_new_altitude(int32_t new_alt)
{
    next_WP.alt     = new_alt;

    if(next_WP.alt > (current_loc.alt + 80)) {
        // we are below, going up
        set_alt_change(ASCENDING);

    }else if(next_WP.alt < (current_loc.alt - 80)) {
        // we are above, going down
        set_alt_change(DESCENDING);

    }else{
        // No Change
        set_alt_change(REACHED_ALT);
    }
}

static void verify_altitude()
{
    if(alt_change_flag == ASCENDING) {
        // we are below, going up
        if(current_loc.alt >  next_WP.alt - 50) {
        	set_alt_change(REACHED_ALT);
        }
    }else if (alt_change_flag == DESCENDING) {
        // we are above, going down
        if(current_loc.alt <=  next_WP.alt + 50){
        	set_alt_change(REACHED_ALT);
        }
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
    target_bearing                  = 0;

    // Will be set by new command
    wp_distance                     = 0;

    // Will be set by new command, used by loiter
    long_error                      = 0;
    lat_error                       = 0;
    nav_lon 						= 0;
    nav_lat 						= 0;
    nav_roll 						= 0;
    nav_pitch 						= 0;
    auto_roll 						= 0;
    auto_pitch 						= 0;

    // make sure we stick to Nav yaw on takeoff
    auto_yaw = nav_yaw;
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
