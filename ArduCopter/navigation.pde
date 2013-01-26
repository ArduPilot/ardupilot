// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// update_navigation - checks for new GPS updates and invokes navigation routines
// called at 50hz
static void update_navigation()
{
    static uint32_t nav_last_update = 0;        // the system time of the last time nav was run update
    bool pos_updated = false;
    bool log_output = false;

#if INERTIAL_NAV_XY == ENABLED
    static uint8_t nav_counter = 0;             // used to slow down the navigation to 10hz

    // check for inertial nav updates
    if( inertial_nav.position_ok() ) {
        nav_counter++;
        if( nav_counter >= 5) {
            nav_counter = 0;

            // calculate time since nav controllers last ran
            dTnav = (float)(millis() - nav_last_update)/ 1000.0f;
            nav_last_update = millis();

            // prevent runnup in dTnav value
            dTnav = min(dTnav, 1.0f);

            // signal to run nav controllers
            pos_updated = true;

            // signal to create log entry
            log_output = true;
        }
    }
#else

    static uint32_t nav_last_gps_time = 0;      // the time according to the gps

    // check for new gps data
    if( g_gps->fix && g_gps->time != nav_last_gps_time ) {

        // used to calculate speed in X and Y, iterms
        // ------------------------------------------
        dTnav = (float)(millis() - nav_last_update)/ 1000.0f;
        nav_last_update = millis();

        // prevent runup from bad GPS
        dTnav = min(dTnav, 1.0f);

        // save GPS time
        nav_last_gps_time = g_gps->time;

        // signal to run nav controllers
        pos_updated = true;

        // signal to create log entry
        log_output = true;
    }
#endif

    // setup to calculate new navigation values and run controllers if
    // we've received a position update
    if( pos_updated ) {

        nav_updates.need_velpos = 1;
        nav_updates.need_dist_bearing = 1;
        nav_updates.need_nav_controllers = 1;
        nav_updates.need_nav_pitch_roll = 1;

        // update log
        if (log_output && (g.log_bitmask & MASK_LOG_NTUN) && motors.armed()) {
            Log_Write_Nav_Tuning();
        }
    }

    // reduce nav outputs to zero if we have not seen a position update in 2 seconds
    if( millis() - nav_last_update > 2000 ) {
        // after 12 reads we guess we may have lost GPS signal, stop navigating
        // we have lost GPS signal for a moment. Reduce our error to avoid flyaways
        auto_roll  >>= 1;
        auto_pitch >>= 1;
    }
}

/*
  run navigation updates from nav_updates. Only run one at a time to
  prevent too much cpu usage hurting the main loop
 */
static void run_nav_updates(void)
{
    if (nav_updates.need_velpos) {
        calc_velocity_and_position();
        nav_updates.need_velpos = 0;
    } else if (nav_updates.need_dist_bearing) {
        calc_distance_and_bearing();
        nav_updates.need_dist_bearing = 0;
    } else if (nav_updates.need_nav_controllers) {
        run_autopilot();
        update_nav_mode();
        nav_updates.need_nav_controllers = 0;
    } else if (nav_updates.need_nav_pitch_roll) {
        calc_nav_pitch_roll();
        nav_updates.need_nav_pitch_roll = 0;
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
#if INERTIAL_NAV_XY == ENABLED
    if( inertial_nav.position_ok() ) {
        // pull velocity from interial nav library
        lon_speed = inertial_nav.get_longitude_velocity();
        lat_speed = inertial_nav.get_latitude_velocity();

        // pull position from interial nav library
        current_loc.lng = inertial_nav.get_longitude();
        current_loc.lat = inertial_nav.get_latitude();
    }
#else
    static int32_t last_gps_longitude = 0;
    static int32_t last_gps_latitude  = 0;

    // initialise last_longitude and last_latitude
    if( last_gps_longitude == 0 && last_gps_latitude == 0 ) {
        last_gps_longitude = g_gps->longitude;
        last_gps_latitude = g_gps->latitude;
    }

    // this speed is ~ in cm because we are using 10^7 numbers from GPS
    float tmp = 1.0f/dTnav;

    // calculate velocity
    lon_speed  = (float)(g_gps->longitude - last_gps_longitude)  * scaleLongDown * tmp;
    lat_speed  = (float)(g_gps->latitude  - last_gps_latitude)  * tmp;

    // calculate position from gps + expected travel during gps_lag
    current_loc.lng = xLeadFilter.get_position(g_gps->longitude, lon_speed, g_gps->get_lag());
    current_loc.lat = yLeadFilter.get_position(g_gps->latitude,  lat_speed, g_gps->get_lag());

    // store gps lat and lon values for next iteration
    last_gps_longitude  = g_gps->longitude;
    last_gps_latitude   = g_gps->latitude;
#endif
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

    // wp_bearing is bearing to next waypoint
    // --------------------------------------------
    wp_bearing          = get_bearing_cd(&current_loc, &next_WP);
    home_bearing        = get_bearing_cd(&current_loc, &home);

    // update super simple bearing (if required) because it relies on home_bearing
    update_super_simple_beading();

    // bearing to target (used when yaw_mode = YAW_LOOK_AT_LOCATION)
    yaw_look_at_WP_bearing = get_bearing_cd(&current_loc, &yaw_look_at_WP);
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

// run_autopilot - highest level call to process mission commands
static void run_autopilot()
{
    switch( control_mode ) {
        case AUTO:
            // majority of command logic is in commands_logic.pde
            verify_commands();
            break;
        case GUIDED:
            // switch to loiter once we've reached the target location and altitude
            if(verify_nav_wp()) {
                set_nav_mode(NAV_LOITER);
            }
        case RTL:
            verify_RTL();
            break;
    }
}

// set_nav_mode - update nav mode and initialise any variables as required
static bool set_nav_mode(uint8_t new_nav_mode)
{
    // boolean to ensure proper initialisation of nav modes
    bool nav_initialised = false;

    // return immediately if no change
    if( new_nav_mode == nav_mode ) {
        return true;
    }

    switch( new_nav_mode ) {

        case NAV_NONE:
            nav_initialised = true;
            break;

        case NAV_CIRCLE:
            // start circling around current location
            set_next_WP(&current_loc);
            circle_WP       = next_WP;
            circle_angle    = 0;
            nav_initialised = true;
            break;

        case NAV_LOITER:
            // set target to current position
            next_WP.lat = current_loc.lat;
            next_WP.lng = current_loc.lng;
            nav_initialised = true;
            break;

        case NAV_WP:
            nav_initialised = true;
            break;

        case NAV_LOITER_INAV:
            loiter_set_target(inertial_nav.get_latitude_diff(), inertial_nav.get_longitude_diff());
            nav_initialised = true;
            break;
    }

    // if initialisation has been successful update the yaw mode
    if( nav_initialised ) {
        nav_mode = new_nav_mode;
    }

    // return success or failure
    return nav_initialised;
}

// update_nav_mode - run navigation controller based on nav_mode
static void update_nav_mode()
{
    int16_t loiter_delta;
    int16_t speed;

    switch( nav_mode ) {

        case NAV_NONE:
            // do nothing
            break;

        case NAV_CIRCLE:
            // check if we have missed the WP
            loiter_delta = (wp_bearing - old_wp_bearing)/100;

            // reset the old value
            old_wp_bearing = wp_bearing;

            // wrap values
            if (loiter_delta > 180) loiter_delta -= 360;
            if (loiter_delta < -180) loiter_delta += 360;

            // sum the angle around the WP
            loiter_sum += loiter_delta;

            circle_angle += (circle_rate * dTnav);

            //1 degree = 0.0174532925 radians

            // wrap
            if (circle_angle > 6.28318531f)
                circle_angle -= 6.28318531f;

            next_WP.lng = circle_WP.lng + (g.circle_radius * 100 * cosf(1.57f - circle_angle) * scaleLongUp);
            next_WP.lat = circle_WP.lat + (g.circle_radius * 100 * sinf(1.57f - circle_angle));

            // use error as the desired rate towards the target
            // nav_lon, nav_lat is calculated

            // if the target location is >4m use waypoint controller
            if(wp_distance > 400) {
                calc_nav_rate(get_desired_speed(g.waypoint_speed_max));
            }else{
                // calc the lat and long error to the target
                calc_location_error(&next_WP);
                // call loiter controller
                calc_loiter(long_error, lat_error);
            }
            break;

        case NAV_LOITER:
            // check if user is overriding the loiter controller
            if((abs(g.rc_2.control_in) + abs(g.rc_1.control_in)) > 500) {
                if(wp_distance > 500){
                    ap.loiter_override = true;
                }
            }

            // check if user has release sticks
            if(ap.loiter_override) {
                if(g.rc_2.control_in == 0 && g.rc_1.control_in == 0) {
                    ap.loiter_override  = false;
                    // reset LOITER to current position
                    next_WP.lat = current_loc.lat;
                    next_WP.lng = current_loc.lng;
                }
                // We bring copy over our Iterms for wind control, but we don't navigate
                nav_lon = g.pid_loiter_rate_lon.get_integrator();
                nav_lat = g.pid_loiter_rate_lon.get_integrator();
                nav_lon = constrain(nav_lon, -2000, 2000);
                nav_lat = constrain(nav_lat, -2000, 2000);
            }else{
                // calc error to target
                calc_location_error(&next_WP);
                // use error as the desired rate towards the target
                calc_loiter(long_error, lat_error);
            }
            break;

        case NAV_WP:
            // calc position error to target
            calc_location_error(&next_WP);
            // calc speed to target
            speed = get_desired_speed(g.waypoint_speed_max);
            // use error as the desired rate towards the target
            calc_nav_rate(speed);
            break;

        case NAV_LOITER_INAV:
            get_loiter_pos_lat_lon(loiter_lat_from_home_cm, loiter_lon_from_home_cm, 0.1f);
            break;
    }

    /*
    // To-Do: check that we haven't broken toy mode
    case TOY_A:
    case TOY_M:
        set_nav_mode(NAV_NONE);
        update_nav_wp();
        break;
    }
    */
}

static bool check_missed_wp()
{
    int32_t temp;
    temp = wp_bearing - original_wp_bearing;
    temp = wrap_180(temp);
    return (labs(temp) > 9000);         // we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////
// Loiter controller (based on GPS position)
////////////////////////////////////////////////////////////////
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
    nav_lon                 = constrain(output, -4500, 4500); // constrain max angle to 45 degrees

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
    nav_lat                 = constrain(output, -4500, 4500); // constrain max angle to 45 degrees

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

///////////////////////////////////////////////////////////
// Waypoint controller (based on GPS position)
///////////////////////////////////////////////////////////
static void calc_nav_rate(int16_t max_speed)
{
    float temp, temp_x, temp_y;

    // push us towards the original track
    update_crosstrack();

    int16_t cross_speed = crosstrack_error * -g.crosstrack_gain;     // scale down crosstrack_error in cm
    cross_speed     = constrain(cross_speed, -150, 150);

    // rotate by 90 to deal with trig functions
    temp                    = (9000l - wp_bearing) * RADX100;
    temp_x                  = cosf(temp);
    temp_y                  = sinf(temp);

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
    // To-Do: remove this hack dependent upon nav_mode
    if( nav_mode != NAV_LOITER_INAV ) {
        // rotate the vector
        auto_roll       = (float)nav_lon * sin_yaw_y - (float)nav_lat * cos_yaw_x;
        auto_pitch      = (float)nav_lon * cos_yaw_x + (float)nav_lat * sin_yaw_y;

        // flip pitch because forward is negative
        auto_pitch = -auto_pitch;

        // constrain maximum roll and pitch angles to 45 degrees
        auto_roll = constrain(auto_roll, -4500, 4500);
        auto_pitch = constrain(auto_pitch, -4500, 4500);
    }
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
            if( temp < 0 ) temp = 0;                // check to ensure we don't try to take the sqrt of a negative number
    		max_speed 		= sqrtf((float)temp);
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
    if (wp_distance >= (unsigned long)max((g.crosstrack_min_distance * 100),0) &&
        abs(wrap_180(wp_bearing - original_wp_bearing)) < 4500) {

	    float temp = (wp_bearing - original_wp_bearing) * RADX100;
    	crosstrack_error = sinf(temp) * wp_distance;          // Meters we are off track line
    }else{
        // fade out crosstrack
        crosstrack_error >>= 1;
    }
}

static void force_new_altitude(int32_t new_alt)
{
    next_WP.alt     = new_alt;
    set_alt_change(REACHED_ALT);
}

static void set_new_altitude(int32_t new_alt)
{
    // if no change exit immediately
    if(new_alt == next_WP.alt) {
        return;
    }

    // update new target altitude
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
    wp_bearing                      = 0;

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

// get_yaw_slew - reduces rate of change of yaw to a maximum
// assumes it is called at 100hz so centi-degrees and update rate cancel each other out
static int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec)
{
    return wrap_360(current_yaw + constrain(wrap_180(desired_yaw - current_yaw), -deg_per_sec, deg_per_sec));
}

////////////////////////////////////////////////////
// Loiter controller using inertial nav
////////////////////////////////////////////////////

// get_loiter_accel - loiter acceration controllers with desired accelerations provided in forward/right directions in cm/s/s
static void
get_loiter_accel(int16_t accel_req_forward, int16_t accel_req_right)
{
    static float z_accel_meas = 0;      // The acceleration error in cm.
    static float accel_forward = 0;   // The acceleration error in cm.
    static float accel_right = 0;     // The acceleration error in cm.

    z_accel_meas = -AP_INTERTIALNAV_GRAVITY * 100;

    // calculate accel and filter with fc = 2 Hz
    // 100hz sample rate, 2hz filter, alpha = 0.11164f 
    // 20hz sample rate, 2hz filter, alpha = 0.38587f
    // 10hz sample rate, 2hz filter, alpha = 0.55686f
    accel_forward = accel_forward + 0.55686f * (accel_req_forward - accel_forward);
    accel_right = accel_right + 0.55686f * (accel_req_right - accel_right);

    // update angle targets that will be passed to stabilize controller
    auto_roll = constrain((accel_right/(-z_accel_meas))*(18000/M_PI), -4500, 4500);
    auto_pitch = constrain((-accel_forward/(-z_accel_meas*cos_roll_x))*(18000/M_PI), -4500, 4500);
}


// get_loiter_accel_lat_lon - loiter acceration controller with desired accelerations provided in lat/lon directions in cm/s/s
static void
get_loiter_accel_lat_lon(int16_t accel_lat, int16_t accel_lon)
{
    float accel_forward;
    float accel_right;

    accel_forward = accel_lat*cos_yaw + accel_lon*sin_yaw;
    accel_right = -accel_lat*sin_yaw + accel_lon*cos_yaw;

    get_loiter_accel(accel_forward, accel_right);
}


// get_loiter_vel_lat_lon - loiter velocity controller with desired velocity provided in lat/lon directions in cm/s
#define MAX_LOITER_VEL_ACCEL 400        // should be 1.5 times larger than MAX_LOITER_POS_ACCEL
static void
get_loiter_vel_lat_lon(int16_t vel_lat, int16_t vel_lon, float dt)
{
    static float speed_error_lat = 0;     // The velocity in cm/s.
    static float speed_error_lon = 0;     // The velocity in cm/s.

    float speed_lat = inertial_nav.get_latitude_velocity();
    float speed_lon = inertial_nav.get_longitude_velocity();

    int32_t accel_lat;
    int32_t accel_lon;
    int32_t accel_total;

    int16_t lat_p,lat_i,lat_d;
    int16_t lon_p,lon_i,lon_d;

    // calculate vel error and Filter with fc = 2 Hz
    // 100hz sample rate, 2hz filter, alpha = 0.11164f 
    // 20hz sample rate, 2hz filter, alpha = 0.38587f
    // 10hz sample rate, 2hz filter, alpha = 0.55686f
    speed_error_lat = speed_error_lat + 0.55686f * ((vel_lat - speed_lat) - speed_error_lat);
    speed_error_lon = speed_error_lon + 0.55686f * ((vel_lon - speed_lon) - speed_error_lon);

    lat_p   = g.pid_loiter_rate_lat.get_p(speed_error_lat);
    lat_i   = g.pid_loiter_rate_lat.get_i(speed_error_lat, dt);
    lat_d   = g.pid_loiter_rate_lat.get_d(speed_error_lat, dt);

    lon_p   = g.pid_loiter_rate_lon.get_p(speed_error_lon);
    lon_i   = g.pid_loiter_rate_lon.get_i(speed_error_lon, dt);
    lon_d   = g.pid_loiter_rate_lon.get_d(speed_error_lon, dt);

    accel_lat = (lat_p+lat_i+lat_d);
    accel_lon = (lon_p+lon_i+lon_d);

    accel_total = safe_sqrt(accel_lat*accel_lat + accel_lon*accel_lon);

    if( accel_total > MAX_LOITER_VEL_ACCEL ) {
        accel_lat = MAX_LOITER_VEL_ACCEL * accel_lat/accel_total;
        accel_lon = MAX_LOITER_VEL_ACCEL * accel_lon/accel_total;
    }

    get_loiter_accel_lat_lon(accel_lat, accel_lon);
}

// get_loiter_pos_lat_lon - loiter position controller with desired position provided as distance from home in lat/lon directions in cm
#define MAX_LOITER_POS_VELOCITY 750     // should be 1.5 ~ 2.0 times the pilot input's max velocity
#define MAX_LOITER_POS_ACCEL 250
static void
get_loiter_pos_lat_lon(int32_t target_lat, int32_t target_lon, float dt)
{
    static float dist_error_lat;
    int32_t desired_vel_lat;

    static float dist_error_lon;
    int32_t desired_vel_lon;

    int32_t dist_error_total;

    int16_t vel_sqrt;
    int32_t vel_total;

    int16_t linear_distance;      // the distace we swap between linear and sqrt.

    // calculate distance error and Filter with fc = 2 Hz
    // 100hz sample rate, 2hz filter, alpha = 0.11164f 
    // 20hz sample rate, 2hz filter, alpha = 0.38587f
    // 10hz sample rate, 2hz filter, alpha = 0.55686f
    dist_error_lat = dist_error_lat + 0.55686f * ((target_lat - inertial_nav.get_latitude_diff()) - dist_error_lat);
    dist_error_lon = dist_error_lon + 0.55686f * ((target_lon - inertial_nav.get_longitude_diff()) - dist_error_lon);

    linear_distance = MAX_LOITER_POS_ACCEL/(2*g.pi_loiter_lat.kP()*g.pi_loiter_lat.kP());

    dist_error_total = safe_sqrt(dist_error_lat*dist_error_lat + dist_error_lon*dist_error_lon);
    if( dist_error_total > 2*linear_distance ) {
        vel_sqrt = constrain(safe_sqrt(2*MAX_LOITER_POS_ACCEL*(dist_error_total-linear_distance)),0,1000);
        desired_vel_lat = vel_sqrt * dist_error_lat/dist_error_total;
        desired_vel_lon = vel_sqrt * dist_error_lon/dist_error_total;
    }else{
        desired_vel_lat = g.pi_loiter_lat.get_p(dist_error_lat);
        desired_vel_lon = g.pi_loiter_lon.get_p(dist_error_lon);
    }

    vel_total = safe_sqrt(desired_vel_lat*desired_vel_lat + desired_vel_lon*desired_vel_lon);
    if( vel_total > MAX_LOITER_POS_VELOCITY ) {
        desired_vel_lat = MAX_LOITER_POS_VELOCITY * desired_vel_lat/vel_total;
        desired_vel_lon = MAX_LOITER_POS_VELOCITY * desired_vel_lon/vel_total;
    }

    get_loiter_vel_lat_lon(desired_vel_lat, desired_vel_lon, dt);
}


#define MAX_LOITER_POS_VEL_VELOCITY 1000
// loiter_set_pos_from_velocity - loiter velocity controller with desired velocity provided in front/right directions in cm/s
static void
loiter_set_pos_from_velocity(int16_t vel_forward_cms, int16_t vel_right_cms, float dt)
{
    int32_t vel_lat;
    int32_t vel_lon;
    int32_t vel_total;

    vel_lat = vel_forward_cms*cos_yaw - vel_right_cms*sin_yaw;
    vel_lon = vel_forward_cms*sin_yaw + vel_right_cms*cos_yaw;

    // constrain the velocity vector and scale if necessary
    vel_total = safe_sqrt(vel_lat*vel_lat + vel_lon*vel_lon);
    if( vel_total > MAX_LOITER_POS_VEL_VELOCITY ) {
        vel_lat = MAX_LOITER_POS_VEL_VELOCITY * vel_lat/vel_total;
        vel_lon = MAX_LOITER_POS_VEL_VELOCITY * vel_lon/vel_total;
    }

    // update loiter target position
    loiter_lat_from_home_cm += vel_lat * dt;
    loiter_lon_from_home_cm += vel_lon * dt;

    // update next_WP location for reporting purposes
    next_WP.lat = home.lat + loiter_lat_from_home_cm;
    next_WP.lng = home.lng + loiter_lat_from_home_cm * scaleLongUp;
}

// loiter_set_target - set loiter's target position from home in cm
static void
loiter_set_target(float lat_from_home_cm, float lon_from_home_cm)
{
    loiter_lat_from_home_cm = lat_from_home_cm;
    loiter_lon_from_home_cm = lon_from_home_cm;

    // update next_WP location for reporting purposes
    next_WP.lat = home.lat + loiter_lat_from_home_cm;
    next_WP.lng = home.lng + loiter_lat_from_home_cm * scaleLongUp;
}
