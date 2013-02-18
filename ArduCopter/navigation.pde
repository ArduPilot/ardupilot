// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// update_navigation - invokes navigation routines
// called at 50hz
static void update_navigation()
{
    static uint32_t nav_last_update = 0;        // the system time of the last time nav was run update

    // check for inertial nav updates
    if( inertial_nav.position_ok() ) {

        // calculate time since nav controllers last ran
        dTnav = (float)(millis() - nav_last_update)/ 1000.0f;
        nav_last_update = millis();

        // prevent runnup in dTnav value
        dTnav = min(dTnav, 1.0f);

        // run the navigation controllers
        update_nav_mode();

        // update log
        if (g.log_bitmask & MASK_LOG_NTUN && motors.armed()) {
            Log_Write_Nav_Tuning();
        }
    }

    // To-Do: replace below with proper GPS failsafe
    // reduce nav outputs to zero if we have not seen a position update in 2 seconds
    if( millis() - nav_last_update > 2000 ) {
        // after 12 reads we guess we may have lost GPS signal, stop navigating
        // we have lost GPS signal for a moment. Reduce our error to avoid flyaways
        auto_roll  >>= 1;
        auto_pitch >>= 1;
    }
}

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
static void run_nav_updates(void)
{
    // fetch position from inertial navigation
    calc_position();

    // check altitude vs target
    verify_altitude();

    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

// calc_position - get lat and lon positions from inertial nav library
static void calc_position(){
    if( inertial_nav.position_ok() ) {
        // pull position from interial nav library
        current_loc.lng = inertial_nav.get_longitude();
        current_loc.lat = inertial_nav.get_latitude();
    }
}

// calc_distance_and_bearing - calculate distance and direction to waypoints for reporting and autopilot decisions
static void calc_distance_and_bearing()
{
    // get current position
    Vector2f curr_pos(inertial_nav.get_latitude_diff(), inertial_nav.get_longitude_diff());
    Vector2f dest;

    // get target from loiter or wpinav controller
    if( nav_mode == NAV_LOITER || nav_mode == NAV_CIRCLE ) {
        dest.x = loiter_lat_from_home_cm;
        dest.y = loiter_lon_from_home_cm;
    }else if( nav_mode == NAV_WP ) {
        dest.x = wpinav_destination.x;
        dest.y = wpinav_destination.y;
    }else{
        dest = curr_pos;
    }

    // calculate distance to target
    lat_error = dest.x - curr_pos.x;
    lon_error = dest.y - curr_pos.y;
    wp_distance = safe_sqrt(lat_error*lat_error+lon_error*lon_error);

    // calculate waypoint bearing
    // To-Do: change this to more efficient calculation
    if( waypoint_valid(next_WP) ) {
        wp_bearing  = get_bearing_cd(&current_loc, &next_WP);
    }else{
        wp_bearing = 0;
    }

    // calculate home distance and bearing
    if( ap.home_is_set ) {
        home_distance = safe_sqrt(curr_pos.x*curr_pos.x + curr_pos.y*curr_pos.y);
        // To-Do: change this to more efficient calculation
        home_bearing = get_bearing_cd(&current_loc, &home);

        // update super simple bearing (if required) because it relies on home_bearing
        update_super_simple_bearing();
    }else{
        home_distance = 0;
        home_bearing = 0;
    }

    // calculate bearing to target (used when yaw_mode = YAW_LOOK_AT_LOCATION)
    // To-Do: move this to the look-at-waypoint yaw controller
    if( waypoint_valid(yaw_look_at_WP) ) {
        yaw_look_at_WP_bearing = get_bearing_cd(&current_loc, &yaw_look_at_WP);
    }
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
            loiter_set_target(inertial_nav.get_latitude_diff(), inertial_nav.get_longitude_diff());
            nav_initialised = true;
            break;

        case NAV_WP:
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

            // wrap
            if (circle_angle > 6.28318531f)
                circle_angle -= 6.28318531f;

            // update target location
            // To-Do: ensure this target is updated for inertial navigation controller
            set_next_WP_latlon(
                circle_WP.lat + (g.circle_radius * 100 * sinf(1.57f - circle_angle)),
                circle_WP.lng + (g.circle_radius * 100 * cosf(1.57f - circle_angle) * scaleLongUp));
            break;

        case NAV_LOITER:
            get_loiter_pos_lat_lon(loiter_lat_from_home_cm, loiter_lon_from_home_cm, 0.1f);
            break;

        case NAV_WP:
            // move forward on the waypoint
            // To-Do: slew up the speed to the max waypoint speed instead of immediately jumping to max
            wpinav_advance_track_desired(g.waypoint_speed_max, 0.1f);
            // run the navigation controller
            get_wpinav_pos(0.1f);
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
    return (labs(temp) > 9000);         // we passed the waypoint by 90 degrees
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

// verify_altitude - check if we have reached the target altitude
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

    // Will be set by nav or loiter controllers
    lon_error                       = 0;
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

// valid_waypoint - checks if a waypoint has been initialised or not
static bool waypoint_valid(Location &wp)
{
     if( wp.lat != 0 || wp.lng != 0 ) {
         return true;
     }else{
         return false;
     }
}

////////////////////////////////////////////////////
// Loiter controller using inertial nav
////////////////////////////////////////////////////

// get_loiter_accel - loiter acceration controllers with desired accelerations provided in forward/right directions in cm/s/s
static void
get_loiter_accel(int16_t accel_req_forward, int16_t accel_req_right)
{
    float z_accel_meas = -AP_INTERTIALNAV_GRAVITY * 100;    // gravity in cm/s/s

    // update angle targets that will be passed to stabilize controller
    auto_roll = constrain((accel_req_right/(-z_accel_meas))*(18000/M_PI), -4500, 4500);
    auto_pitch = constrain((-accel_req_forward/(-z_accel_meas*cos_roll_x))*(18000/M_PI), -4500, 4500);
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
    float speed_error_lat = 0;     // The velocity in cm/s.
    float speed_error_lon = 0;     // The velocity in cm/s.

    float speed_lat = inertial_nav.get_latitude_velocity();
    float speed_lon = inertial_nav.get_longitude_velocity();

    int32_t accel_lat;
    int32_t accel_lon;
    int32_t accel_total;

    int16_t lat_p,lat_i,lat_d;
    int16_t lon_p,lon_i,lon_d;

    // calculate vel error
    speed_error_lat = vel_lat - speed_lat;
    speed_error_lon = vel_lon - speed_lon;

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
get_loiter_pos_lat_lon(int32_t target_lat_from_home, int32_t target_lon_from_home, float dt)
{
    float dist_error_lat;
    int32_t desired_vel_lat;

    float dist_error_lon;
    int32_t desired_vel_lon;

    int32_t dist_error_total;

    int16_t vel_sqrt;
    int32_t vel_total;

    int16_t linear_distance;      // the distace we swap between linear and sqrt.

    // calculate distance error
    dist_error_lat = target_lat_from_home - inertial_nav.get_latitude_diff();
    dist_error_lon = target_lon_from_home - inertial_nav.get_longitude_diff();

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
    set_next_WP_latlon(
        home.lat + loiter_lat_from_home_cm / LATLON_TO_CM,
        home.lng + loiter_lat_from_home_cm / LATLON_TO_CM * scaleLongUp);
}

// loiter_set_target - set loiter's target position from home in cm
// To-Do: change this function to accept a target in lat/lon format (and remove setting of next_WP?)
static void
loiter_set_target(float lat_from_home_cm, float lon_from_home_cm)
{
    loiter_lat_from_home_cm = lat_from_home_cm;
    loiter_lon_from_home_cm = lon_from_home_cm;

    // update next_WP location for reporting purposes
    set_next_WP_latlon(
        home.lat + loiter_lat_from_home_cm / LATLON_TO_CM,
        home.lng + loiter_lat_from_home_cm / LATLON_TO_CM * scaleLongUp);
}

//////////////////////////////////////////////////////////
// waypoint inertial navigation controller
//////////////////////////////////////////////////////////
// Waypoint navigation is accomplished by moving the target location up to a maximum of 10m from the current location

// get_wpinav_pos - wpinav position controller with desired position held in wpinav_destination
static void
get_wpinav_pos(float dt)
{
    // re-use loiter position controller
    get_loiter_pos_lat_lon(wpinav_target.x, wpinav_target.y, dt);
}

// wpinav_set_destination - set destination using lat/lon coordinates
void wpinav_set_destination(const Location& destination)
{
    wpinav_set_origin_and_destination(current_loc, destination);
}

// wpinav_set_origin_and_destination - set origin and destination using lat/lon coordinates
void wpinav_set_origin_and_destination(const Location& origin, const Location& destination)
{
    wpinav_origin.x = (origin.lat-home.lat) * LATLON_TO_CM;
    wpinav_origin.y = (origin.lng-home.lng) * LATLON_TO_CM * scaleLongDown;
    wpinav_destination.x = (destination.lat-home.lat) * LATLON_TO_CM;
    wpinav_destination.y = (destination.lng-home.lng) * LATLON_TO_CM * scaleLongDown;
    wpinav_pos_delta = wpinav_destination - wpinav_origin;
    wpinav_track_length = wpinav_pos_delta.length();
    wpinav_track_desired = 0;

    // set next_WP, prev_WP for reporting purposes
    // To-Do: move calcs below to a function
    set_next_WP_latlon(
        home.lat + wpinav_destination.x / LATLON_TO_CM,
        home.lng + wpinav_destination.y / LATLON_TO_CM * scaleLongUp);
}

#define WPINAV_MAX_POS_ERROR 2000.0f        // maximum distance (in cm) that the desired track can stray from our current location.
void
wpinav_advance_track_desired(float velocity_cms, float dt)
{
    float cross_track_dist;
    float track_covered;
    float track_desired_max;
    float line_a, line_b, line_c, line_m;

    // get current location
    Vector2f curr(inertial_nav.get_latitude_diff(), inertial_nav.get_longitude_diff());

    // check for zero length segment
    if( wpinav_pos_delta.x == 0 && wpinav_pos_delta.y == 0) {
        wpinav_target = wpinav_destination;
        return;
    }

    if( wpinav_pos_delta.x == 0 ) {
        // x is zero
        cross_track_dist = fabs(curr.x - wpinav_destination.x);
        track_covered = fabs(curr.y - wpinav_origin.y);
    }else if(wpinav_pos_delta.y == 0) {
        // y is zero
        cross_track_dist = fabs(curr.y - wpinav_destination.y);
        track_covered = fabs(curr.x - wpinav_origin.x);
    }else{
        // both x and y non zero
        line_a = wpinav_pos_delta.y;
        line_b = -wpinav_pos_delta.x;
        line_c = wpinav_pos_delta.x * wpinav_origin.y - wpinav_pos_delta.y * wpinav_origin.x;
        line_m = line_a / line_b;
        cross_track_dist = abs(line_a * curr.x + line_b * curr.y + line_c ) / wpinav_track_length;

        line_m = 1/line_m;
        line_a = line_m;
        line_b = -1;
        line_c = curr.y - line_m * curr.x;

        // calculate the distance to the closest point along the track and it's distance from the origin
        track_covered = abs(line_a*wpinav_origin.x + line_b*wpinav_origin.y + line_c) / safe_sqrt(line_a*line_a+line_b*line_b);
    }

    // maximum distance along the track that we will allow (stops target point from getting too far from the current position)
    track_desired_max = track_covered + safe_sqrt(WPINAV_MAX_POS_ERROR*WPINAV_MAX_POS_ERROR-cross_track_dist*cross_track_dist);

    // advance the current target
    wpinav_track_desired += velocity_cms * dt;

    // constrain the target from moving too far
    if( wpinav_track_desired > track_desired_max ) {
        wpinav_track_desired = track_desired_max;
    }
    if( wpinav_track_desired > wpinav_track_length ) {
        wpinav_track_desired = wpinav_track_length;
    }

    // recalculate the desired position
    float track_length_pct = wpinav_track_desired/wpinav_track_length;
    wpinav_target.x = wpinav_origin.x + wpinav_pos_delta.x * track_length_pct;
    wpinav_target.y = wpinav_origin.y + wpinav_pos_delta.y * track_length_pct;
}
