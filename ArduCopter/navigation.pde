// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// update_navigation - invokes navigation routines
// called at 10hz
static void update_navigation()
{
    static uint32_t nav_last_update = 0;        // the system time of the last time nav was run update

    // exit immediately if not auto_armed
    if (!ap.auto_armed) {
        return;
    }

    // check for inertial nav updates
    if( inertial_nav.position_ok() ) {

        // calculate time since nav controllers last ran
        dTnav = (float)(millis() - nav_last_update)/ 1000.0f;
        nav_last_update = millis();

        // prevent runnup in dTnav value
        dTnav = min(dTnav, 1.0f);

        // run the navigation controllers
        update_nav_mode();
    }
}

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
static void run_nav_updates(void)
{
    // fetch position from inertial navigation
    calc_position();

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
    Vector3f curr = inertial_nav.get_position();
    
    // get target from loiter or wpinav controller
    if( nav_mode == NAV_LOITER || nav_mode == NAV_CIRCLE ) {
        wp_distance = wp_nav.get_distance_to_target();
        wp_bearing = wp_nav.get_bearing_to_target();
    }else if( nav_mode == NAV_WP ) {
        wp_distance = wp_nav.get_distance_to_destination();
        wp_bearing = wp_nav.get_bearing_to_destination();
    }else{
        wp_distance = 0;
        wp_bearing = 0;
    }

    // calculate home distance and bearing
    if( ap.home_is_set && (g_gps->status() == GPS::GPS_OK_FIX_3D || g_gps->status() == GPS::GPS_OK_FIX_2D)) {
        home_distance = pythagorous2(curr.x, curr.y);
        home_bearing = pv_get_bearing_cd(curr,Vector3f(0,0,0));

        // update super simple bearing (if required) because it relies on home_bearing
        update_super_simple_bearing();
    }else{
        home_distance = 0;
        home_bearing = 0;
    }
}

// run_autopilot - highest level call to process mission commands
static void run_autopilot()
{
    switch( control_mode ) {
        case AUTO:
            // load the next command if the command queues are empty
            update_commands();

            // process the active navigation and conditional commands
            verify_commands();
            break;
        case GUIDED:
            // no need to do anything - wp_nav should take care of getting us to the desired location
            break;
        case RTL:
            verify_RTL();
            break;
    }
}

// set_nav_mode - update nav mode and initialise any variables as required
static bool set_nav_mode(uint8_t new_nav_mode)
{
    bool nav_initialised = false;       // boolean to ensure proper initialisation of nav modes
    Vector3f stopping_point;            // stopping point for circle mode

    // return immediately if no change
    if( new_nav_mode == nav_mode ) {
        return true;
    }

    switch( new_nav_mode ) {

        case NAV_NONE:
            nav_initialised = true;
            break;

        case NAV_CIRCLE:
            // set center of circle to current position
            wp_nav.get_stopping_point(inertial_nav.get_position(),inertial_nav.get_velocity(),stopping_point);
            circle_set_center(stopping_point,ahrs.yaw);
            nav_initialised = true;
            break;

        case NAV_LOITER:
            // set target to current position
            wp_nav.init_loiter_target(inertial_nav.get_position(), inertial_nav.get_velocity());
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
    switch( nav_mode ) {

        case NAV_NONE:
            // do nothing
            break;

        case NAV_CIRCLE:
            // call circle controller which in turn calls loiter controller
            update_circle(dTnav);
            break;

        case NAV_LOITER:
            // reset target if we are still on the ground
            if (ap.land_complete) {
                wp_nav.init_loiter_target(inertial_nav.get_position(),inertial_nav.get_velocity());
            }else{
                // call loiter controller
                wp_nav.update_loiter();
            }
            break;

        case NAV_WP:
            // call waypoint controller
            wp_nav.update_wpnav();
            break;
    }

    // log to dataflash
    if ((g.log_bitmask & MASK_LOG_NTUN) && nav_mode != NAV_NONE) {
        Log_Write_Nav_Tuning();
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

// Keeps old data out of our calculation / logs
static void reset_nav_params(void)
{
    // Will be set by new command
    wp_bearing                      = 0;

    // Will be set by new command
    wp_distance                     = 0;

    // Will be set by nav or loiter controllers
    lon_error                       = 0;
    lat_error                       = 0;
    nav_roll 						= 0;
    nav_pitch 						= 0;
}

// get_yaw_slew - reduces rate of change of yaw to a maximum
// assumes it is called at 100hz so centi-degrees and update rate cancel each other out
static int32_t get_yaw_slew(int32_t current_yaw, int32_t desired_yaw, int16_t deg_per_sec)
{
    return wrap_360_cd(current_yaw + constrain_int16(wrap_180_cd(desired_yaw - current_yaw), -deg_per_sec, deg_per_sec));
}


//////////////////////////////////////////////////////////
// circle navigation controller
//////////////////////////////////////////////////////////

// circle_set_center -- set circle controller's center position and starting angle
static void
circle_set_center(const Vector3f current_position, float heading_in_radians)
{
    float max_velocity;
    float cir_radius = g.circle_radius * 100;

    // set circle center to circle_radius ahead of current position
    circle_center.x = current_position.x + cir_radius * cos_yaw;
    circle_center.y = current_position.y + cir_radius * sin_yaw;

    // if we are doing a panorama set the circle_angle to the current heading
    if( g.circle_radius <= 0 ) {
        circle_angle = heading_in_radians;
        circle_angular_velocity_max = ToRad(g.circle_rate);
        circle_angular_acceleration = circle_angular_velocity_max;  // reach maximum yaw velocity in 1 second
    }else{
        // set starting angle to current heading - 180 degrees
        circle_angle = wrap_PI(heading_in_radians-PI);

        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        max_velocity = min(wp_nav.get_horizontal_velocity(), safe_sqrt(0.5f*wp_nav.get_waypoint_acceleration()*g.circle_radius*100.0f)); 

        // angular_velocity in radians per second
        circle_angular_velocity_max = max_velocity/((float)g.circle_radius * 100.0f);
        circle_angular_velocity_max = constrain_float(ToRad(g.circle_rate),-circle_angular_velocity_max,circle_angular_velocity_max);

        // angular_velocity in radians per second
        circle_angular_acceleration = wp_nav.get_waypoint_acceleration()/((float)g.circle_radius * 100);
        if (g.circle_rate < 0.0f) {
            circle_angular_acceleration = -circle_angular_acceleration;
        }
    }

    // initialise other variables
    circle_angle_total = 0;
    circle_angular_velocity = 0;

    // initialise loiter target.  Note: feed forward velocity set to zero
    wp_nav.init_loiter_target(current_position, Vector3f(0,0,0));
}

// update_circle - circle position controller's main call which in turn calls loiter controller with updated target position
static void
update_circle(float dt)
{
    float cir_radius = g.circle_radius * 100;
    Vector3f circle_target;

    // ramp up angular velocity to maximum
    if (g.circle_rate >= 0) {
        if (circle_angular_velocity < circle_angular_velocity_max) {
            circle_angular_velocity += circle_angular_acceleration * dt;
            circle_angular_velocity = constrain_float(circle_angular_velocity, 0, circle_angular_velocity_max);
        }
    }else{
        if (circle_angular_velocity > circle_angular_velocity_max) {
            circle_angular_velocity += circle_angular_acceleration * dt;
            circle_angular_velocity = constrain_float(circle_angular_velocity, circle_angular_velocity_max, 0);
        }
    }

    // update the target angle
    circle_angle += circle_angular_velocity * dt;
    circle_angle = wrap_PI(circle_angle);

    // update the total angle travelled
    circle_angle_total += circle_angular_velocity * dt;

    // if the circle_radius is zero we are doing panorama so no need to update loiter target
    if( g.circle_radius != 0.0 ) {
        // calculate target position
        circle_target.x = circle_center.x + cir_radius * cosf(-circle_angle);
        circle_target.y = circle_center.y - cir_radius * sinf(-circle_angle);
        circle_target.z = wp_nav.get_desired_alt();

        // re-use loiter position controller
        wp_nav.set_loiter_target(circle_target);
    }

    // call loiter controller
    wp_nav.update_loiter();
}
