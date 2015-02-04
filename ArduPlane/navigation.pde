// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


// set the nav_controller pointer to the right controller
static void set_nav_controller(void)
{
    switch ((AP_Navigation::ControllerType)g.nav_controller.get()) {
    case AP_Navigation::CONTROLLER_L1:
        nav_controller = &L1_controller;
        break;
    }
}

/*
  reset the total loiter angle
 */
static void loiter_angle_reset(void)
{
    loiter.sum_cd = 0;
    loiter.total_cd = 0;
}

/*
  update the total angle we have covered in a loiter. Used to support
  commands to do N circles of loiter
 */
static void loiter_angle_update(void)
{
    int32_t target_bearing_cd = nav_controller->target_bearing_cd();
    int32_t loiter_delta_cd;
    if (loiter.sum_cd == 0) {
        // use 1 cd for initial delta
        loiter_delta_cd = 1;
    } else {
        loiter_delta_cd = target_bearing_cd - loiter.old_target_bearing_cd;
    }
    loiter.old_target_bearing_cd = target_bearing_cd;
    loiter_delta_cd = wrap_180_cd(loiter_delta_cd);

    loiter.sum_cd += loiter_delta_cd * loiter.direction;
}

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
{
    // allow change of nav controller mid-flight
    set_nav_controller();

    // do not navigate with corrupt data
    // ---------------------------------
    if (!have_position) {
        return;
    }

    if (next_WP_loc.lat == 0) {
        return;
    }

    // waypoint distance from plane
    // ----------------------------
    auto_state.wp_distance = get_distance(current_loc, next_WP_loc);
    auto_state.wp_proportion = location_path_proportion(current_loc, 
                                                        prev_WP_loc, next_WP_loc);

    // update total loiter angle
    loiter_angle_update();

    // control mode specific updates to navigation demands
    // ---------------------------------------------------
    update_navigation();
}

static void calc_airspeed_errors()
{
    float aspeed_cm = airspeed.get_airspeed_cm();

    // Normal airspeed target
    target_airspeed_cm = g.airspeed_cruise_cm;

    // FBW_B airspeed target
    if (control_mode == FLY_BY_WIRE_B || 
        control_mode == CRUISE) {
        target_airspeed_cm = ((int32_t)(aparm.airspeed_max -
                                        aparm.airspeed_min) *
                              channel_throttle->control_in) +
                             ((int32_t)aparm.airspeed_min * 100);
    }

    // Set target to current airspeed + ground speed undershoot,
    // but only when this is faster than the target airspeed commanded
    // above.
    if (control_mode >= FLY_BY_WIRE_B && (g.min_gndspeed_cm > 0)) {
        int32_t min_gnd_target_airspeed = aspeed_cm + groundspeed_undershoot;
        if (min_gnd_target_airspeed > target_airspeed_cm)
            target_airspeed_cm = min_gnd_target_airspeed;
    }

    // Bump up the target airspeed based on throttle nudging
    if (control_mode >= AUTO && airspeed_nudge_cm > 0) {
        target_airspeed_cm += airspeed_nudge_cm;
    }

    // Apply airspeed limit
    if (target_airspeed_cm > (aparm.airspeed_max * 100))
        target_airspeed_cm = (aparm.airspeed_max * 100);

    // use the TECS view of the target airspeed for reporting, to take
    // account of the landing speed
    airspeed_error_cm = SpdHgt_Controller->get_target_airspeed()*100 - aspeed_cm;
}

static void calc_gndspeed_undershoot()
{
 	// Use the component of ground speed in the forward direction
	// This prevents flyaway if wind takes plane backwards
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
	    Vector2f gndVel = ahrs.groundspeed_vector();
		const Matrix3f &rotMat = ahrs.get_dcm_matrix();
		Vector2f yawVect = Vector2f(rotMat.a.x,rotMat.b.x);
		yawVect.normalize();
		float gndSpdFwd = yawVect * gndVel;
        groundspeed_undershoot = (g.min_gndspeed_cm > 0) ? (g.min_gndspeed_cm - gndSpdFwd*100) : 0;
    }
}

static void update_loiter()
{
    nav_controller->update_loiter(next_WP_loc, abs(g.loiter_radius), loiter.direction);
}

/*
  handle CRUISE mode, locking heading to GPS course when we have
  sufficient ground speed, and no aileron or rudder input
 */
static void update_cruise()
{
    if (!cruise_state.locked_heading &&
        channel_roll->control_in == 0 &&
        channel_rudder->control_in == 0 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
        gps.ground_speed() >= 3 &&
        cruise_state.lock_timer_ms == 0) {
        // user wants to lock the heading - start the timer
        cruise_state.lock_timer_ms = hal.scheduler->millis();
    }
    if (cruise_state.lock_timer_ms != 0 &&
        (hal.scheduler->millis() - cruise_state.lock_timer_ms) > 500) {
        // lock the heading after 0.5 seconds of zero heading input
        // from user
        cruise_state.locked_heading = true;
        cruise_state.lock_timer_ms = 0;
        cruise_state.locked_heading_cd = gps.ground_course_cd();
        prev_WP_loc = current_loc;
    }
    if (cruise_state.locked_heading) {
        next_WP_loc = prev_WP_loc;
        // always look 1km ahead
        location_update(next_WP_loc,
                        cruise_state.locked_heading_cd*0.01f, 
                        get_distance(prev_WP_loc, current_loc) + 1000);
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
    }
}


/*
  handle speed and height control in FBWB or CRUISE mode. 
  In this mode the elevator is used to change target altitude. The
  throttle is used to change target airspeed or throttle
 */
static void update_fbwb_speed_height(void)
{
    static float last_elevator_input;
    float elevator_input;
    elevator_input = channel_pitch->control_in / 4500.0f;
    
    if (g.flybywire_elev_reverse) {
        elevator_input = -elevator_input;
    }
    
    change_target_altitude(g.flybywire_climb_rate * elevator_input * delta_us_fast_loop * 0.0001f);
    
    if (elevator_input == 0.0f && last_elevator_input != 0.0f) {
        // the user has just released the elevator, lock in
        // the current altitude
        set_target_altitude_current();
    }

    // check for FBWB altitude limit
    check_minimum_altitude();

    altitude_error_cm = calc_altitude_error_cm();
    
    last_elevator_input = elevator_input;
    
    calc_throttle();
    calc_nav_pitch();
}

/*
  calculate the turn angle for the next leg of the mission
 */
static void setup_turn_angle(void)
{
    int32_t next_ground_course_cd = mission.get_next_ground_course_cd(-1);
    if (next_ground_course_cd == -1) {
        // the mission library can't determine a turn angle, assume 90 degrees
        auto_state.next_turn_angle = 90.0f;
    } else {
        // get the heading of the current leg
        int32_t ground_course_cd = get_bearing_cd(prev_WP_loc, next_WP_loc);

        // work out the angle we need to turn through
        auto_state.next_turn_angle = wrap_180_cd(next_ground_course_cd - ground_course_cd) * 0.01f;
    }
}    

