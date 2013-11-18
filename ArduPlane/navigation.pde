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

    loiter.sum_cd += loiter_delta_cd;
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

    if (next_WP.lat == 0) {
        return;
    }

    // waypoint distance from plane
    // ----------------------------
    wp_distance = get_distance(current_loc, next_WP);

    if (wp_distance < 0) {
        gcs_send_text_P(SEVERITY_HIGH,PSTR("WP error - distance < 0"));
        return;
    }

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

    airspeed_error_cm = target_airspeed_cm - aspeed_cm;
}

static void calc_gndspeed_undershoot()
{
 	// Use the component of ground speed in the forward direction
	// This prevents flyaway if wind takes plane backwards
    if (g_gps && g_gps->status() >= GPS::GPS_OK_FIX_2D) {
	    Vector2f gndVel = ahrs.groundspeed_vector();
		const Matrix3f &rotMat = ahrs.get_dcm_matrix();
		Vector2f yawVect = Vector2f(rotMat.a.x,rotMat.b.x);
		yawVect.normalize();
		float gndSpdFwd = yawVect * gndVel;
        groundspeed_undershoot = (g.min_gndspeed_cm > 0) ? (g.min_gndspeed_cm - gndSpdFwd*100) : 0;
    }
}

static void calc_altitude_error()
{
    if (control_mode == FLY_BY_WIRE_B ||
        control_mode == CRUISE) {
        return;
    }
    if (nav_controller->reached_loiter_target()) {
        // once we reach a loiter target then lock to the final
        // altitude target
        target_altitude_cm = next_WP.alt;
    } else if (offset_altitude_cm != 0) {
        // control climb/descent rate
        target_altitude_cm = next_WP.alt - (offset_altitude_cm*((float)(wp_distance-30) / (float)(wp_totalDistance-30)));

        // stay within a certain range
        if (prev_WP.alt > next_WP.alt) {
            target_altitude_cm = constrain_int32(target_altitude_cm, next_WP.alt, prev_WP.alt);
        }else{
            target_altitude_cm = constrain_int32(target_altitude_cm, prev_WP.alt, next_WP.alt);
        }
    } else if (non_nav_command_ID != MAV_CMD_CONDITION_CHANGE_ALT) {
        target_altitude_cm = next_WP.alt;
    }

    altitude_error_cm       = target_altitude_cm - adjusted_altitude_cm();
}

static void update_loiter()
{
    nav_controller->update_loiter(next_WP, abs(g.loiter_radius), loiter.direction);
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
        g_gps && g_gps->status() >= GPS::GPS_OK_FIX_2D &&
        g_gps->ground_speed_cm >= 300 &&
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
        cruise_state.locked_heading_cd = g_gps->ground_course_cd;
        prev_WP = current_loc;
    }
    if (cruise_state.locked_heading) {
        next_WP = prev_WP;
        // always look 1km ahead
        location_update(next_WP, 
                        cruise_state.locked_heading_cd*0.01f, 
                        get_distance(prev_WP, current_loc) + 1000);
        nav_controller->update_waypoint(prev_WP, next_WP);
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
    
    target_altitude_cm += g.flybywire_climb_rate * elevator_input * delta_us_fast_loop * 0.0001f;
    
    if (elevator_input == 0.0f && last_elevator_input != 0.0f) {
        // the user has just released the elevator, lock in
        // the current altitude
        target_altitude_cm = current_loc.alt;
    }

    // check for FBWB altitude limit
    if (g.FBWB_min_altitude_cm != 0 && target_altitude_cm < home.alt + g.FBWB_min_altitude_cm) {
        target_altitude_cm = home.alt + g.FBWB_min_altitude_cm;
    }
    altitude_error_cm = target_altitude_cm - adjusted_altitude_cm();
    
    last_elevator_input = elevator_input;
    
    calc_throttle();
    calc_nav_pitch();
}

static void setup_glide_slope(void)
{
    // establish the distance we are travelling to the next waypoint,
    // for calculating out rate of change of altitude
    wp_totalDistance        = get_distance(current_loc, next_WP);
    wp_distance             = wp_totalDistance;

    /*
      work out if we will gradually change altitude, or try to get to
      the new altitude as quickly as possible.
     */
    switch (control_mode) {
    case RTL:
    case GUIDED:
        /* glide down slowly if above target altitude, but ascend more
           rapidly if below it. See
           https://github.com/diydrones/ardupilot/issues/39
        */
        if (current_loc.alt > next_WP.alt) {
            offset_altitude_cm = next_WP.alt - current_loc.alt;            
        } else {
            offset_altitude_cm = 0;
        }
        break;

    case AUTO:
        if (prev_WP.id != MAV_CMD_NAV_TAKEOFF && 
            prev_WP.alt != home.alt && 
            (next_WP.id == MAV_CMD_NAV_WAYPOINT || next_WP.id == MAV_CMD_NAV_LAND)) {
            offset_altitude_cm = next_WP.alt - prev_WP.alt;
        } else {
            offset_altitude_cm = 0;        
        }
        break;
    default:
        offset_altitude_cm = 0;        
        break;
    }
}

/*
  return relative altitude in meters (relative to home)
 */
static float relative_altitude(void)
{
    return (current_loc.alt - home.alt) * 0.01f;
}

/*
  return relative altitude in centimeters, absolute value
 */
static int32_t relative_altitude_abs_cm(void)
{
    return labs(current_loc.alt - home.alt);
}

