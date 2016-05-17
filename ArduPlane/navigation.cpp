// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

// set the nav_controller pointer to the right controller
void Plane::set_nav_controller(void)
{
    switch ((AP_Navigation::ControllerType)g.nav_controller.get()) {

    default:
    case AP_Navigation::CONTROLLER_DEFAULT:
        // fall through to L1 as default controller

    case AP_Navigation::CONTROLLER_L1:
        nav_controller = &L1_controller;
        break;
    }
}

/*
  reset the total loiter angle
 */
void Plane::loiter_angle_reset(void)
{
    loiter.sum_cd = 0;
    loiter.total_cd = 0;
}

/*
  update the total angle we have covered in a loiter. Used to support
  commands to do N circles of loiter
 */
void Plane::loiter_angle_update(void)
{
    int32_t target_bearing_cd = nav_controller->target_bearing_cd();
    int32_t loiter_delta_cd;
    if (loiter.sum_cd == 0 && !reached_loiter_target()) {
        // we don't start summing until we are doing the real loiter
        loiter_delta_cd = 0;
    } else if (loiter.sum_cd == 0) {
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
void Plane::navigate()
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
    SpdHgt_Controller->set_path_proportion(auto_state.wp_proportion);

    // update total loiter angle
    loiter_angle_update();

    // control mode specific updates to navigation demands
    // ---------------------------------------------------
    update_navigation();
}

void Plane::calc_airspeed_errors()
{
    float airspeed_measured_cm = airspeed.get_airspeed_cm();

    // Normal airspeed target
    target_airspeed_cm = g.airspeed_cruise_cm;

    // FBW_B airspeed target
    if (control_mode == FLY_BY_WIRE_B || 
        control_mode == CRUISE) {
        target_airspeed_cm = ((int32_t)(aparm.airspeed_max -
                                        aparm.airspeed_min) *
                              channel_throttle->get_control_in()) +
                             ((int32_t)aparm.airspeed_min * 100);
    }

    // Landing airspeed target
    if (control_mode == AUTO) {
        float land_airspeed = SpdHgt_Controller->get_land_airspeed();
        switch (flight_stage) {
        case AP_SpdHgtControl::FLIGHT_LAND_APPROACH:
            if (land_airspeed >= 0) {
                target_airspeed_cm = land_airspeed * 100;
            }
            break;

        case AP_SpdHgtControl::FLIGHT_LAND_PREFLARE:
        case AP_SpdHgtControl::FLIGHT_LAND_FINAL:
            if (auto_state.land_pre_flare && aparm.land_pre_flare_airspeed > 0) {
                // if we just preflared then continue using the pre-flare airspeed during final flare
                target_airspeed_cm = aparm.land_pre_flare_airspeed * 100;
            } else if (land_airspeed >= 0) {
                target_airspeed_cm = land_airspeed * 100;
            }
            break;

        default:
            break;
        }
    }

    // Set target to current airspeed + ground speed undershoot,
    // but only when this is faster than the target airspeed commanded
    // above.
    if (control_mode >= FLY_BY_WIRE_B && (g.min_gndspeed_cm > 0)) {
        int32_t min_gnd_target_airspeed = airspeed_measured_cm + groundspeed_undershoot;
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
    airspeed_error = SpdHgt_Controller->get_target_airspeed() - airspeed_measured_cm * 0.01f;
}

void Plane::calc_gndspeed_undershoot()
{
 	// Use the component of ground speed in the forward direction
	// This prevents flyaway if wind takes plane backwards
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
	    Vector2f gndVel = ahrs.groundspeed_vector();
		const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
		Vector2f yawVect = Vector2f(rotMat.a.x,rotMat.b.x);
		yawVect.normalize();
		float gndSpdFwd = yawVect * gndVel;
        groundspeed_undershoot = (g.min_gndspeed_cm > 0) ? (g.min_gndspeed_cm - gndSpdFwd*100) : 0;
    }
}

void Plane::update_loiter(uint16_t radius)
{
    if (radius <= 1) {
        // if radius is <=1 then use the general loiter radius. if it's small, use default
        radius = (abs(g.loiter_radius) <= 1) ? LOITER_RADIUS_DEFAULT : abs(g.loiter_radius);
        if (next_WP_loc.flags.loiter_ccw == 1) {
            loiter.direction = -1;
        } else {
            loiter.direction = (g.loiter_radius < 0) ? -1 : 1;
        }
    }

    if (loiter.start_time_ms != 0 &&
        quadplane.available() &&
        quadplane.guided_mode != 0) {
        if (!auto_state.vtol_loiter) {
            auto_state.vtol_loiter = true;
            // reset loiter start time, so we don't consider the point
            // reached till we get much closer
            loiter.start_time_ms = 0;
            quadplane.guided_start();
        }
    } else if (loiter.start_time_ms == 0 &&
        control_mode == AUTO &&
        !auto_state.no_crosstrack &&
        get_distance(current_loc, next_WP_loc) > radius*2) {
        // if never reached loiter point and using crosstrack and somewhat far away from loiter point
        // navigate to it like in auto-mode for normal crosstrack behavior
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
    } else {
        nav_controller->update_loiter(next_WP_loc, radius, loiter.direction);
    }

    if (loiter.start_time_ms == 0) {
        if (reached_loiter_target() ||
            auto_state.wp_proportion > 1) {
            // we've reached the target, start the timer
            loiter.start_time_ms = millis();
            if (control_mode == GUIDED) {
                // starting a loiter in GUIDED means we just reached the target point
                gcs_send_mission_item_reached_message(0);
            }
            if (quadplane.available() &&
                quadplane.guided_mode != 0) {
                quadplane.guided_start();
            }
        }
    }
}

/*
  handle CRUISE mode, locking heading to GPS course when we have
  sufficient ground speed, and no aileron or rudder input
 */
void Plane::update_cruise()
{
    if (!cruise_state.locked_heading &&
        channel_roll->get_control_in() == 0 &&
        rudder_input == 0 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
        gps.ground_speed() >= 3 &&
        cruise_state.lock_timer_ms == 0) {
        // user wants to lock the heading - start the timer
        cruise_state.lock_timer_ms = millis();
    }
    if (cruise_state.lock_timer_ms != 0 &&
        (millis() - cruise_state.lock_timer_ms) > 500) {
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
void Plane::update_fbwb_speed_height(void)
{
    static float last_elevator_input;
    float elevator_input;
    elevator_input = channel_pitch->get_control_in() / 4500.0f;
    
    if (g.flybywire_elev_reverse) {
        elevator_input = -elevator_input;
    }
    
    change_target_altitude(g.flybywire_climb_rate * elevator_input * perf.delta_us_fast_loop * 0.0001f);
    
    if (is_zero(elevator_input) && !is_zero(last_elevator_input)) {
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
void Plane::setup_turn_angle(void)
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

/*
  see if we have reached our loiter target
 */
bool Plane::reached_loiter_target(void)
{
    if (quadplane.in_vtol_auto()) {
        return auto_state.wp_distance < 3;
    }
    return nav_controller->reached_loiter_target();
}
    
