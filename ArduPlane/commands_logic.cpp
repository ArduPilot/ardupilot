#include "Plane.h"

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
bool Plane::start_command(const AP_Mission::Mission_Command& cmd)
{
    // default to non-VTOL loiter
    auto_state.vtol_loiter = false;

        // log when new commands start
    if (should_log(MASK_LOG_CMD)) {
        logger.Write_Mission_Cmd(mission, cmd);
    }

    // special handling for nav vs non-nav commands
    if (AP_Mission::is_nav_cmd(cmd)) {
        // set land_complete to false to stop us zeroing the throttle
        auto_state.sink_rate = 0;

        // set takeoff_complete to true so we don't add extra elevator
        // except in a takeoff
        auto_state.takeoff_complete = true;

        // start non-idle
        auto_state.idle_mode = false;
        
        nav_controller->set_data_is_stale();

        // reset loiter start time. New command is a new loiter
        loiter.start_time_ms = 0;

        AP_Mission::Mission_Command next_nav_cmd;
        const uint16_t next_index = mission.get_current_nav_index() + 1;
        auto_state.wp_is_land_approach = mission.get_next_nav_cmd(next_index, next_nav_cmd) && (next_nav_cmd.id == MAV_CMD_NAV_LAND) &&
            !quadplane.is_vtol_land(next_nav_cmd.id);

        gcs().send_text(MAV_SEVERITY_INFO, "Executing nav command ID #%i",cmd.id);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Executing command ID #%i",cmd.id);
    }

    switch(cmd.id) {

    case MAV_CMD_NAV_TAKEOFF:
        crash_state.is_crashed = false;
        if (quadplane.is_vtol_takeoff(cmd.id)) {
            return quadplane.do_vtol_takeoff(cmd);
        }
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // LAND to Waypoint
        if (quadplane.is_vtol_land(cmd.id)) {
            crash_state.is_crashed = false;
            return quadplane.do_vtol_land(cmd);            
        }
        do_land(cmd);
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              // Loiter N Times
        do_loiter_turns(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        do_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        do_loiter_to_alt(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        set_mode(RTL, MODE_REASON_UNKNOWN);
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        do_continue_and_change_alt(cmd);
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:
        do_altitude_wait(cmd);
        break;

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        crash_state.is_crashed = false;
        return quadplane.do_vtol_takeoff(cmd);

    case MAV_CMD_NAV_VTOL_LAND:
        if (quadplane.options & QuadPlane::OPTION_MISSION_LAND_FW_APPROACH) {
            // the user wants to approach the landing in a fixed wing flight mode
            // the waypoint will be used as a loiter_to_alt
            // after which point the plane will compute the optimal into the wind direction
            // and fly in on that direction towards the landing waypoint
            // it will then transition to VTOL and do a normal quadplane landing
            do_landing_vtol_approach(cmd);
            break;
        } else {
            return quadplane.do_vtol_land(cmd);
        }
        
    // Conditional commands

    case MAV_CMD_CONDITION_DELAY:
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        do_within_distance(cmd);
        break;

    // Do commands

    case MAV_CMD_DO_CHANGE_SPEED:
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:
        if (cmd.p1 == 0 || cmd.p1 == 1) {
            auto_state.inverted_flight = (bool)cmd.p1;
            gcs().send_text(MAV_SEVERITY_INFO, "Set inverted %u", cmd.p1);
        }
        break;

    case MAV_CMD_DO_LAND_START:
        break;

    case MAV_CMD_DO_FENCE_ENABLE:
#if GEOFENCE_ENABLED == ENABLED
        if (cmd.p1 != 2) {
            if (!geofence_set_enabled((bool) cmd.p1, AUTO_TOGGLED)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Unable to set fence. Enabled state to %u", cmd.p1);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Set fence enabled state to %u", cmd.p1);
            }
        } else { //commanding to only disable floor
            if (! geofence_set_floor_enabled(false)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Unable to disable fence floor");
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "Fence floor disabled");
            }
        }    
#endif
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        autotune_enable(cmd.p1);
        break;

#if MOUNT == ENABLED
    // Sets the region of interest (ROI) for a sensor set or the
    // vehicle itself. This can then be used by the vehicles control
    // system to control the vehicle attitude and the attitude of various
    // devices such as cameras.
    //    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
    case MAV_CMD_DO_SET_ROI:
        if (cmd.content.location.alt == 0 && cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
            // switch off the camera tracking if enabled
            if (camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                camera_mount.set_mode_to_default();
            }
        } else {
            // set mount's target location
            camera_mount.set_roi_target(cmd.content.location);
        }
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:          // 205
        // point the camera to a specified angle
        camera_mount.set_angle_targets(cmd.content.mount_control.roll, 
                                       cmd.content.mount_control.pitch, 
                                       cmd.content.mount_control.yaw);
        break;
#endif

    case MAV_CMD_DO_VTOL_TRANSITION:
        plane.quadplane.handle_do_vtol_transition((enum MAV_VTOL_STATE)cmd.content.do_vtol_transition.target_state);
        break;

    case MAV_CMD_DO_ENGINE_CONTROL:
        plane.g2.ice_control.engine_control(cmd.content.do_engine_control.start_control,
                                            cmd.content.do_engine_control.cold_start,
                                            cmd.content.do_engine_control.height_delay_cm*0.01f);
        break;

    default:
        // unable to use the command, allow the vehicle to try the next command
        return false;
    }

    return true;
}

/*******************************************************************************
Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
Return true if we do not recognize the command so that we move on to the next command
*******************************************************************************/

bool Plane::verify_command(const AP_Mission::Mission_Command& cmd)        // Returns true if command complete
{
    switch(cmd.id) {

    case MAV_CMD_NAV_TAKEOFF:
        if (quadplane.is_vtol_takeoff(cmd.id)) {
            return quadplane.verify_vtol_takeoff(cmd);
        }
        return verify_takeoff();

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_LAND:
        if (quadplane.is_vtol_land(cmd.id)) {
            return quadplane.verify_vtol_land();            
        }
        if (flight_stage == AP_Vehicle::FixedWing::FlightStage::FLIGHT_ABORT_LAND) {
            return landing.verify_abort_landing(prev_WP_loc, next_WP_loc, current_loc, auto_state.takeoff_altitude_rel_cm, throttle_suppressed);

        } else {
            // use rangefinder to correct if possible
            const float height = height_above_target() - rangefinder_correction();
            return landing.verify_land(prev_WP_loc, next_WP_loc, current_loc,
                height, auto_state.sink_rate, auto_state.wp_proportion, auto_state.last_flying_ms, arming.is_armed(), is_flying(), rangefinder_state.in_range);
        }

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlim(cmd);

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_loiter_turns(cmd);

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();

    case MAV_CMD_NAV_LOITER_TO_ALT:
        return verify_loiter_to_alt(cmd);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        return verify_continue_and_change_alt();

    case MAV_CMD_NAV_ALTITUDE_WAIT:
        return verify_altitude_wait(cmd);

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return quadplane.verify_vtol_takeoff(cmd);

    case MAV_CMD_NAV_VTOL_LAND:
        if ((quadplane.options & QuadPlane::OPTION_MISSION_LAND_FW_APPROACH) && !verify_landing_vtol_approach(cmd)) {
            // verify_landing_vtol_approach will return true once we have completed the approach,
            // in which case we fall over to normal vtol landing code
            return false;
        } else {
            return quadplane.verify_vtol_land();
        }

    // Conditional commands

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();

    // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_INVERTED_FLIGHT:
    case MAV_CMD_DO_LAND_START:
    case MAV_CMD_DO_FENCE_ENABLE:
    case MAV_CMD_DO_AUTOTUNE_ENABLE:
    case MAV_CMD_DO_CONTROL_VIDEO:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_MOUNT_CONTROL:
    case MAV_CMD_DO_VTOL_TRANSITION:
    case MAV_CMD_DO_ENGINE_CONTROL:
        return true;

    default:
        // error message
        gcs().send_text(MAV_SEVERITY_WARNING,"Skipping invalid cmd #%i",cmd.id);
        // return true if we do not recognize the command so that we move on to the next command
        return true;
    }
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

void Plane::do_RTL(int32_t rtl_altitude)
{
    auto_state.next_wp_crosstrack = false;
    auto_state.crosstrack = false;
    prev_WP_loc = current_loc;
    next_WP_loc = rally.calc_best_rally_or_home_location(current_loc, rtl_altitude);
    setup_terrain_target_alt(next_WP_loc);
    set_target_altitude_location(next_WP_loc);

    if (aparm.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    setup_glide_slope();
    setup_turn_angle();

    logger.Write_Mode(control_mode, control_mode_reason);
}

/*
  start a NAV_TAKEOFF command
 */
void Plane::do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    prev_WP_loc = current_loc;
    set_next_WP(cmd.content.location);
    // pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
    auto_state.takeoff_pitch_cd        = (int16_t)cmd.p1 * 100;
    if (auto_state.takeoff_pitch_cd <= 0) {
        // if the mission doesn't specify a pitch use 4 degrees
        auto_state.takeoff_pitch_cd = 400;
    }
    auto_state.takeoff_altitude_rel_cm = next_WP_loc.alt - home.alt;
    next_WP_loc.lat = home.lat + 10;
    next_WP_loc.lng = home.lng + 10;
    auto_state.takeoff_speed_time_ms = 0;
    auto_state.takeoff_complete = false;                            // set flag to use gps ground course during TO.  IMU will be doing yaw drift correction
    auto_state.height_below_takeoff_to_level_off_cm = 0;
    // Flag also used to override "on the ground" throttle disable

    // zero locked course
    steer_state.locked_course_err = 0;
    steer_state.hold_course_cd = -1;
    auto_state.baro_takeoff_alt = barometer.get_altitude();
}

void Plane::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
}

void Plane::do_land(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);

    // configure abort altitude and pitch
    // if NAV_LAND has an abort altitude then use it, else use last takeoff, else use 50m
    if (cmd.p1 > 0) {
        auto_state.takeoff_altitude_rel_cm = (int16_t)cmd.p1 * 100;
    } else if (auto_state.takeoff_altitude_rel_cm <= 0) {
        auto_state.takeoff_altitude_rel_cm = 3000;
    }

    if (auto_state.takeoff_pitch_cd <= 0) {
        // If no takeoff command has ever been used, default to a conservative 10deg
        auto_state.takeoff_pitch_cd = 1000;
    }

    // zero rangefinder state, start to accumulate good samples now
    memset(&rangefinder_state, 0, sizeof(rangefinder_state));

    landing.do_land(cmd, relative_altitude);

    if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
        // if we were in an abort we need to explicitly move out of the abort state, as it's sticky
        set_flight_stage(AP_Vehicle::FixedWing::FLIGHT_LAND);
    }

#if GEOFENCE_ENABLED == ENABLED 
    if (g.fence_autoenable == 1) {
        if (! geofence_set_enabled(false, AUTO_TOGGLED)) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Disable fence failed (autodisable)");
        } else {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Fence disabled (autodisable)");
        }
    } else if (g.fence_autoenable == 2) {
        if (! geofence_set_floor_enabled(false)) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Disable fence floor failed (autodisable)");
        } else {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Fence floor disabled (auto disable)");
        }
    }
#endif
}

void Plane::do_landing_vtol_approach(const AP_Mission::Mission_Command& cmd)
{
    //set target alt
    Location loc = cmd.content.location;
    location_sanitize(current_loc, loc);
    set_next_WP(loc);

    // only set the direction if the quadplane landing radius override is not 0
    // if it's 0 update_loiter will manage the direction for us when we hand it
    // 0 later in the controller
    if (is_negative(quadplane.fw_land_approach_radius)) {
        loiter.direction = -1;
    } else if (is_positive(quadplane.fw_land_approach_radius)) {
        loiter.direction = 1;
    }

    vtol_approach_s.approach_stage = LOITER_TO_ALT;
}

void Plane::loiter_set_direction_wp(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.location.loiter_ccw) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
}

void Plane::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    location_sanitize(current_loc, cmdloc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);
}

void Plane::do_loiter_turns(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    location_sanitize(current_loc, cmdloc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);

    loiter.total_cd = (uint32_t)(LOWBYTE(cmd.p1)) * 36000UL;
    condition_value = 1; // used to signify primary turns goal not yet met
}

void Plane::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    location_sanitize(current_loc, cmdloc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);

    // we set start_time_ms when we reach the waypoint
    loiter.time_max_ms = cmd.p1 * (uint32_t)1000;     // convert sec to ms
    condition_value = 1; // used to signify primary time goal not yet met
}

void Plane::do_continue_and_change_alt(const AP_Mission::Mission_Command& cmd)
{
    // select heading method. Either mission, gps bearing projection or yaw based
    // If prev_WP_loc and next_WP_loc are different then an accurate wp based bearing can
    // be computed. However, if we had just changed modes before this, such as an aborted landing
    // via mode change, the prev and next wps are the same.
    float bearing;
    if (!locations_are_same(prev_WP_loc, next_WP_loc)) {
        // use waypoint based bearing, this is the usual case
        steer_state.hold_course_cd = -1;
    } else if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
        // use gps ground course based bearing hold
        steer_state.hold_course_cd = -1;
        bearing = AP::gps().ground_course_cd() * 0.01f;
        location_update(next_WP_loc, bearing, 1000); // push it out 1km
    } else {
        // use yaw based bearing hold
        steer_state.hold_course_cd = wrap_360_cd(ahrs.yaw_sensor);
        bearing = ahrs.yaw_sensor * 0.01f;
        location_update(next_WP_loc, bearing, 1000); // push it out 1km
    }

    next_WP_loc.alt = cmd.content.location.alt + home.alt;
    condition_value = cmd.p1;
    reset_offset_altitude();
}

void Plane::do_altitude_wait(const AP_Mission::Mission_Command& cmd)
{
    // set all servos to trim until we reach altitude or descent speed
    auto_state.idle_mode = true;
}

void Plane::do_loiter_to_alt(const AP_Mission::Mission_Command& cmd)
{
    //set target alt  
    Location loc = cmd.content.location;
    location_sanitize(current_loc, loc);
    set_next_WP(loc);
    loiter_set_direction_wp(cmd);

    // init to 0, set to 1 when altitude is reached
    condition_value = 0;
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
bool Plane::verify_takeoff()
{
    if (ahrs.yaw_initialised() && steer_state.hold_course_cd == -1) {
        const float min_gps_speed = 5;
        if (auto_state.takeoff_speed_time_ms == 0 && 
            gps.status() >= AP_GPS::GPS_OK_FIX_3D && 
            gps.ground_speed() > min_gps_speed &&
            hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
            auto_state.takeoff_speed_time_ms = millis();
        }
        if (auto_state.takeoff_speed_time_ms != 0 &&
            millis() - auto_state.takeoff_speed_time_ms >= 2000) {
            // once we reach sufficient speed for good GPS course
            // estimation we save our current GPS ground course
            // corrected for summed yaw to set the take off
            // course. This keeps wings level until we are ready to
            // rotate, and also allows us to cope with arbitrary
            // compass errors for auto takeoff
            float takeoff_course = wrap_PI(radians(gps.ground_course_cd()*0.01f)) - steer_state.locked_course_err;
            takeoff_course = wrap_PI(takeoff_course);
            steer_state.hold_course_cd = wrap_360_cd(degrees(takeoff_course)*100);
            gcs().send_text(MAV_SEVERITY_INFO, "Holding course %ld at %.1fm/s (%.1f)",
                              steer_state.hold_course_cd,
                              (double)gps.ground_speed(),
                              (double)degrees(steer_state.locked_course_err));
        }
    }

    if (steer_state.hold_course_cd != -1) {
        // call navigation controller for heading hold
        nav_controller->update_heading_hold(steer_state.hold_course_cd);
    } else {
        nav_controller->update_level_flight();        
    }

    // see if we have reached takeoff altitude
    int32_t relative_alt_cm = adjusted_relative_altitude_cm();
    if (relative_alt_cm > auto_state.takeoff_altitude_rel_cm) {
        gcs().send_text(MAV_SEVERITY_INFO, "Takeoff complete at %.2fm",
                          (double)(relative_alt_cm*0.01f));
        steer_state.hold_course_cd = -1;
        auto_state.takeoff_complete = true;
        next_WP_loc = prev_WP_loc = current_loc;

        plane.complete_auto_takeoff();

        // don't cross-track on completion of takeoff, as otherwise we
        // can end up doing too sharp a turn
        auto_state.next_wp_crosstrack = false;
        return true;
    } else {
        return false;
    }
}

/*
  update navigation for normal mission waypoints. Return true when the
  waypoint is complete
 */
bool Plane::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    steer_state.hold_course_cd = -1;

    // depending on the pass by flag either go to waypoint in regular manner or
    // fly past it for set distance along the line of waypoints
    Location flex_next_WP_loc = next_WP_loc;

    uint8_t cmd_passby = HIGHBYTE(cmd.p1); // distance in meters to pass beyond the wp
    uint8_t cmd_acceptance_distance = LOWBYTE(cmd.p1); // radius in meters to accept reaching the wp

    if (cmd_passby > 0) {
        float dist = get_distance(prev_WP_loc, flex_next_WP_loc);

        if (!is_zero(dist)) {
            float factor = (dist + cmd_passby) / dist;

            flex_next_WP_loc.lat = flex_next_WP_loc.lat + (flex_next_WP_loc.lat - prev_WP_loc.lat) * (factor - 1.0f);
            flex_next_WP_loc.lng = flex_next_WP_loc.lng + (flex_next_WP_loc.lng - prev_WP_loc.lng) * (factor - 1.0f);
        }
    }

    if (auto_state.crosstrack) {
        nav_controller->update_waypoint(prev_WP_loc, flex_next_WP_loc);
    } else {
        nav_controller->update_waypoint(current_loc, flex_next_WP_loc);
    }

    // see if the user has specified a maximum distance to waypoint
    // If override with p3 - then this is not used as it will overfly badly
    if (g.waypoint_max_radius > 0 &&
        auto_state.wp_distance > (uint16_t)g.waypoint_max_radius) {
        if (location_passed_point(current_loc, prev_WP_loc, flex_next_WP_loc)) {
            // this is needed to ensure completion of the waypoint
            if (cmd_passby == 0) {
                prev_WP_loc = current_loc;
            }
        }
        return false;
    }

    float acceptance_distance_m = 0; // default to: if overflown - let it fly up to the point
    if (cmd_acceptance_distance > 0) {
        // allow user to override acceptance radius
        acceptance_distance_m = cmd_acceptance_distance;
    } else if (cmd_passby == 0) {
        acceptance_distance_m = nav_controller->turn_distance(g.waypoint_radius, auto_state.next_turn_angle);
    } else {

    }
    
    if (auto_state.wp_distance <= acceptance_distance_m) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached waypoint #%i dist %um",
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)get_distance(current_loc, flex_next_WP_loc));
        return true;
	}

    // have we flown past the waypoint?
    if (location_passed_point(current_loc, prev_WP_loc, flex_next_WP_loc)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Passed waypoint #%i dist %um",
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)get_distance(current_loc, flex_next_WP_loc));
        return true;
    }

    return false;
}

bool Plane::verify_loiter_unlim(const AP_Mission::Mission_Command &cmd)
{
    if (cmd.p1 <= 1 && abs(g.rtl_radius) > 1) {
        // if mission radius is 0,1, and rtl_radius is valid, use rtl_radius.
        loiter.direction = (g.rtl_radius < 0) ? -1 : 1;
        update_loiter(abs(g.rtl_radius));
    } else {
        // else use mission radius
        update_loiter(cmd.p1);
    }
    return false;
}

bool Plane::verify_loiter_time()
{
    bool result = false;
    // mission radius is always aparm.loiter_radius
    update_loiter(0);

    if (loiter.start_time_ms == 0) {
        if (reached_loiter_target() && loiter.sum_cd > 1) {
            // we've reached the target, start the timer
            loiter.start_time_ms = millis();
        }
    } else if (condition_value != 0) {
        // primary goal, loiter time
        if ((millis() - loiter.start_time_ms) > loiter.time_max_ms) {
            // primary goal completed, initialize secondary heading goal
            condition_value = 0;
            result = verify_loiter_heading(true);
        }
    } else {
        // secondary goal, loiter to heading
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs().send_text(MAV_SEVERITY_INFO,"Loiter time complete");
        auto_state.vtol_loiter = false;
    }
    return result;
}

bool Plane::verify_loiter_turns(const AP_Mission::Mission_Command &cmd)
{
    bool result = false;
    uint16_t radius = HIGHBYTE(cmd.p1);
    update_loiter(radius);

    // LOITER_TURNS makes no sense as VTOL
    auto_state.vtol_loiter = false;

    if (condition_value != 0) {
        // primary goal, loiter time
        if (loiter.sum_cd > loiter.total_cd && loiter.sum_cd > 1) {
            // primary goal completed, initialize secondary heading goal
            condition_value = 0;
            result = verify_loiter_heading(true);
        }
    } else {
        // secondary goal, loiter to heading
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs().send_text(MAV_SEVERITY_INFO,"Loiter orbits complete");
    }
    return result;
}

/*
  verify a LOITER_TO_ALT command. This involves checking we have
  reached both the desired altitude and desired heading. The desired
  altitude only needs to be reached once.
 */
bool Plane::verify_loiter_to_alt(const AP_Mission::Mission_Command &cmd)
{
    bool result = false;

    update_loiter(cmd.p1);

    // condition_value == 0 means alt has never been reached
    if (condition_value == 0) {
        // primary goal, loiter to alt
        if (labs(loiter.sum_cd) > 1 && (loiter.reached_target_alt || loiter.unable_to_acheive_target_alt)) {
            // primary goal completed, initialize secondary heading goal
            if (loiter.unable_to_acheive_target_alt) {
                gcs().send_text(MAV_SEVERITY_INFO,"Loiter to alt was stuck at %d", current_loc.alt/100);
            }

            condition_value = 1;
            result = verify_loiter_heading(true);
        }
    } else {
        // secondary goal, loiter to heading
        result = verify_loiter_heading(false);
    }

    if (result) {
        gcs().send_text(MAV_SEVERITY_INFO,"Loiter to alt complete");
    }
    return result;
}

bool Plane::verify_RTL()
{
    if (g.rtl_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
    update_loiter(abs(g.rtl_radius));
	if (auto_state.wp_distance <= (uint32_t)MAX(g.waypoint_radius,0) || 
        reached_loiter_target()) {
			gcs().send_text(MAV_SEVERITY_INFO,"Reached RTL location");
			return true;
    } else {
        return false;
	}
}

bool Plane::verify_continue_and_change_alt()
{
    // is waypoint info not available and heading hold is?
    if (locations_are_same(prev_WP_loc, next_WP_loc) &&
        steer_state.hold_course_cd != -1) {
        //keep flying the same course with fixed steering heading computed at start if cmd
        nav_controller->update_heading_hold(steer_state.hold_course_cd);
    }
    else {
        // Is the next_WP less than 200 m away?
        if (get_distance(current_loc, next_WP_loc) < 200.0f) {
            //push another 300 m down the line
            int32_t next_wp_bearing_cd = get_bearing_cd(prev_WP_loc, next_WP_loc);
            location_update(next_WP_loc, next_wp_bearing_cd * 0.01f, 300.0f);
        }

        //keep flying the same course
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
    }

    //climbing?
    if (condition_value == 1 && adjusted_altitude_cm() >= next_WP_loc.alt) {
        return true;
    }
    //descending?
    else if (condition_value == 2 &&
             adjusted_altitude_cm() <= next_WP_loc.alt) {
        return true;
    }    
    //don't care if we're climbing or descending
    else if (labs(adjusted_altitude_cm() - next_WP_loc.alt) <= 500) {
        return true;
    }

    return false;
}

/*
  see if we have reached altitude or descent speed
 */
bool Plane::verify_altitude_wait(const AP_Mission::Mission_Command &cmd)
{
    if (current_loc.alt > cmd.content.altitude_wait.altitude*100.0f) {
        gcs().send_text(MAV_SEVERITY_INFO,"Reached altitude");
        return true;
    }
    if (auto_state.sink_rate > cmd.content.altitude_wait.descent_rate) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached descent rate %.1f m/s", (double)auto_state.sink_rate);
        return true;        
    }

    // if requested, wiggle servos
    if (cmd.content.altitude_wait.wiggle_time != 0) {
        static uint32_t last_wiggle_ms;
        if (auto_state.idle_wiggle_stage == 0 &&
            AP_HAL::millis() - last_wiggle_ms > cmd.content.altitude_wait.wiggle_time*1000) {
            auto_state.idle_wiggle_stage = 1;
            last_wiggle_ms = AP_HAL::millis();
        }
        // idle_wiggle_stage is updated in set_servos_idle()
    }

    return false;
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

void Plane::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value  = cmd.content.delay.seconds * 1000;    // convert seconds to milliseconds
}

void Plane::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters;
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

bool Plane::verify_wait_delay()
{
    if ((unsigned)(millis() - condition_start) > (unsigned)condition_value) {
        condition_value         = 0;
        return true;
    }
    return false;
}

bool Plane::verify_within_distance()
{
    if (auto_state.wp_distance < MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

void Plane::do_loiter_at_location()
{
    if (aparm.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
    next_WP_loc = current_loc;
}

void Plane::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.content.speed.speed_type)
    {
    case 0:             // Airspeed
        if (cmd.content.speed.target_ms > 0) {
            aparm.airspeed_cruise_cm.set(cmd.content.speed.target_ms * 100);
            gcs().send_text(MAV_SEVERITY_INFO, "Set airspeed %u m/s", (unsigned)cmd.content.speed.target_ms);
        }
        break;
    case 1:             // Ground speed
        gcs().send_text(MAV_SEVERITY_INFO, "Set groundspeed %u", (unsigned)cmd.content.speed.target_ms);
        aparm.min_gndspeed_cm.set(cmd.content.speed.target_ms * 100);
        break;
    }

    if (cmd.content.speed.throttle_pct > 0 && cmd.content.speed.throttle_pct <= 100) {
        gcs().send_text(MAV_SEVERITY_INFO, "Set throttle %u", (unsigned)cmd.content.speed.throttle_pct);
        aparm.throttle_cruise.set(cmd.content.speed.throttle_pct);
    }
}

void Plane::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        if (!set_home_persistently(gps.location())) {
            // silently ignore error
        }
    } else {
        if (!AP::ahrs().set_home(cmd.content.location)) {
            // silently ignore failure
        }
    }
}

// start_command_callback - callback function called from ap-mission when it begins a new mission command
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool Plane::start_command_callback(const AP_Mission::Mission_Command &cmd)
{
    if (control_mode == AUTO) {
        return start_command(cmd);
    }
    return true;
}

// verify_command_callback - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool Plane::verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    if (control_mode == AUTO) {
        bool cmd_complete = verify_command(cmd);

        // send message to GCS
        if (cmd_complete) {
            gcs().send_mission_item_reached_message(cmd.index);
        }

        return cmd_complete;
    }
    return false;
}

// exit_mission_callback - callback function called from ap-mission when the mission has completed
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
void Plane::exit_mission_callback()
{
    if (control_mode == AUTO) {
        set_mode(RTL, MODE_REASON_MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Mission complete, changing mode to RTL");
    }
}

bool Plane::verify_landing_vtol_approach(const AP_Mission::Mission_Command &cmd)
{
    switch (vtol_approach_s.approach_stage) {
        case LOITER_TO_ALT:
            {
                update_loiter(fabsf(quadplane.fw_land_approach_radius));

                if (labs(loiter.sum_cd) > 1 && (loiter.reached_target_alt || loiter.unable_to_acheive_target_alt)) {
                    Vector3f wind = ahrs.wind_estimate();
                    vtol_approach_s.approach_direction_deg = degrees(atan2f(-wind.y, -wind.x));
                    gcs().send_text(MAV_SEVERITY_INFO, "Selected an approach path of %.1f", (double)vtol_approach_s.approach_direction_deg);
                    vtol_approach_s.approach_stage = ENSURE_RADIUS;
                }
                break;
            }
        case ENSURE_RADIUS:
            {
                float radius;
                if (is_zero(quadplane.fw_land_approach_radius)) {
                    radius = aparm.loiter_radius;
                } else {
                    radius = quadplane.fw_land_approach_radius;
                }
                const int8_t direction = is_negative(radius) ? -1 : 1;
                radius = fabsf(radius);

                // validate that the vehicle is at least the expected distance away from the loiter point
                // require an angle total of at least 2 centidegrees, due to special casing of 1 centidegree
                if (((fabsf(get_distance(cmd.content.location, current_loc) - radius) > 5.0f) &&
                      (get_distance(cmd.content.location, current_loc) < radius)) ||
                    (loiter.sum_cd < 2)) {
                    nav_controller->update_loiter(cmd.content.location, radius, direction);
                    break;
                }
                vtol_approach_s.approach_stage = WAIT_FOR_BREAKOUT;
                FALLTHROUGH;
            }
        case WAIT_FOR_BREAKOUT:
            {
                float radius = quadplane.fw_land_approach_radius;
                if (is_zero(radius)) {
                    radius = aparm.loiter_radius;
                }
                const int8_t direction = is_negative(radius) ? -1 : 1;

                nav_controller->update_loiter(cmd.content.location, radius, direction);

                const float breakout_direction_rad = radians(wrap_180(vtol_approach_s.approach_direction_deg + (direction > 0 ? 270 : 90)));

                // breakout when within 5 degrees of the opposite direction
                if (fabsf(ahrs.yaw - breakout_direction_rad) < radians(5.0f)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Starting VTOL land approach path");
                    vtol_approach_s.approach_stage = APPROACH_LINE;
                    set_next_WP(cmd.content.location);
                    // fallthrough
                } else {
                    break;
                }
                FALLTHROUGH;
            }
        case APPROACH_LINE:
            {
                // project an apporach path
                Location start = cmd.content.location;
                Location end = cmd.content.location;

                // project a 1km waypoint to either side of the landing location
                location_update(start, vtol_approach_s.approach_direction_deg + 180, 1000);
                location_update(end, vtol_approach_s.approach_direction_deg, 1000);

                nav_controller->update_waypoint(start, end);

                // check if we should move on to the next waypoint
                Location breakout_loc = cmd.content.location;
                location_update(breakout_loc, vtol_approach_s.approach_direction_deg + 180, quadplane.stopping_distance());
                if(location_passed_point(current_loc, start, breakout_loc)) {
                    vtol_approach_s.approach_stage = VTOL_LANDING;
                    quadplane.do_vtol_land(cmd);
                    // fallthrough
                } else {
                    break;
                }
                FALLTHROUGH;
            }
        case VTOL_LANDING:
            // nothing to do here, we should be into the quadplane landing code
            return true;
    }

    return false;
}

bool Plane::verify_loiter_heading(bool init)
{
    if (quadplane.in_vtol_auto()) {
        // skip heading verify if in VTOL auto
        return true;
    }

    //Get the lat/lon of next Nav waypoint after this one:
    AP_Mission::Mission_Command next_nav_cmd;
    if (! mission.get_next_nav_cmd(mission.get_current_nav_index() + 1,
                                   next_nav_cmd)) {
        //no next waypoint to shoot for -- go ahead and break out of loiter
        return true;
    }

    if (get_distance(next_WP_loc, next_nav_cmd.content.location) < fabsf(aparm.loiter_radius)) {
        /* Whenever next waypoint is within the loiter radius,
           maintaining loiter would prevent us from ever pointing toward the next waypoint.
           Hence break out of loiter immediately
         */
        return true;
    }

    // Bearing in degrees
    int32_t bearing_cd = get_bearing_cd(current_loc,next_nav_cmd.content.location);

    // get current heading.
    int32_t heading_cd = gps.ground_course_cd();

    int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);

    if (init) {
        loiter.sum_cd = 0;
    }

    /*
      Check to see if the the plane is heading toward the land
      waypoint. We use 20 degrees (+/-10 deg) of margin so that
      we can handle 200 degrees/second of yaw.

      After every full circle, extend acceptance criteria to ensure
      aircraft will not loop forever in case high winds are forcing
      it beyond 200 deg/sec when passing the desired exit course
    */

    // Use integer division to get discrete steps
    int32_t expanded_acceptance = 1000 * (loiter.sum_cd / 36000);

    if (labs(heading_err_cd) <= 1000 + expanded_acceptance) {
        // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp

        // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        if (next_WP_loc.loiter_xtrack) {
            next_WP_loc = current_loc;
        }
        return true;
    }
    return false;
}
