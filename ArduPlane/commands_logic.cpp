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
        const bool have_next_cmd = mission.get_next_nav_cmd(next_index, next_nav_cmd);
        auto_state.wp_is_land_approach = have_next_cmd && (next_nav_cmd.id == MAV_CMD_NAV_LAND);
#if HAL_QUADPLANE_ENABLED
        if (have_next_cmd && quadplane.is_vtol_land(next_nav_cmd.id)) {
            auto_state.wp_is_land_approach = false;
        }
#endif
    }

    switch(cmd.id) {

    case MAV_CMD_NAV_TAKEOFF:
        crash_state.is_crashed = false;
#if HAL_QUADPLANE_ENABLED
        if (quadplane.is_vtol_takeoff(cmd.id)) {
            return quadplane.do_vtol_takeoff(cmd);
        }
#endif
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // LAND to Waypoint
#if HAL_QUADPLANE_ENABLED
        if (quadplane.is_vtol_land(cmd.id)) {
            crash_state.is_crashed = false;
            return quadplane.do_vtol_land(cmd);            
        }
#endif
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
        set_mode(mode_rtl, ModeReason::MISSION_CMD);
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        do_continue_and_change_alt(cmd);
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:
        do_altitude_wait(cmd);
        break;

#if HAL_QUADPLANE_ENABLED
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        crash_state.is_crashed = false;
        return quadplane.do_vtol_takeoff(cmd);

    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        if (quadplane.landing_with_fixed_wing_spiral_approach()) {
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
#endif

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
#if AP_FENCE_ENABLED
        if (cmd.p1 == 0) { // disable fence
            plane.fence.enable(false);
            gcs().send_text(MAV_SEVERITY_INFO, "Fence disabled");
        } else if (cmd.p1 == 1) { // enable fence
            plane.fence.enable(true);
            gcs().send_text(MAV_SEVERITY_INFO, "Fence enabled");
        } else if (cmd.p1 == 2) { // disable fence floor only
            plane.fence.disable_floor();
            gcs().send_text(MAV_SEVERITY_INFO, "Fence floor disabled");
        }
#endif
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        autotune_enable(cmd.p1);
        break;

#if HAL_MOUNT_ENABLED
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
        camera_mount.set_angle_target(cmd.content.mount_control.roll, cmd.content.mount_control.pitch, cmd.content.mount_control.yaw, false);
        break;
#endif

#if HAL_QUADPLANE_ENABLED
    case MAV_CMD_DO_VTOL_TRANSITION:
        plane.quadplane.handle_do_vtol_transition((enum MAV_VTOL_STATE)cmd.content.do_vtol_transition.target_state);
        break;
#endif

#if AP_ICENGINE_ENABLED
    case MAV_CMD_DO_ENGINE_CONTROL:
        plane.g2.ice_control.engine_control(cmd.content.do_engine_control.start_control,
                                            cmd.content.do_engine_control.cold_start,
                                            cmd.content.do_engine_control.height_delay_cm*0.01f);
        break;
#endif

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        do_nav_script_time(cmd);
        break;
#endif

    case MAV_CMD_NAV_DELAY:
        do_nav_delay(cmd);
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
#if HAL_QUADPLANE_ENABLED
        if (quadplane.is_vtol_takeoff(cmd.id)) {
            return quadplane.verify_vtol_takeoff(cmd);
        }
#endif
        return verify_takeoff();

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_LAND:
#if HAL_QUADPLANE_ENABLED
        if (quadplane.is_vtol_land(cmd.id)) {
            return quadplane.verify_vtol_land();
        }
#endif
        if (flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
            return landing.verify_abort_landing(prev_WP_loc, next_WP_loc, current_loc, auto_state.takeoff_altitude_rel_cm, throttle_suppressed);

        } else {
            // use rangefinder to correct if possible
            float height = height_above_target() - rangefinder_correction();
            // for flare calculations we don't want to use the terrain
            // correction as otherwise we will flare early on rising
            // ground
            height -= auto_state.terrain_correction;
            return landing.verify_land(prev_WP_loc, next_WP_loc, current_loc,
                                       height, auto_state.sink_rate, auto_state.wp_proportion, auto_state.last_flying_ms, arming.is_armed(), is_flying(),
                                       g.rangefinder_landing && rangefinder_state.in_range);
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

#if HAL_QUADPLANE_ENABLED
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return quadplane.verify_vtol_takeoff(cmd);
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        if (quadplane.landing_with_fixed_wing_spiral_approach() && !verify_landing_vtol_approach(cmd)) {
            // verify_landing_vtol_approach will return true once we have completed the approach,
            // in which case we fall over to normal vtol landing code
            return false;
        } else {
            return quadplane.verify_vtol_land();
        }
#endif  // HAL_QUADPLANE_ENABLED

    // Conditional commands

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        return verify_nav_script_time(cmd);
#endif

     case MAV_CMD_NAV_DELAY:
         return verify_nav_delay(cmd);

    // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_INVERTED_FLIGHT:
    case MAV_CMD_DO_LAND_START:
    case MAV_CMD_DO_FENCE_ENABLE:
    case MAV_CMD_DO_AUTOTUNE_ENABLE:
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

void Plane::do_RTL(int32_t rtl_altitude_AMSL_cm)
{
    auto_state.next_wp_crosstrack = false;
    auto_state.crosstrack = false;
    prev_WP_loc = current_loc;
    next_WP_loc = rally.calc_best_rally_or_home_location(current_loc, rtl_altitude_AMSL_cm);
    setup_terrain_target_alt(next_WP_loc);
    set_target_altitude_location(next_WP_loc);

    if (aparm.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    setup_glide_slope();
    setup_turn_angle();

    logger.Write_Mode(control_mode->mode_number(), control_mode_reason);
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

    if (flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
        // if we were in an abort we need to explicitly move out of the abort state, as it's sticky
        set_flight_stage(AP_FixedWing::FlightStage::LAND);
    }

#if AP_FENCE_ENABLED
    plane.fence.auto_disable_fence_for_landing();
#endif
}

#if HAL_QUADPLANE_ENABLED
void Plane::do_landing_vtol_approach(const AP_Mission::Mission_Command& cmd)
{
    //set target alt
    Location loc = cmd.content.location;
    loc.sanitize(current_loc);
    set_next_WP(loc);

    vtol_approach_s.approach_stage = LOITER_TO_ALT;
}
#endif

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
    cmdloc.sanitize(current_loc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);
}

void Plane::do_loiter_turns(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    cmdloc.sanitize(current_loc);
    set_next_WP(cmdloc);
    loiter_set_direction_wp(cmd);

    loiter.total_cd = (uint32_t)(LOWBYTE(cmd.p1)) * 36000UL;
    condition_value = 1; // used to signify primary turns goal not yet met
}

void Plane::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    Location cmdloc = cmd.content.location;
    cmdloc.sanitize(current_loc);
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
    if (!prev_WP_loc.same_latlon_as(next_WP_loc)) {
        // use waypoint based bearing, this is the usual case
        steer_state.hold_course_cd = -1;
    } else if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
        // use gps ground course based bearing hold
        steer_state.hold_course_cd = -1;
        bearing = AP::gps().ground_course();
        next_WP_loc.offset_bearing(bearing, 1000); // push it out 1km
    } else {
        // use yaw based bearing hold
        steer_state.hold_course_cd = wrap_360_cd(ahrs.yaw_sensor);
        bearing = ahrs.yaw_sensor * 0.01f;
        next_WP_loc.offset_bearing(bearing, 1000); // push it out 1km
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
    loc.sanitize(current_loc);
    set_next_WP(loc);
    loiter_set_direction_wp(cmd);

    // init to 0, set to 1 when altitude is reached
    condition_value = 0;
}

// do_nav_delay - Delay the next navigation command
void Plane::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay.time_start_ms = millis();

    if (cmd.content.nav_delay.seconds > 0) {
        // relative delay
        nav_delay.time_max_ms = cmd.content.nav_delay.seconds * 1000; // convert seconds to milliseconds
    } else {
        // absolute delay to utc time
        nav_delay.time_max_ms = AP::rtc().get_time_utc(cmd.content.nav_delay.hour_utc, cmd.content.nav_delay.min_utc, cmd.content.nav_delay.sec_utc, 0);
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Delaying %u sec", (unsigned)(nav_delay.time_max_ms/1000));
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
bool Plane::verify_takeoff()
{
    if (ahrs.dcm_yaw_initialised() && steer_state.hold_course_cd == -1) {
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
            float takeoff_course = wrap_PI(radians(gps.ground_course())) - steer_state.locked_course_err;
            takeoff_course = wrap_PI(takeoff_course);
            steer_state.hold_course_cd = wrap_360_cd(degrees(takeoff_course)*100);
            gcs().send_text(MAV_SEVERITY_INFO, "Holding course %d at %.1fm/s (%.1f)",
                              (int)steer_state.hold_course_cd,
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

    // check for optional takeoff timeout
    if (takeoff_state.start_time_ms != 0 && g2.takeoff_timeout > 0) {
        const float ground_speed = gps.ground_speed();
        const float takeoff_min_ground_speed = 4;
        if (!arming.is_armed_and_safety_off()) {
            return false;
        }
        if (ground_speed >= takeoff_min_ground_speed) {
            takeoff_state.start_time_ms = 0;
        } else {
            uint32_t now = AP_HAL::millis();
            if (now - takeoff_state.start_time_ms > (uint32_t)(1000U * g2.takeoff_timeout)) {
                gcs().send_text(MAV_SEVERITY_INFO, "Takeoff timeout at %.1f m/s", ground_speed);
                plane.arming.disarm(AP_Arming::Method::TAKEOFFTIMEOUT);
                mission.reset();
            }
        }
    }

    // see if we have reached takeoff altitude
    int32_t relative_alt_cm = adjusted_relative_altitude_cm();
    if (relative_alt_cm > auto_state.takeoff_altitude_rel_cm) {
        gcs().send_text(MAV_SEVERITY_INFO, "Takeoff complete at %.2fm",
                          (double)(relative_alt_cm*0.01f));
        steer_state.hold_course_cd = -1;
        auto_state.takeoff_complete = true;
        next_WP_loc = prev_WP_loc = current_loc;

#if AP_FENCE_ENABLED
        plane.fence.auto_enable_fence_after_takeoff();
#endif

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
        const float dist = prev_WP_loc.get_distance(flex_next_WP_loc);
        const float bearing_deg = degrees(prev_WP_loc.get_bearing(flex_next_WP_loc));

        if (is_positive(dist)) {
            flex_next_WP_loc.offset_bearing(bearing_deg, cmd_passby);
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
        if (current_loc.past_interval_finish_line(prev_WP_loc, flex_next_WP_loc)) {
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
        acceptance_distance_m = nav_controller->turn_distance(get_wp_radius(), auto_state.next_turn_angle);
    }
    const float wp_dist = current_loc.get_distance(flex_next_WP_loc);
    if (wp_dist <= acceptance_distance_m) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached waypoint #%i dist %um",
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)current_loc.get_distance(flex_next_WP_loc));
        return true;
	}

    // have we flown past the waypoint?
    if (current_loc.past_interval_finish_line(prev_WP_loc, flex_next_WP_loc)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Passed waypoint #%i dist %um",
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)current_loc.get_distance(flex_next_WP_loc));
        return true;
    }

    return false;
}

bool Plane::verify_loiter_unlim(const AP_Mission::Mission_Command &cmd)
{
    // else use mission radius
    update_loiter(cmd.p1);
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
    if (cmd.type_specific_bits & (1U<<0)) {
        // special storage handling allows for larger radii
        radius *= 10;
    }
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
                gcs().send_text(MAV_SEVERITY_INFO,"Loiter to alt was stuck at %d", int(current_loc.alt/100));
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
	if (auto_state.wp_distance <= (uint32_t)MAX(get_wp_radius(),0) || 
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
    if (prev_WP_loc.same_latlon_as(next_WP_loc) &&
        steer_state.hold_course_cd != -1) {
        //keep flying the same course with fixed steering heading computed at start if cmd
        nav_controller->update_heading_hold(steer_state.hold_course_cd);
    }
    else {
        // Is the next_WP less than 200 m away?
        if (current_loc.get_distance(next_WP_loc) < 200.0f) {
            //push another 300 m down the line
            int32_t next_wp_bearing_cd = prev_WP_loc.get_bearing_to(next_WP_loc);
            next_WP_loc.offset_bearing(next_wp_bearing_cd * 0.01f, 300.0f);
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

// verify_nav_delay - check if we have waited long enough
bool Plane::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (arming.is_armed_and_safety_off()) {
        // don't delay while armed, we need a nav controller running
        return true;
    }
    if (millis() - nav_delay.time_start_ms > nav_delay.time_max_ms) {
        nav_delay.time_max_ms = 0;
        return true;
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

bool Plane::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.content.speed.speed_type)
    {
    case 0:             // Airspeed
        if (is_equal(cmd.content.speed.target_ms, -2.0f)) {
            new_airspeed_cm = -1; // return to default airspeed
            return true;
        } else if ((cmd.content.speed.target_ms >= aparm.airspeed_min.get()) &&
                   (cmd.content.speed.target_ms <= aparm.airspeed_max.get()))  {
            new_airspeed_cm = cmd.content.speed.target_ms * 100; //new airspeed target for AUTO or GUIDED modes
            gcs().send_text(MAV_SEVERITY_INFO, "Set airspeed %u m/s", (unsigned)cmd.content.speed.target_ms);
            return true;
        }
        break;
    case 1:             // Ground speed
        gcs().send_text(MAV_SEVERITY_INFO, "Set groundspeed %u", (unsigned)cmd.content.speed.target_ms);
        aparm.min_gndspeed_cm.set(cmd.content.speed.target_ms * 100);
        return true;
    }

    if (cmd.content.speed.throttle_pct > 0 && cmd.content.speed.throttle_pct <= 100) {
        gcs().send_text(MAV_SEVERITY_INFO, "Set throttle %u", (unsigned)cmd.content.speed.throttle_pct);
        aparm.throttle_cruise.set(cmd.content.speed.throttle_pct);
        return true;
    }

    return false;
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
    if (control_mode == &mode_auto) {
        return start_command(cmd);
    }
    return true;
}

// verify_command_callback - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool Plane::verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    if (control_mode == &mode_auto) {
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
    if (control_mode == &mode_auto) {
        set_mode(mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Mission complete, changing mode to RTL");
    }
}

#if HAL_QUADPLANE_ENABLED
bool Plane::verify_landing_vtol_approach(const AP_Mission::Mission_Command &cmd)
{
    const float radius = is_zero(quadplane.fw_land_approach_radius)? aparm.loiter_radius : quadplane.fw_land_approach_radius;
    const int8_t direction = is_negative(radius) ? -1 : 1;
    const float abs_radius = fabsf(radius);

    loiter.direction = direction;

    switch (vtol_approach_s.approach_stage) {
        case RTL:
            {
                // fly home and loiter at RTL alt
                nav_controller->update_loiter(cmd.content.location, abs_radius, direction);
                if (plane.reached_loiter_target()) {
                    // decend to Q RTL alt
                    plane.do_RTL(plane.home.alt + plane.quadplane.qrtl_alt*100UL);
                    plane.loiter_angle_reset();
                    vtol_approach_s.approach_stage = LOITER_TO_ALT;
                }
                break;
            }
        case LOITER_TO_ALT:
            {
                nav_controller->update_loiter(cmd.content.location, abs_radius, direction);

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
                // validate that the vehicle is at least the expected distance away from the loiter point
                // require an angle total of at least 2 centidegrees, due to special casing of 1 centidegree
                if (((fabsf(cmd.content.location.get_distance(current_loc) - abs_radius) > 5.0f) &&
                      (cmd.content.location.get_distance(current_loc) < abs_radius)) ||
                    (labs(loiter.sum_cd) < 2)) {
                    nav_controller->update_loiter(cmd.content.location, abs_radius, direction);
                    break;
                }
                vtol_approach_s.approach_stage = WAIT_FOR_BREAKOUT;
                FALLTHROUGH;
            }
        case WAIT_FOR_BREAKOUT:
            {
                nav_controller->update_loiter(cmd.content.location, radius, direction);

                const float breakout_direction_rad = radians(vtol_approach_s.approach_direction_deg + (direction > 0 ? 270 : 90));

                // breakout when within 5 degrees of the opposite direction
                if (fabsf(wrap_PI(ahrs.yaw - breakout_direction_rad)) < radians(5.0f)) {
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
                start.offset_bearing(vtol_approach_s.approach_direction_deg + 180, 1000);
                end.offset_bearing(vtol_approach_s.approach_direction_deg, 1000);

                nav_controller->update_waypoint(start, end);

                // check if we should move on to the next waypoint
                Location breakout_stopping_loc = cmd.content.location;
                breakout_stopping_loc.offset_bearing(vtol_approach_s.approach_direction_deg + 180, quadplane.stopping_distance());
                const bool past_finish_line = current_loc.past_interval_finish_line(start, breakout_stopping_loc);

                Location breakout_loc = cmd.content.location;
                breakout_loc.offset_bearing(vtol_approach_s.approach_direction_deg + 180, abs_radius);
                const bool half_radius = current_loc.line_path_proportion(breakout_loc, cmd.content.location) > 0.5;
                bool lined_up = true;
                Vector3f vel_NED;
                if (ahrs.get_velocity_NED(vel_NED)) {
                    const Vector2f target_vec = current_loc.get_distance_NE(cmd.content.location);
                    const float angle_err = fabsf(wrap_180(degrees(vel_NED.xy().angle(target_vec))));
                    lined_up = (angle_err < 30);
                }

                if (past_finish_line && (lined_up || half_radius)) {
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
#endif // HAL_QUADPLANE_ENABLED

bool Plane::verify_loiter_heading(bool init)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_auto()) {
        // skip heading verify if in VTOL auto
        return true;
    }
#endif

    //Get the lat/lon of next Nav waypoint after this one:
    AP_Mission::Mission_Command next_nav_cmd;
    if (! mission.get_next_nav_cmd(mission.get_current_nav_index() + 1,
                                   next_nav_cmd)) {
        //no next waypoint to shoot for -- go ahead and break out of loiter
        return true;
    }

    if (init) {
        loiter.sum_cd = 0;
    }

    return plane.mode_loiter.isHeadingLinedUp(next_WP_loc, next_nav_cmd.content.location);
}

float Plane::get_wp_radius() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_mode()) {
        return plane.quadplane.wp_nav->get_wp_radius_cm() * 0.01;
    }
#endif
    return g.waypoint_radius;
}

#if AP_SCRIPTING_ENABLED
/*
  support for scripted navigation, with verify operation for completion
 */
void Plane::do_nav_script_time(const AP_Mission::Mission_Command& cmd)
{
    nav_scripting.enabled = true;
    nav_scripting.id++;
    nav_scripting.start_ms = AP_HAL::millis();
    nav_scripting.current_ms = nav_scripting.start_ms;

    // start with current roll rate, pitch rate and throttle
    nav_scripting.roll_rate_dps = plane.rollController.get_pid_info().target;
    nav_scripting.pitch_rate_dps = plane.pitchController.get_pid_info().target;
    nav_scripting.yaw_rate_dps = degrees(ahrs.get_gyro().z);
    nav_scripting.throttle_pct = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
}

/*
  wait for scripting to say that the mission item is complete
 */
bool Plane::verify_nav_script_time(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.nav_script_time.timeout_s > 0) {
        const uint32_t now = AP_HAL::millis();
        if (now - nav_scripting.start_ms > cmd.content.nav_script_time.timeout_s*1000U) {
            gcs().send_text(MAV_SEVERITY_INFO, "NavScriptTime timed out");
            nav_scripting.enabled = false;
            nav_scripting.rudder_offset_pct = 0;
            nav_scripting.run_yaw_rate_controller = true;
        }
    }
    return !nav_scripting.enabled;
}

// check if we are in a NAV_SCRIPT_* command
bool Plane::nav_scripting_active(void)
{
    if (nav_scripting.enabled && AP_HAL::millis() - nav_scripting.current_ms > 1000) {
        // set_target_throttle_rate_rpy has not been called from script in last 1000ms
        nav_scripting.enabled = false;
        nav_scripting.current_ms = 0;
        nav_scripting.rudder_offset_pct = 0;
        nav_scripting.run_yaw_rate_controller = true;
        gcs().send_text(MAV_SEVERITY_INFO, "NavScript time out");
    }
    if (control_mode == &mode_auto &&
        mission.get_current_nav_cmd().id != MAV_CMD_NAV_SCRIPT_TIME) {
        nav_scripting.enabled = false;
    }
    return nav_scripting.enabled;
}

// support for NAV_SCRIPTING mission command
bool Plane::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
    if (!nav_scripting_active()) {
        // not in NAV_SCRIPT_TIME
        return false;
    }
    const auto &c = mission.get_current_nav_cmd().content.nav_script_time;
    id = nav_scripting.id;
    cmd = c.command;
    arg1 = c.arg1.get();
    arg2 = c.arg2.get();
    arg3 = c.arg3;
    arg4 = c.arg4;
    return true;
}

// called when script has completed the command
void Plane::nav_script_time_done(uint16_t id)
{
    if (id == nav_scripting.id) {
        nav_scripting.enabled = false;
    }
}

// support for NAV_SCRIPTING mission command and aerobatics in other allowed modes
void Plane::set_target_throttle_rate_rpy(float throttle_pct, float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps)
{
    nav_scripting.roll_rate_dps = constrain_float(roll_rate_dps, -g.acro_roll_rate, g.acro_roll_rate);
    nav_scripting.pitch_rate_dps = constrain_float(pitch_rate_dps, -g.acro_pitch_rate, g.acro_pitch_rate);
    nav_scripting.yaw_rate_dps = constrain_float(yaw_rate_dps, -g.acro_yaw_rate, g.acro_yaw_rate);
    nav_scripting.throttle_pct = constrain_float(throttle_pct, aparm.throttle_min, aparm.throttle_max);
    nav_scripting.current_ms = AP_HAL::millis();
}

// support for rudder offset override in aerobatic scripting
void Plane::set_rudder_offset(float rudder_pct, bool run_yaw_rate_controller)
{
    nav_scripting.rudder_offset_pct = rudder_pct;
    nav_scripting.run_yaw_rate_controller = run_yaw_rate_controller;
}

// enable NAV_SCRIPTING takeover in modes other than AUTO using script time mission commands
bool Plane::nav_scripting_enable(uint8_t mode)
{
   uint8_t current_control_mode = control_mode->mode_number();
   if (current_control_mode == mode) {
       switch (current_control_mode) {
       case Mode::Number::CIRCLE:
       case Mode::Number::STABILIZE:
       case Mode::Number::ACRO:
       case Mode::Number::FLY_BY_WIRE_A:
       case Mode::Number::FLY_BY_WIRE_B:
       case Mode::Number::CRUISE:
       case Mode::Number::LOITER:
           nav_scripting.enabled = true;
           nav_scripting.current_ms = AP_HAL::millis();
           break;
       default:
           nav_scripting.enabled = false;
       }
   } else {
       nav_scripting.enabled = false;
   }
   return nav_scripting.enabled;
}
#endif // AP_SCRIPTING_ENABLED

/*
  return true if this is a LAND command
  note that we consider a PAYLOAD_PLACE to be a land command as it
  follows the landing logic for quadplanes
 */
bool Plane::is_land_command(uint16_t command) const
{
    return
        command == MAV_CMD_NAV_VTOL_LAND ||
        command == MAV_CMD_NAV_LAND ||
        command == MAV_CMD_NAV_PAYLOAD_PLACE;
}

/*
  return true if in a specific AUTO mission command
 */
bool Plane::in_auto_mission_id(uint16_t command) const
{
    return control_mode == &mode_auto && mission.get_current_nav_id() == command;
}
