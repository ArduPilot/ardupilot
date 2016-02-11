/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
bool Plane::start_command(const AP_Mission::Mission_Command& cmd)
{
    // log when new commands start
    if (should_log(MASK_LOG_CMD)) {
        DataFlash.Log_Write_Mission_Cmd(mission, cmd);
    }

    // special handling for nav vs non-nav commands
    if (AP_Mission::is_nav_cmd(cmd)) {
        // set land_complete to false to stop us zeroing the throttle
        auto_state.land_complete = false;
        auto_state.land_pre_flare = false;
        auto_state.sink_rate = 0;

        // set takeoff_complete to true so we don't add extra evevator
        // except in a takeoff
        auto_state.takeoff_complete = true;

        // if a go around had been commanded, clear it now.
        auto_state.commanded_go_around = false;

        // start non-idle
        auto_state.idle_mode = false;
        
        // once landed, post some landing statistics to the GCS
        auto_state.post_landing_stats = false;

        // reset loiter start time. New command is a new loiter
        loiter.start_time_ms = 0;

        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Executing nav command ID #%i",cmd.id);
    } else {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Executing command ID #%i",cmd.id);
    }

    switch(cmd.id) {

    case MAV_CMD_NAV_TAKEOFF:
        crash_state.is_crashed = false;
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // LAND to Waypoint
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
        set_mode(RTL);
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
        crash_state.is_crashed = false;
        return quadplane.do_vtol_land(cmd);
        
    // Conditional commands

    case MAV_CMD_CONDITION_DELAY:
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        do_within_distance(cmd);
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        do_change_alt(cmd);
        break;

    // Do commands

    case MAV_CMD_DO_CHANGE_SPEED:
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_SET_SERVO:
        ServoRelayEvents.do_set_servo(cmd.content.servo.channel, cmd.content.servo.pwm);
        break;

    case MAV_CMD_DO_SET_RELAY:
        ServoRelayEvents.do_set_relay(cmd.content.relay.num, cmd.content.relay.state);
        break;

    case MAV_CMD_DO_REPEAT_SERVO:
        ServoRelayEvents.do_repeat_servo(cmd.content.repeat_servo.channel, cmd.content.repeat_servo.pwm,
                                         cmd.content.repeat_servo.repeat_count, cmd.content.repeat_servo.cycle_time * 1000.0f);
        break;

    case MAV_CMD_DO_REPEAT_RELAY:
        ServoRelayEvents.do_repeat_relay(cmd.content.repeat_relay.num, cmd.content.repeat_relay.repeat_count,
                                         cmd.content.repeat_relay.cycle_time * 1000.0f);
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:
        if (cmd.p1 == 0 || cmd.p1 == 1) {
            auto_state.inverted_flight = (bool)cmd.p1;
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set inverted %u", cmd.p1);
        }
        break;

    case MAV_CMD_DO_LAND_START:
        //ensure go around hasn't been set
        auto_state.commanded_go_around = false;
        break;

    case MAV_CMD_DO_FENCE_ENABLE:
#if GEOFENCE_ENABLED == ENABLED
        if (cmd.p1 != 2) {
            if (!geofence_set_enabled((bool) cmd.p1, AUTO_TOGGLED)) {
                gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Unable to set fence. Enabled state to %u", cmd.p1);
            } else {
                gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set fence enabled state to %u", cmd.p1);
            }
        } else { //commanding to only disable floor
            if (! geofence_set_floor_enabled(false)) {
                gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Unabled to disable fence floor");
            } else {
                gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Fence floor disabled");
            }
        }    
#endif
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        autotune_enable(cmd.p1);
        break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        do_digicam_configure(cmd);
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        // do_digicam_control Send Digicam Control message with the camera library
        do_digicam_control(cmd);
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        camera.set_trigger_distance(cmd.content.cam_trigg_dist.meters);
        break;
#endif

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:
        do_parachute(cmd);
        break;
#endif

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
    }

    return true;
}

/*******************************************************************************
Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
*******************************************************************************/

bool Plane::verify_command(const AP_Mission::Mission_Command& cmd)        // Returns true if command complete
{
    switch(cmd.id) {

    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();

    case MAV_CMD_NAV_LAND:
        return verify_land();

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlim();

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_loiter_turns();

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();

    case MAV_CMD_NAV_LOITER_TO_ALT:
        return verify_loiter_to_alt();

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        return verify_continue_and_change_alt();

    case MAV_CMD_NAV_ALTITUDE_WAIT:
        return verify_altitude_wait(cmd);

    // Conditional commands

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return verify_change_alt();

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:
        // assume parachute was released successfully
        return true;
        break;
#endif

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return quadplane.verify_vtol_takeoff(cmd);

    case MAV_CMD_NAV_VTOL_LAND:
        return quadplane.verify_vtol_land(cmd);
        
    // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_SERVO:
    case MAV_CMD_DO_SET_RELAY:
    case MAV_CMD_DO_REPEAT_SERVO:
    case MAV_CMD_DO_REPEAT_RELAY:
    case MAV_CMD_DO_CONTROL_VIDEO:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_NAV_ROI:
    case MAV_CMD_DO_MOUNT_CONFIGURE:
    case MAV_CMD_DO_INVERTED_FLIGHT:
    case MAV_CMD_DO_LAND_START:
    case MAV_CMD_DO_FENCE_ENABLE:
    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        return true;

    default:
        // error message
        if (AP_Mission::is_nav_cmd(cmd)) {
            gcs_send_text(MAV_SEVERITY_WARNING,"Verify nav. Invalid or no current nav cmd");
        }else{
        gcs_send_text(MAV_SEVERITY_WARNING,"Verify conditon. Invalid or no current condition cmd");
    }
        // return true so that we do not get stuck at this command
        return true;
    }
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

void Plane::do_RTL(void)
{
    auto_state.next_wp_no_crosstrack = true;
    auto_state.no_crosstrack = true;
    prev_WP_loc = current_loc;
    next_WP_loc = rally.calc_best_rally_or_home_location(current_loc, get_RTL_altitude());
    setup_terrain_target_alt(next_WP_loc);
    set_target_altitude_location(next_WP_loc);

    if (g.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    update_flight_stage();
    setup_glide_slope();
    setup_turn_angle();

    if (should_log(MASK_LOG_MODE))
        DataFlash.Log_Write_Mode(control_mode);
}

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
    auto_state.commanded_go_around = false;
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

#if RANGEFINDER_ENABLED == ENABLED
    // zero rangefinder state, start to accumulate good samples now
    memset(&rangefinder_state, 0, sizeof(rangefinder_state));
#endif
}

void Plane::loiter_set_direction_wp(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.location.flags.loiter_ccw) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
}

void Plane::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
    loiter_set_direction_wp(cmd);
}

void Plane::do_loiter_turns(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
    loiter.total_cd = (uint32_t)(LOWBYTE(cmd.p1)) * 36000UL;
    loiter_set_direction_wp(cmd);
}

void Plane::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
    // we set start_time_ms when we reach the waypoint
    loiter.time_max_ms = cmd.p1 * (uint32_t)1000;     // units are seconds
    loiter_set_direction_wp(cmd);
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
    } else if (ahrs.get_gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
        // use gps ground course based bearing hold
        steer_state.hold_course_cd = -1;
        bearing = ahrs.get_gps().ground_course_cd() * 0.01f;
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
    next_WP_loc.alt = cmd.content.location.alt;

    // convert relative alt to absolute alt
    if (cmd.content.location.flags.relative_alt) {
        next_WP_loc.flags.relative_alt = false;
        next_WP_loc.alt += home.alt;
    }

    //I know I'm storing this twice -- I'm doing that on purpose -- 
    //see verify_loiter_alt() function
    condition_value = next_WP_loc.alt;
    
    //are lat and lon 0?  if so, don't change the current wp lat/lon
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        set_next_WP(cmd.content.location);
    }
    //set loiter direction
    loiter_set_direction_wp(cmd);

    //must the plane be heading towards the next waypoint before breaking?
    condition_value2 = LOWBYTE(cmd.p1);
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
            gps.ground_speed() > min_gps_speed) {
            auto_state.takeoff_speed_time_ms = millis();
        }
        if (auto_state.takeoff_speed_time_ms != 0 &&
            millis() - auto_state.takeoff_speed_time_ms >= 2000) {
            // once we reach sufficient speed for good GPS course
            // estimation we save our current GPS ground course
            // corrected for summed yaw to set the take off
            // course. This keeps wings level until we are ready to
            // rotate, and also allows us to cope with arbitary
            // compass errors for auto takeoff
            float takeoff_course = wrap_PI(radians(gps.ground_course_cd()*0.01f)) - steer_state.locked_course_err;
            takeoff_course = wrap_PI(takeoff_course);
            steer_state.hold_course_cd = wrap_360_cd(degrees(takeoff_course)*100);
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "Holding course %ld at %.1fm/s (%.1f)",
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
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Takeoff complete at %.2fm",
                          (double)(relative_alt_cm*0.01f));
        steer_state.hold_course_cd = -1;
        auto_state.takeoff_complete = true;
        next_WP_loc = prev_WP_loc = current_loc;

#if GEOFENCE_ENABLED == ENABLED
        if (g.fence_autoenable > 0) {
            if (! geofence_set_enabled(true, AUTO_TOGGLED)) {
                gcs_send_text(MAV_SEVERITY_NOTICE, "Enable fence failed (cannot autoenable");
            } else {
                gcs_send_text(MAV_SEVERITY_INFO, "Fence enabled (autoenabled)");
            }
        }
#endif

        // don't cross-track on completion of takeoff, as otherwise we
        // can end up doing too sharp a turn
        auto_state.next_wp_no_crosstrack = true;
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

    if (auto_state.no_crosstrack) {
        nav_controller->update_waypoint(current_loc, next_WP_loc);
    } else {
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
    }

    // see if the user has specified a maximum distance to waypoint
    if (g.waypoint_max_radius > 0 && 
        auto_state.wp_distance > (uint16_t)g.waypoint_max_radius) {
        if (location_passed_point(current_loc, prev_WP_loc, next_WP_loc)) {
            // this is needed to ensure completion of the waypoint
            prev_WP_loc = current_loc;
        }
        return false;
    }

    float acceptance_distance = nav_controller->turn_distance(g.waypoint_radius, auto_state.next_turn_angle);
    if (cmd.p1 > 0) {
        // allow user to override acceptance radius
        acceptance_distance = cmd.p1;
    }
    
    if (auto_state.wp_distance <= acceptance_distance) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Reached waypoint #%i dist %um",
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)get_distance(current_loc, next_WP_loc));
        return true;
	}

    // have we flown past the waypoint?
    if (location_passed_point(current_loc, prev_WP_loc, next_WP_loc)) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Passed waypoint #%i dist %um",
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)get_distance(current_loc, next_WP_loc));
        return true;
    }

    return false;
}

bool Plane::verify_loiter_unlim()
{
    if (g.rtl_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
    update_loiter(abs(g.rtl_radius));
    return false;
}

bool Plane::verify_loiter_time()
{
    update_loiter(0);
    if (loiter.start_time_ms == 0) {
        if (nav_controller->reached_loiter_target()) {
            // we've reached the target, start the timer
            loiter.start_time_ms = millis();
        }
    } else if ((millis() - loiter.start_time_ms) > loiter.time_max_ms) {
        gcs_send_text(MAV_SEVERITY_WARNING,"Verify nav: LOITER time complete");
        return true;
    }
    return false;
}

bool Plane::verify_loiter_turns()
{
    update_loiter(0);
    if (loiter.sum_cd > loiter.total_cd) {
        loiter.total_cd = 0;
        gcs_send_text(MAV_SEVERITY_WARNING,"Verify nav: LOITER orbits complete");
        // clear the command queue;
        return true;
    }
    return false;
}

/*
  verify a LOITER_TO_ALT command. This involves checking we have
  reached both the desired altitude and desired heading. The desired
  altitude only needs to be reached once.
 */
bool Plane::verify_loiter_to_alt() 
{
    update_loiter(0);

    //has target altitude been reached?
    if (condition_value != 0) {
        if (labs(condition_value - current_loc.alt) < 500) {
            //Only have to reach the altitude once -- that's why I need
            //this global condition variable.
            //
            //This is in case of altitude oscillation when still trying
            //to reach the target heading.
            condition_value = 0;
        } else {
            return false;
        }
    }
    
    //has target heading been reached?
    if (condition_value2 != 0) {
        //Get the lat/lon of next Nav waypoint after this one:
        AP_Mission::Mission_Command next_nav_cmd;
        if (! mission.get_next_nav_cmd(mission.get_current_nav_index() + 1,
                                       next_nav_cmd)) {
            //no next waypoint to shoot for -- go ahead and break out of loiter
            return true;        
        }

        if (get_distance(next_WP_loc, next_nav_cmd.content.location) < labs(g.loiter_radius)) {
            /* Whenever next waypoint is within the loiter radius, 
               maintaining loiter would prevent us from ever pointing toward the next waypoint.
               Hence break out of loiter immediately
             */
            return true;
        }

        // Bearing in radians
        int32_t bearing_cd = get_bearing_cd(current_loc,next_nav_cmd.content.location);

        // get current heading. We should probably be using the ground
        // course instead to improve the accuracy in wind
        int32_t heading_cd = ahrs.yaw_sensor;

        int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);
 
        /*
          Check to see if the the plane is heading toward the land
          waypoint
          We use 10 degrees of slop so that we can handle 100
          degrees/second of yaw
        */
        if (labs(heading_err_cd) <= 1000) {
            //Want to head in a straight line from _here_ to the next waypoint.
            //DON'T want to head in a line from the center of the loiter to 
            //the next waypoint.
            //Therefore: mark the "last" (next_wp_loc is about to be updated)
            //wp lat/lon as the current location.
            next_WP_loc = current_loc;
            return true;
        } else {
            return false;
        }
    } 

    return true;
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
        nav_controller->reached_loiter_target()) {
			gcs_send_text(MAV_SEVERITY_INFO,"Reached HOME");
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
        gcs_send_text(MAV_SEVERITY_INFO,"Reached altitude");
        return true;
    }
    if (auto_state.sink_rate > cmd.content.altitude_wait.descent_rate) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Reached descent rate %.1f m/s", (double)auto_state.sink_rate);
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

/*
  process a DO_CHANGE_ALT request
 */
void Plane::do_change_alt(const AP_Mission::Mission_Command& cmd)
{
    condition_rate = labs((int)cmd.content.location.lat);   // climb rate in cm/s
    condition_value = cmd.content.location.alt;             // To-Do: ensure this altitude is an absolute altitude?
    if (condition_value < adjusted_altitude_cm()) {
        condition_rate = -condition_rate;
    }
    set_target_altitude_current_adjusted();
    change_target_altitude(condition_rate/10);
    next_WP_loc.alt = condition_value;                                      // For future nav calculations
    reset_offset_altitude();
    setup_glide_slope();
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

bool Plane::verify_change_alt()
{
    if( (condition_rate>=0 && adjusted_altitude_cm() >= condition_value) || 
        (condition_rate<=0 && adjusted_altitude_cm() <= condition_value)) {
        condition_value = 0;
        return true;
    }
    // condition_rate is climb rate in cm/s.  
    // We divide by 10 because this function is called at 10hz
    change_target_altitude(condition_rate/10);
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
    if (g.loiter_radius < 0) {
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
            g.airspeed_cruise_cm.set(cmd.content.speed.target_ms * 100);
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set airspeed %u m/s", (unsigned)cmd.content.speed.target_ms);
        }
        break;
    case 1:             // Ground speed
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set groundspeed %u", (unsigned)cmd.content.speed.target_ms);
        g.min_gndspeed_cm.set(cmd.content.speed.target_ms * 100);
        break;
    }

    if (cmd.content.speed.throttle_pct > 0 && cmd.content.speed.throttle_pct <= 100) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Set throttle %u", (unsigned)cmd.content.speed.throttle_pct);
        aparm.throttle_cruise.set(cmd.content.speed.throttle_pct);
    }
}

void Plane::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        init_home();
    } else {
        ahrs.set_home(cmd.content.location);
        home_is_set = HOME_SET_NOT_LOCKED;
        Log_Write_Home_And_Origin();
        GCS_MAVLINK::send_home_all(cmd.content.location);
    }
}

// do_digicam_configure Send Digicam Configure message with the camera library
void Plane::do_digicam_configure(const AP_Mission::Mission_Command& cmd)
{
#if CAMERA == ENABLED
    camera.configure(cmd.content.digicam_configure.shooting_mode,
                     cmd.content.digicam_configure.shutter_speed,
                     cmd.content.digicam_configure.aperture,
                     cmd.content.digicam_configure.ISO,
                     cmd.content.digicam_configure.exposure_type,
                     cmd.content.digicam_configure.cmd_id,
                     cmd.content.digicam_configure.engine_cutoff_time);
#endif
}

// do_digicam_control Send Digicam Control message with the camera library
void Plane::do_digicam_control(const AP_Mission::Mission_Command& cmd)
{
#if CAMERA == ENABLED
    if (camera.control(cmd.content.digicam_control.session,
                       cmd.content.digicam_control.zoom_pos,
                       cmd.content.digicam_control.zoom_step,
                       cmd.content.digicam_control.focus_lock,
                       cmd.content.digicam_control.shooting_cmd,
                       cmd.content.digicam_control.cmd_id)) {
        log_picture();
    }
#endif
}

// do_take_picture - take a picture with the camera library
void Plane::do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic(true);
    log_picture();
#endif
}

#if PARACHUTE == ENABLED
// do_parachute - configure or release parachute
void Plane::do_parachute(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.p1) {
        case PARACHUTE_DISABLE:
            parachute.enabled(false);
            break;
        case PARACHUTE_ENABLE:
            parachute.enabled(true);
            break;
        case PARACHUTE_RELEASE:
            parachute_release();
            break;
        default:
            // do nothing
            break;
    }
}
#endif

// log_picture - log picture taken and send feedback to GCS
void Plane::log_picture()
{
#if CAMERA == ENABLED
    if (!camera.using_feedback_pin()) {
        gcs_send_message(MSG_CAMERA_FEEDBACK);
        if (should_log(MASK_LOG_CAMERA)) {
            DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
        }
    } else {
        if (should_log(MASK_LOG_CAMERA)) {
            DataFlash.Log_Write_Trigger(ahrs, gps, current_loc);
        }      
    }
#endif
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
            gcs_send_mission_item_reached_message(cmd.index);
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
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Returning to HOME");
        memset(&auto_rtl_command, 0, sizeof(auto_rtl_command));
        auto_rtl_command.content.location = 
            rally.calc_best_rally_or_home_location(current_loc, get_RTL_altitude());
        auto_rtl_command.id = MAV_CMD_NAV_LOITER_UNLIM;
        setup_terrain_target_alt(auto_rtl_command.content.location);
        update_flight_stage();
        setup_glide_slope();
        setup_turn_angle();
        start_command(auto_rtl_command);
    }
}
