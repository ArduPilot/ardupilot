/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// forward declarations to make compiler happy
static void do_takeoff(const AP_Mission::Mission_Command& cmd);
static void do_nav_wp(const AP_Mission::Mission_Command& cmd);
static void do_land(const AP_Mission::Mission_Command& cmd);
static void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
static void do_loiter_turns(const AP_Mission::Mission_Command& cmd);
static void do_loiter_time(const AP_Mission::Mission_Command& cmd);
static void do_wait_delay(const AP_Mission::Mission_Command& cmd);
static void do_within_distance(const AP_Mission::Mission_Command& cmd);
static void do_change_alt(const AP_Mission::Mission_Command& cmd);
static void do_change_speed(const AP_Mission::Mission_Command& cmd);
static void do_set_home(const AP_Mission::Mission_Command& cmd);
static void do_continue_and_change_alt(const AP_Mission::Mission_Command& cmd);
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);


/********************************************************************************/
// Command Event Handlers
/********************************************************************************/

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
static bool
start_command(const AP_Mission::Mission_Command& cmd)
{
    // log when new commands start
    if (should_log(MASK_LOG_CMD)) {
        Log_Write_Cmd(cmd);
    }

    // special handling for nav vs non-nav commands
    if (AP_Mission::is_nav_cmd(cmd)) {
        // set land_complete to false to stop us zeroing the throttle
        auto_state.land_complete = false;
        auto_state.land_sink_rate = 0;

        // set takeoff_complete to true so we don't add extra evevator
        // except in a takeoff
        auto_state.takeoff_complete = true;

        // if a go around had been commanded, clear it now.
        auto_state.commanded_go_around = false;
        
        gcs_send_text_fmt(PSTR("Executing nav command ID #%i"),cmd.id);
    } else {
        gcs_send_text_fmt(PSTR("Executing command ID #%i"),cmd.id);
    }

    switch(cmd.id) {

    case MAV_CMD_NAV_TAKEOFF:
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

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        set_mode(RTL);
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        do_continue_and_change_alt(cmd);
        break;

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
            gcs_send_text_fmt(PSTR("Set inverted %u"), cmd.p1);
        }
        break;

    case MAV_CMD_DO_LAND_START:
        //ensure go around hasn't been set
        auto_state.commanded_go_around = false;
        break;

    case MAV_CMD_DO_FENCE_ENABLE:
#if GEOFENCE_ENABLED == ENABLED
        if (!geofence_set_enabled((bool) cmd.p1, AUTO_TOGGLED)) {
            gcs_send_text_fmt(PSTR("Unable to set fence enabled state to %u"), cmd.p1);
        } else {
            gcs_send_text_fmt(PSTR("Set fence enabled state to %u"), cmd.p1);
        }    
#endif
        break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        do_take_picture();
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        camera.set_trigger_distance(cmd.content.cam_trigg_dist.meters);
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
            // send the command to the camera mount
            camera_mount.set_roi_cmd(&cmd.content.location);
        }
        break;

    case MAV_CMD_DO_MOUNT_CONFIGURE:                    // Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
        camera_mount.configure_cmd();
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
        camera_mount.control_cmd();
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

static bool verify_command(const AP_Mission::Mission_Command& cmd)        // Returns true if command complete
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

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        return verify_continue_and_change_alt();

    // Conditional commands

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return verify_change_alt();
        break;

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
    case MAV_CMD_DO_MOUNT_CONTROL:
    case MAV_CMD_DO_INVERTED_FLIGHT:
    case MAV_CMD_DO_LAND_START:
    case MAV_CMD_DO_FENCE_ENABLE:
        return true;

    default:
        // error message
        if (AP_Mission::is_nav_cmd(cmd)) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_nav: Invalid or no current Nav cmd"));
        }else{
        gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_conditon: Invalid or no current Condition cmd"));
    }
        // return true so that we do not get stuck at this command
        return true;
    }
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

static void do_RTL(void)
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
        Log_Write_Mode(control_mode);
}

static void do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    prev_WP_loc = current_loc;
    set_next_WP(cmd.content.location);
    // pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
    auto_state.takeoff_pitch_cd        = (int16_t)cmd.p1 * 100;
    auto_state.takeoff_altitude_cm     = next_WP_loc.alt;
    next_WP_loc.lat = home.lat + 10;
    next_WP_loc.lng = home.lng + 10;
    auto_state.takeoff_speed_time_ms = 0;
    auto_state.takeoff_complete = false;                            // set flag to use gps ground course during TO.  IMU will be doing yaw drift correction
    // Flag also used to override "on the ground" throttle disable

    // zero locked course
    steer_state.locked_course_err = 0;
    
}

static void do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
}

static void do_land(const AP_Mission::Mission_Command& cmd)
{
    auto_state.commanded_go_around = false;
    set_next_WP(cmd.content.location);
}

static void loiter_set_direction_wp(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.location.flags.loiter_ccw) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
}

static void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
    loiter_set_direction_wp(cmd);
}

static void do_loiter_turns(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
    loiter.total_cd = (uint32_t)(LOWBYTE(cmd.p1)) * 36000UL;
    loiter_set_direction_wp(cmd);
}

static void do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    set_next_WP(cmd.content.location);
    // we set start_time_ms when we reach the waypoint
    loiter.start_time_ms = 0;
    loiter.time_max_ms = cmd.p1 * (uint32_t)1000;     // units are seconds
    loiter_set_direction_wp(cmd);
}

static void do_continue_and_change_alt(const AP_Mission::Mission_Command& cmd)
{
    next_WP_loc.alt = cmd.content.location.alt + home.alt;
    reset_offset_altitude();
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
static bool verify_takeoff()
{
    if (ahrs.yaw_initialised() && steer_state.hold_course_cd == -1) {
        const float min_gps_speed = 5;
        if (auto_state.takeoff_speed_time_ms == 0 && 
            gps.status() >= AP_GPS::GPS_OK_FIX_3D && 
            gps.ground_speed() > min_gps_speed) {
            auto_state.takeoff_speed_time_ms = hal.scheduler->millis();
        }
        if (auto_state.takeoff_speed_time_ms != 0 &&
            hal.scheduler->millis() - auto_state.takeoff_speed_time_ms >= 2000) {
            // once we reach sufficient speed for good GPS course
            // estimation we save our current GPS ground course
            // corrected for summed yaw to set the take off
            // course. This keeps wings level until we are ready to
            // rotate, and also allows us to cope with arbitary
            // compass errors for auto takeoff
            float takeoff_course = wrap_PI(radians(gps.ground_course_cd()*0.01)) - steer_state.locked_course_err;
            takeoff_course = wrap_PI(takeoff_course);
            steer_state.hold_course_cd = wrap_360_cd(degrees(takeoff_course)*100);
            gcs_send_text_fmt(PSTR("Holding course %ld at %.1fm/s (%.1f)"), 
                              steer_state.hold_course_cd,
                              gps.ground_speed(),
                              degrees(steer_state.locked_course_err));
        }
    }

    if (steer_state.hold_course_cd != -1) {
        // call navigation controller for heading hold
        nav_controller->update_heading_hold(steer_state.hold_course_cd);
    } else {
        nav_controller->update_level_flight();        
    }

    // see if we have reached takeoff altitude
    if (adjusted_altitude_cm() > auto_state.takeoff_altitude_cm) {
        steer_state.hold_course_cd = -1;
        auto_state.takeoff_complete = true;
        next_WP_loc = prev_WP_loc = current_loc;

#if GEOFENCE_ENABLED == ENABLED
        if (g.fence_autoenable == 1) {
            if (! geofence_set_enabled(true, AUTO_TOGGLED)) {
                gcs_send_text_P(SEVERITY_HIGH, PSTR("Enable fence failed (cannot autoenable"));
            } else {
                gcs_send_text_P(SEVERITY_HIGH, PSTR("Fence enabled. (autoenabled)"));
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
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd)
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
        gcs_send_text_fmt(PSTR("Reached Waypoint #%i dist %um"),
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)get_distance(current_loc, next_WP_loc));
        return true;
	}

    // have we flown past the waypoint?
    if (location_passed_point(current_loc, prev_WP_loc, next_WP_loc)) {
        gcs_send_text_fmt(PSTR("Passed Waypoint #%i dist %um"),
                          (unsigned)mission.get_current_nav_cmd().index,
                          (unsigned)get_distance(current_loc, next_WP_loc));
        return true;
    }

    return false;
}

static bool verify_loiter_unlim()
{
    update_loiter();
    return false;
}

static bool verify_loiter_time()
{
    update_loiter();
    if (loiter.start_time_ms == 0) {
        if (nav_controller->reached_loiter_target()) {
            // we've reached the target, start the timer
            loiter.start_time_ms = millis();
        }
    } else if ((millis() - loiter.start_time_ms) > loiter.time_max_ms) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("verify_nav: LOITER time complete"));
        return true;
    }
    return false;
}

static bool verify_loiter_turns()
{
    update_loiter();
    if (loiter.sum_cd > loiter.total_cd) {
        loiter.total_cd = 0;
        gcs_send_text_P(SEVERITY_LOW,PSTR("verify_nav: LOITER orbits complete"));
        // clear the command queue;
        return true;
    }
    return false;
}

static bool verify_RTL()
{
    update_loiter();
	if (auto_state.wp_distance <= (uint32_t)max(g.waypoint_radius,0) || 
        nav_controller->reached_loiter_target()) {
			gcs_send_text_P(SEVERITY_LOW,PSTR("Reached home"));
			return true;
    } else {
        return false;
	}
}

static bool verify_continue_and_change_alt()
{
    if (abs(adjusted_altitude_cm() - next_WP_loc.alt) <= 500) {
        return true;
    }
   
    // Is the next_WP less than 200 m away?
    if (get_distance(current_loc, next_WP_loc) < 200.f) {
        //push another 300 m down the line
        int32_t next_wp_bearing_cd = get_bearing_cd(prev_WP_loc, next_WP_loc);
        location_update(next_WP_loc, next_wp_bearing_cd * 0.01f, 300.f);
    }

    //keep flying the same course
    nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);

    return false;
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

static void do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value  = cmd.content.delay.seconds * 1000;    // convert seconds to milliseconds
}

/*
  process a DO_CHANGE_ALT request
 */
static void do_change_alt(const AP_Mission::Mission_Command& cmd)
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

static void do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters;
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
    if ((unsigned)(millis() - condition_start) > (unsigned)condition_value) {
        condition_value         = 0;
        return true;
    }
    return false;
}

static bool verify_change_alt()
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

static bool verify_within_distance()
{
    if (auto_state.wp_distance < max(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

static void do_loiter_at_location()
{
    if (g.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }
    next_WP_loc = current_loc;
}

static void do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.content.speed.speed_type)
    {
    case 0:             // Airspeed
        if (cmd.content.speed.target_ms > 0) {
            g.airspeed_cruise_cm.set(cmd.content.speed.target_ms * 100);
            gcs_send_text_fmt(PSTR("Set airspeed %u m/s"), (unsigned)cmd.content.speed.target_ms);
        }
        break;
    case 1:             // Ground speed
        gcs_send_text_fmt(PSTR("Set groundspeed %u"), (unsigned)cmd.content.speed.target_ms);
        g.min_gndspeed_cm.set(cmd.content.speed.target_ms * 100);
        break;
    }

    if (cmd.content.speed.throttle_pct > 0 && cmd.content.speed.throttle_pct <= 100) {
        gcs_send_text_fmt(PSTR("Set throttle %u"), (unsigned)cmd.content.speed.throttle_pct);
        aparm.throttle_cruise.set(cmd.content.speed.throttle_pct);
    }
}

static void do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        init_home();
    } else {
        ahrs.set_home(cmd.content.location);
        home_is_set = true;
    }
}

// do_take_picture - take a picture with the camera library
static void do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic();
    gcs_send_message(MSG_CAMERA_FEEDBACK);
    if (should_log(MASK_LOG_CAMERA)) {
        DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
    }
#endif
}

// start_command_callback - callback function called from ap-mission when it begins a new mission command
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
static bool start_command_callback(const AP_Mission::Mission_Command &cmd)
{
    if (control_mode == AUTO) {
        return start_command(cmd);
    }
    return true;
}

// verify_command_callback - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
static bool verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    if (control_mode == AUTO) {
        return verify_command(cmd);
    }
    return false;
}

// exit_mission_callback - callback function called from ap-mission when the mission has completed
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
static void exit_mission_callback()
{
    if (control_mode == AUTO) {
        gcs_send_text_fmt(PSTR("Returning to Home"));
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
