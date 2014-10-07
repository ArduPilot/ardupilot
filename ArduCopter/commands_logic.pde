/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// forward declarations to make compiler happy
static void do_takeoff(const AP_Mission::Mission_Command& cmd);
static void do_nav_wp(const AP_Mission::Mission_Command& cmd);
static void do_land(const AP_Mission::Mission_Command& cmd);
static void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
static void do_circle(const AP_Mission::Mission_Command& cmd);
static void do_loiter_time(const AP_Mission::Mission_Command& cmd);
static void do_spline_wp(const AP_Mission::Mission_Command& cmd);
#if NAV_GUIDED == ENABLED
static void do_nav_guided(const AP_Mission::Mission_Command& cmd);
#endif
static void do_wait_delay(const AP_Mission::Mission_Command& cmd);
static void do_within_distance(const AP_Mission::Mission_Command& cmd);
static void do_change_alt(const AP_Mission::Mission_Command& cmd);
static void do_yaw(const AP_Mission::Mission_Command& cmd);
static void do_change_speed(const AP_Mission::Mission_Command& cmd);
static void do_set_home(const AP_Mission::Mission_Command& cmd);
static void do_roi(const AP_Mission::Mission_Command& cmd);
#if PARACHUTE == ENABLED
static void do_parachute(const AP_Mission::Mission_Command& cmd);
#endif
#if EPM_ENABLED == ENABLED
static void do_gripper(const AP_Mission::Mission_Command& cmd);
#endif
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
static bool verify_circle(const AP_Mission::Mission_Command& cmd);
static bool verify_spline_wp(const AP_Mission::Mission_Command& cmd);
#if NAV_GUIDED == ENABLED
static bool verify_nav_guided(const AP_Mission::Mission_Command& cmd);
#endif
static void auto_spline_start(const Vector3f& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Vector3f& next_spline_destination);

// start_command - this function will be called when the ap_mission lib wishes to start a new command
static bool start_command(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: logging when new commands start/end
    if (g.log_bitmask & MASK_LOG_CMD) {
        Log_Write_Cmd(cmd);
    }

    switch(cmd.id) {

    ///
    /// navigation commands
    ///
    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_land(cmd);
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20
        do_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:           // 82  Navigate to Waypoint using spline
        do_spline_wp(cmd);
        break;

#if NAV_GUIDED == ENABLED
#ifdef MAV_CMD_NAV_GUIDED
    case MAV_CMD_NAV_GUIDED:             // 90  accept navigation commands from external nav computer
        do_nav_guided(cmd);
        break;
#endif
#endif

    //
    // conditional commands
    //
    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance(cmd);
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:             // 113
        do_change_alt(cmd);
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw(cmd);
        break;

    ///
    /// do commands
    ///
    case MAV_CMD_DO_CHANGE_SPEED:             // 178
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:             // 179
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

    case MAV_CMD_DO_SET_ROI:                // 201
        // point the copter and camera at a region of interest (ROI)
        do_roi(cmd);
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
    case MAV_CMD_DO_MOUNT_CONFIGURE:                    // Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
        camera_mount.configure_cmd();
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
        camera_mount.control_cmd();
        break;
#endif

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:                          // Mission command to configure or release parachute
        do_parachute(cmd);
        break;
#endif

#if EPM_ENABLED == ENABLED
    case MAV_CMD_DO_GRIPPER:                            // Mission command to control EPM gripper
        do_gripper(cmd);
        break;
#endif

    default:
        // do nothing with unrecognized MAVLink messages
        break;
    }

    // always return success
    return true;
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

// verify_command - this will be called repeatedly by ap_mission lib to ensure the active commands are progressing
//  should return true once the active navigation command completes successfully
//  called at 10hz or higher
static bool verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch(cmd.id) {

    //
    // navigation commands
    //
    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:
        return verify_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        return verify_spline_wp(cmd);
        break;

#if NAV_GUIDED == ENABLED
#ifdef MAV_CMD_NAV_GUIDED
    case MAV_CMD_NAV_GUIDED:
        return verify_nav_guided(cmd);
        break;
#endif
#endif

    ///
    /// conditional commands
    ///
    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return verify_change_alt();
        break;

    case MAV_CMD_CONDITION_YAW:
        return verify_yaw();
        break;

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:
        // assume parachute was released successfully
        return true;
        break;
#endif

    default:
        // return true if we do not recognise the command so that we move on to the next command
        return true;
        break;
    }
}

// exit_mission - function that is called once the mission completes
static void exit_mission()
{
    // if we are not on the ground switch to loiter or land
    if(!ap.land_complete) {
        // try to enter loiter but if that fails land
        if (!set_mode(LOITER)) {
            set_mode(LAND);
        }
    }else{
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (g.rc_3.control_in == 0 || failsafe.radio) {
            init_disarm_motors();
        }
#else
        // if we've landed it's safe to disarm
        init_disarm_motors();
#endif
    }
}

/********************************************************************************/
//
/********************************************************************************/

// do_RTL - start Return-to-Launch
static void do_RTL(void)
{
    // start rtl in auto flight mode
    auto_rtl_start();
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

// do_takeoff - initiate takeoff navigation command
static void do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    // Set wp navigation target to safe altitude above current position
    float takeoff_alt = cmd.content.location.alt;
    takeoff_alt = max(takeoff_alt,current_loc.alt);
    takeoff_alt = max(takeoff_alt,100.0f);
    auto_takeoff_start(takeoff_alt);
}

// do_nav_wp - initiate move to next waypoint
static void do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    const Vector3f &curr_pos = inertial_nav.get_position();
    Vector3f local_pos = pv_location_to_vector(cmd.content.location);

    // set target altitude to current altitude if not provided
    if (cmd.content.location.alt == 0) {
        local_pos.z = curr_pos.z;
    }

    // set lat/lon position to current position if not provided
    if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
        local_pos.x = curr_pos.x;
        local_pos.y = curr_pos.y;
    }

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = abs(cmd.p1);

    // Set wp navigation target
    auto_wp_start(local_pos);
    // if no delay set the waypoint as "fast"
    if (loiter_time_max == 0 ) {
        wp_nav.set_fast_waypoint(true);
    }
}

// do_land - initiate landing procedure
static void do_land(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: check if we have already landed

    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to fly to location
        land_state = LAND_STATE_FLY_TO_LOCATION;

        // calculate and set desired location above landing target
        Vector3f pos = pv_location_to_vector(cmd.content.location);
        pos.z = current_loc.alt;
        auto_wp_start(pos);
    }else{
        // set landing state
        land_state = LAND_STATE_DESCENDING;

        // initialise landing controller
        auto_land_start();
    }
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
static void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    Vector3f target_pos;

    // get current position
    Vector3f curr_pos = inertial_nav.get_position();

    // default to use position provided
    target_pos = pv_location_to_vector(cmd.content.location);

    // use current location if not provided
    if(cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
        wp_nav.get_wp_stopping_point_xy(target_pos);
    }

    // use current altitude if not provided
    // To-Do: use z-axis stopping point instead of current alt
    if( cmd.content.location.alt == 0 ) {
        target_pos.z = curr_pos.z;
    }

    // start way point navigator and provide it the desired location
    auto_wp_start(target_pos);
}

// do_circle - initiate moving in a circle
static void do_circle(const AP_Mission::Mission_Command& cmd)
{
    Vector3f curr_pos = inertial_nav.get_position();
    Vector3f circle_center = pv_location_to_vector(cmd.content.location);
    uint8_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1
    bool move_to_edge_required = false;

    // set target altitude if not provided
    if (cmd.content.location.alt == 0) {
        circle_center.z = curr_pos.z;
    } else {
        move_to_edge_required = true;
    }

    // set lat/lon position if not provided
    // To-Do: use previous command's destination if it was a straight line or spline waypoint command
    if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
        circle_center.x = curr_pos.x;
        circle_center.y = curr_pos.y;
    } else {
        move_to_edge_required = true;
    }

    // set circle controller's center
    circle_nav.set_center(circle_center);

    // set circle radius
    if (circle_radius_m != 0) {
        circle_nav.set_radius((float)circle_radius_m * 100.0f);
    }

    // check if we need to move to edge of circle
    if (move_to_edge_required) {
        // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
        auto_circle_movetoedge_start();
    } else {
        // start circling
        auto_circle_start();
    }
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
static void do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    Vector3f target_pos;

    // get current position
    Vector3f curr_pos = inertial_nav.get_position();

    // default to use position provided
    target_pos = pv_location_to_vector(cmd.content.location);

    // use current location if not provided
    if(cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
        wp_nav.get_wp_stopping_point_xy(target_pos);
    }

    // use current altitude if not provided
    if( cmd.content.location.alt == 0 ) {
        target_pos.z = curr_pos.z;
    }

    // start way point navigator and provide it the desired location
    auto_wp_start(target_pos);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}

// do_spline_wp - initiate move to next waypoint
static void do_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    Vector3f local_pos = pv_location_to_vector(cmd.content.location);

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = abs(cmd.p1);

    // determine segment start and end type
    bool stopped_at_start = true;
    AC_WPNav::spline_segment_end_type seg_end_type = AC_WPNav::SEGMENT_END_STOP;
    AP_Mission::Mission_Command temp_cmd;
    Vector3f next_destination;      // end of next segment

    // if previous command was a wp_nav command with no delay set stopped_at_start to false
    // To-Do: move processing of delay into wp-nav controller to allow it to determine the stopped_at_start value itself?
    uint16_t prev_cmd_idx = mission.get_prev_nav_cmd_index();
    if (prev_cmd_idx != AP_MISSION_CMD_INDEX_NONE) {
        if (mission.read_cmd_from_storage(prev_cmd_idx, temp_cmd)) {
            if ((temp_cmd.id == MAV_CMD_NAV_WAYPOINT || temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT) && temp_cmd.p1 == 0) {
                stopped_at_start = false;
            }
        }
    }

    // if there is no delay at the end of this segment get next nav command
    if (cmd.p1 == 0 && mission.get_next_nav_cmd(cmd.index+1, temp_cmd)) {
        // if the next nav command is a waypoint set end type to spline or straight
        if (temp_cmd.id == MAV_CMD_NAV_WAYPOINT) {
            seg_end_type = AC_WPNav::SEGMENT_END_STRAIGHT;
            next_destination = pv_location_to_vector(temp_cmd.content.location);
        }else if (temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT) {
            seg_end_type = AC_WPNav::SEGMENT_END_SPLINE;
            next_destination = pv_location_to_vector(temp_cmd.content.location);
        }
    }

    // set spline navigation target
    auto_spline_start(local_pos, stopped_at_start, seg_end_type, next_destination);
}

#if NAV_GUIDED == ENABLED
// do_nav_guided - initiate accepting commands from exernal nav computer
static void do_nav_guided(const AP_Mission::Mission_Command& cmd)
{
    // record start time so it can be compared vs timeout
    nav_guided.start_time = millis();

    // record start position so it can be compared vs horizontal limit
    nav_guided.start_position = inertial_nav.get_position();

    // set spline navigation target
    auto_nav_guided_start();
}
#endif  // NAV_GUIDED


#if PARACHUTE == ENABLED
// do_parachute - configure or release parachute
static void do_parachute(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.p1) {
        case PARACHUTE_DISABLE:
            parachute.enabled(false);
            Log_Write_Event(DATA_PARACHUTE_DISABLED);
            break;
        case PARACHUTE_ENABLE:
            parachute.enabled(true);
            Log_Write_Event(DATA_PARACHUTE_ENABLED);
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

#if EPM_ENABLED == ENABLED
// do_gripper - control EPM gripper
static void do_gripper(const AP_Mission::Mission_Command& cmd)
{
    // Note: we ignore the gripper num parameter because we only support one gripper
    switch (cmd.content.gripper.action) {
        case GRIPPER_ACTION_RELEASE:
            epm.release();
            Log_Write_Event(DATA_EPM_RELEASE);
            break;
        case GRIPPER_ACTION_GRAB:
            epm.grab();
            Log_Write_Event(DATA_EPM_GRAB);
            break;
        default:
            // do nothing
            break;
    }
}
#endif

/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - check if we have completed the takeoff
static bool verify_takeoff()
{
    // have we reached our target altitude?
    return wp_nav.reached_wp_destination();
}

// verify_land - returns true if landing has been completed
static bool verify_land()
{
    bool retval = false;

    switch( land_state ) {
        case LAND_STATE_FLY_TO_LOCATION:
            // check if we've reached the location
            if (wp_nav.reached_wp_destination()) {
                // get destination so we can use it for loiter target
                Vector3f dest = wp_nav.get_wp_destination();

                // initialise landing controller
                auto_land_start(dest);

                // advance to next state
                land_state = LAND_STATE_DESCENDING;
            }
            break;

        case LAND_STATE_DESCENDING:
            // rely on THROTTLE_LAND mode to correctly update landing status
            retval = ap.land_complete;
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully landed
    return retval;
}

// verify_nav_wp - check if we have reached the next way point
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if( !wp_nav.reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if(loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs_send_text_fmt(PSTR("Reached Command #%i"),cmd.index);
        return true;
    }else{
        return false;
    }
}

static bool verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
static bool verify_loiter_time()
{
    // return immediately if we haven't reached our destination
    if (!wp_nav.reached_wp_destination()) {
        return false;
    }

    // start our loiter timer
    if( loiter_time == 0 ) {
        loiter_time = millis();
    }

    // check if loiter timer has run out
    return (((millis() - loiter_time) / 1000) >= loiter_time_max);
}

// verify_circle - check if we have circled the point enough
static bool verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // check if we've reached the edge
    if (auto_mode == Auto_CircleMoveToEdge) {
        if (wp_nav.reached_wp_destination()) {
            Vector3f curr_pos = inertial_nav.get_position();
            Vector3f circle_center = pv_location_to_vector(cmd.content.location);

            // set target altitude if not provided
            if (circle_center.z == 0) {
                circle_center.z = curr_pos.z;
            }

            // set lat/lon position if not provided
            if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
                circle_center.x = curr_pos.x;
                circle_center.y = curr_pos.y;
            }

            // start circling
            auto_circle_start();
        }
        return false;
    }

    // check if we have completed circling
    return fabsf(circle_nav.get_angle_total()/(2*M_PI)) >= (float)LOWBYTE(cmd.p1);
}

// externs to remove compiler warning
extern bool rtl_state_complete;

// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
static bool verify_RTL()
{
    return (rtl_state_complete && (rtl_state == FinalDescent || rtl_state == Land));
}

// verify_spline_wp - check if we have reached the next way point using spline
static bool verify_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if( !wp_nav.reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if(loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs_send_text_fmt(PSTR("Reached Command #%i"),cmd.index);
        return true;
    }else{
        return false;
    }
}

#if NAV_GUIDED == ENABLED
// verify_nav_guided - check if we have breached any limits
static bool verify_nav_guided(const AP_Mission::Mission_Command& cmd)
{
    // check if we have passed the timeout
    if ((cmd.p1 > 0) && ((millis() - nav_guided.start_time) / 1000 >= cmd.p1)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position();

    // check if we have gone below min alt
    if (cmd.content.nav_guided.alt_min != 0 && (curr_pos.z / 100) < cmd.content.nav_guided.alt_min) {
        return true;
    }

    // check if we have gone above max alt
    if (cmd.content.nav_guided.alt_max != 0 && (curr_pos.z / 100) > cmd.content.nav_guided.alt_max) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (cmd.content.nav_guided.horiz_max != 0) {
        float horiz_move = pv_get_horizontal_distance_cm(nav_guided.start_position, curr_pos) / 100;
        if (horiz_move > cmd.content.nav_guided.horiz_max) {
            return true;
        }
    }

    // if we got here we should continue with the external nav controls
    return false;
}
#endif  // NAV_GUIDED


/********************************************************************************/
//	Condition (May) commands
/********************************************************************************/

static void do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = cmd.content.delay.seconds * 1000;     // convert seconds to milliseconds
}

static void do_change_alt(const AP_Mission::Mission_Command& cmd)
{
    // adjust target appropriately for each nav mode
    if (control_mode == AUTO) {
        switch (auto_mode) {
        case Auto_TakeOff:
            // To-Do: adjust waypoint target altitude to new provided altitude
            break;
        case Auto_WP:
        case Auto_Spline:
            // To-Do; reset origin to current location + stopping distance at new altitude
            break;
        case Auto_Land:
        case Auto_RTL:
            // ignore altitude
            break;
        case Auto_CircleMoveToEdge:
        case Auto_Circle:
            // move circle altitude up to target (we will need to store this target in circle class)
            break;
        case Auto_NavGuided:
            // ignore altitude
            break;
        }
    }
    // To-Do: store desired altitude in a variable so that it can be verified later
}

static void do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters * 100;
}

static void do_yaw(const AP_Mission::Mission_Command& cmd)
{
	set_auto_yaw_look_at_heading(
		cmd.content.yaw.angle_deg,
		cmd.content.yaw.turn_rate_dps,
		cmd.content.yaw.direction,
		cmd.content.yaw.relative_angle);
}


/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
    if (millis() - condition_start > (uint32_t)max(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

static bool verify_change_alt()
{
    // To-Do: use recorded target altitude to verify we have reached the target
    return true;
}

static bool verify_within_distance()
{
    // update distance calculation
    calc_wp_distance();
    if (wp_distance < max(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
static bool verify_yaw()
{
    // set yaw mode if it has been changed (the waypoint controller often retakes control of yaw as it executes a new waypoint command)
    if (auto_yaw_mode != AUTO_YAW_LOOK_AT_HEADING) {
        set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
    }

    // check if we are within 2 degrees of the target heading
    if (labs(wrap_180_cd(ahrs.yaw_sensor-yaw_look_at_heading)) <= 200) {
        return true;
    }else{
        return false;
    }
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/

// do_guided - start guided mode
static bool do_guided(const AP_Mission::Mission_Command& cmd)
{
    Vector3f pos_or_vel;    // target location or velocity

    // only process guided waypoint if we are in guided mode
    if (control_mode != GUIDED && !(control_mode == AUTO && auto_mode == Auto_NavGuided)) {
        return false;
    }

    // switch to handle different commands
    switch (cmd.id) {

        case MAV_CMD_NAV_WAYPOINT:
            // set wp_nav's destination
            pos_or_vel = pv_location_to_vector(cmd.content.location);
            guided_set_destination(pos_or_vel);
            return true;
            break;

#ifdef MAV_CMD_NAV_VELOCITY
        case MAV_CMD_NAV_VELOCITY:
            // set target velocity
            pos_or_vel.x = cmd.content.nav_velocity.x * 100.0f;
            pos_or_vel.y = cmd.content.nav_velocity.y * 100.0f;
            pos_or_vel.z = cmd.content.nav_velocity.z * 100.0f;
            guided_set_velocity(pos_or_vel);
            return true;
            break;
#endif

        case MAV_CMD_CONDITION_YAW:
            do_yaw(cmd);
            return true;
            break;

        default:
            // reject unrecognised command
            return false;
            break;
    }

    return true;
}

static void do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0) {
        wp_nav.set_speed_xy(cmd.content.speed.target_ms * 100.0f);
    }
}

static void do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if(cmd.p1 == 1) {
        init_home();
    } else {
        Location loc = cmd.content.location;
        ahrs.set_home(loc);
        set_home_is_set(true);
    }
}

// do_roi - starts actions required by MAV_CMD_NAV_ROI
//          this involves either moving the camera to point at the ROI (region of interest)
//          and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
//	TO-DO: add support for other features of MAV_CMD_DO_SET_ROI including pointing at a given waypoint
static void do_roi(const AP_Mission::Mission_Command& cmd)
{
    set_auto_yaw_roi(cmd.content.location);
}

// do_take_picture - take a picture with the camera library
static void do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic();
    if (g.log_bitmask & MASK_LOG_CAMERA) {
        DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
    }
#endif
}
