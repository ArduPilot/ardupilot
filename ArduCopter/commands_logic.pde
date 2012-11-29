/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
static void process_nav_command()
{
    switch(command_nav_queue.id) {

    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp();
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        yaw_mode                = YAW_HOLD;
        do_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_loiter_turns();
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20
        do_RTL();
        break;

    // point the copter and camera at a region of interest (ROI)
    case MAV_CMD_NAV_ROI:             // 80
        do_nav_roi();
        break;

    default:
        break;
    }

}

static void process_cond_command()
{
    switch(command_cond_queue.id) {

    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:             // 113
        do_change_alt();
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw();
        break;

    default:
        break;
    }
}

static void process_now_command()
{
    switch(command_cond_queue.id) {

    case MAV_CMD_DO_JUMP:              // 177
        do_jump();
        break;

    case MAV_CMD_DO_CHANGE_SPEED:             // 178
        do_change_speed();
        break;

    case MAV_CMD_DO_SET_HOME:             // 179
        do_set_home();
        break;

    case MAV_CMD_DO_SET_SERVO:             // 183
        do_set_servo();
        break;

    case MAV_CMD_DO_SET_RELAY:             // 181
        do_set_relay();
        break;

    case MAV_CMD_DO_REPEAT_SERVO:             // 184
        do_repeat_servo();
        break;

    case MAV_CMD_DO_REPEAT_RELAY:             // 182
        do_repeat_relay();
        break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
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

    default:
        // do nothing with unrecognized MAVLink messages
        break;
    }
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

static bool verify_must()
{
    switch(command_nav_queue.id) {

    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp();
        break;

    case MAV_CMD_NAV_LAND:
        return verify_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_loiter_turns();
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();
        break;

    case MAV_CMD_NAV_ROI:             // 80
        return verify_nav_roi();
        break;

    default:
        //gcs_send_text_P(SEVERITY_HIGH,PSTR("<verify_must: default> No current Must commands"));
        return false;
        break;
    }
}

static bool verify_may()
{
    switch(command_cond_queue.id) {

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

    default:
        //gcs_send_text_P(SEVERITY_HIGH,PSTR("<verify_must: default> No current May commands"));
        return false;
        break;
    }
}

/********************************************************************************/
//
/********************************************************************************/

// do_RTL - start Return-to-Launch
static void do_RTL(void)
{
    // set rtl state
    rtl_state = RTL_STATE_RETURNING_HOME;

    // set roll, pitch and yaw modes
    roll_pitch_mode     = RTL_RP;
    yaw_mode            = YAW_AUTO;
    auto_yaw_tracking   = MAV_ROI_WPNEXT;
    set_throttle_mode(RTL_THR);

    // set navigation mode
    wp_control = WP_MODE;

    // so we know where we are navigating from
    next_WP = current_loc;

    // Set navigation target to home
    set_next_WP(&home);

    // override altitude to RTL altitude
    set_new_altitude(get_RTL_alt());

    // output control mode to the ground station
    // -----------------------------------------
    gcs_send_message(MSG_HEARTBEAT);
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

static void do_takeoff()
{
    wp_control = LOITER_MODE;

    // Start with current location
    Location temp = current_loc;

    // alt is always relative
    temp.alt = command_nav_queue.alt;

    // prevent flips
    reset_I_all();

    // Set our waypoint
    set_next_WP(&temp);
}

static void do_nav_wp()
{
    wp_control = WP_MODE;

    set_next_WP(&command_nav_queue);

    // this is our bitmask to verify we have met all conditions to move on
    wp_verify_byte  = 0;

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time     = 0;

    // this is the delay, stored in seconds and expanded to millis
    loiter_time_max = command_nav_queue.p1 * 1000;

    if((next_WP.options & WP_OPTION_ALT_REQUIRED) == false) {
        wp_verify_byte |= NAV_ALTITUDE;
    }
}

static void do_land()
{
    wp_control = LOITER_MODE;
    set_throttle_mode(THROTTLE_LAND);

    // hold at our current location
    set_next_WP(&current_loc);
}

static void do_loiter_unlimited()
{
    wp_control = LOITER_MODE;

    //cliSerial->println("dloi ");
    if(command_nav_queue.lat == 0) {
        set_next_WP(&current_loc);
        wp_control = LOITER_MODE;
    }else{
        set_next_WP(&command_nav_queue);
        wp_control = WP_MODE;
    }
}

static void do_loiter_turns()
{
    wp_control = CIRCLE_MODE;

    if(command_nav_queue.lat == 0) {
        // allow user to specify just the altitude
        if(command_nav_queue.alt > 0) {
            current_loc.alt = command_nav_queue.alt;
        }
        set_next_WP(&current_loc);
    }else{
        set_next_WP(&command_nav_queue);
    }

    circle_WP = next_WP;

    loiter_total = command_nav_queue.p1 * 360;
    loiter_sum       = 0;
    old_target_bearing = target_bearing;

    circle_angle = target_bearing + 18000;
    circle_angle = wrap_360(circle_angle);
    circle_angle *= RADX100;
}

static void do_loiter_time()
{
    if(command_nav_queue.lat == 0) {
        wp_control              = LOITER_MODE;
        loiter_time     = millis();
        set_next_WP(&current_loc);
    }else{
        wp_control              = WP_MODE;
        set_next_WP(&command_nav_queue);
    }

    loiter_time_max = command_nav_queue.p1 * 1000;     // units are (seconds)
}

/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

static bool verify_takeoff()
{
    // wait until we are ready!
    if(g.rc_3.control_in == 0) {
        return false;
    }
    // are we above our target altitude?
    return (current_loc.alt > next_WP.alt);
}

// verify_land - returns true if landing has been completed
static bool verify_land()
{
    // loiter above 3m
    if(current_loc.alt > 300) {
        wp_control = LOITER_MODE;
    }

    // turn off loiter below 1m
    // To-Do: instead of turning off loiter we should make loiter less aggressive
    if(current_loc.alt < 100 ) {
        wp_control      = NO_NAV_MODE;
    }

    // rely on THROTTLE_LAND mode to correctly update landing status
    return ap.land_complete;
}

static bool verify_nav_wp()
{
    // Altitude checking
    if(next_WP.options & WP_OPTION_ALT_REQUIRED) {
        // we desire a certain minimum altitude
        if(alt_change_flag == REACHED_ALT) {

            // we have reached that altitude
            wp_verify_byte |= NAV_ALTITUDE;
        }
    }

    // Did we pass the WP?	// Distance checking
    if((wp_distance <= (g.waypoint_radius * 100)) || check_missed_wp()) {

        // if we have a distance calc error, wp_distance may be less than 0
        if(wp_distance > 0) {
            wp_verify_byte |= NAV_LOCATION;

            if(loiter_time == 0) {
                loiter_time = millis();
            }
        }
    }

    // Hold at Waypoint checking, we cant move on until this is OK
    if(wp_verify_byte & NAV_LOCATION) {
        // we have reached our goal

        // loiter at the WP
        wp_control      = LOITER_MODE;

        if ((millis() - loiter_time) > loiter_time_max) {
            wp_verify_byte |= NAV_DELAY;
            //gcs_send_text_P(SEVERITY_LOW,PSTR("verify_must: LOITER time complete"));
            //cliSerial->println("vlt done");
        }
    }

    if(wp_verify_byte >= 7) {
        //if(wp_verify_byte & NAV_LOCATION){
        gcs_send_text_fmt(PSTR("Reached Command #%i"),command_nav_index);
        wp_verify_byte = 0;
        copter_leds_nav_blink = 15;             // Cause the CopterLEDs to blink three times to indicate waypoint reached
        return true;
    }else{
        return false;
    }
}

static bool verify_loiter_unlimited()
{
    if(wp_control == WP_MODE &&  wp_distance <= (g.waypoint_radius * 100)) {
        // switch to position hold
        wp_control      = LOITER_MODE;
    }
    return false;
}

static bool verify_loiter_time()
{
    if(wp_control == LOITER_MODE) {
        if ((millis() - loiter_time) > loiter_time_max) {
            return true;
        }
    }
    if(wp_control == WP_MODE &&  wp_distance <= (g.waypoint_radius * 100)) {
        // reset our loiter time
        loiter_time = millis();
        // switch to position hold
        wp_control      = LOITER_MODE;
    }
    return false;
}

static bool verify_loiter_turns()
{
    //cliSerial->printf("loiter_sum: %d \n", loiter_sum);
    // have we rotated around the center enough times?
    // -----------------------------------------------
    if(abs(loiter_sum) > loiter_total) {
        loiter_total    = 0;
        loiter_sum              = 0;
        //gcs_send_text_P(SEVERITY_LOW,PSTR("verify_must: LOITER orbits complete"));
        // clear the command queue;
        return true;
    }
    return false;
}

// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
static bool verify_RTL()
{
    bool retval = false;

    switch( rtl_state ) {

        case RTL_STATE_RETURNING_HOME:
            // if we've reached home initiate loiter
            if (wp_distance <= g.waypoint_radius * 100 || check_missed_wp()) {
                rtl_state = RTL_STATE_LOITERING_AT_HOME;
                wp_control = LOITER_MODE;

                // set loiter timer
                rtl_loiter_start_time = millis();

                // give pilot back control of yaw
                yaw_mode = YAW_HOLD;
            }
            break;

        case RTL_STATE_LOITERING_AT_HOME:
            // check if we've loitered long enough
            if( millis() - rtl_loiter_start_time > (uint32_t)g.rtl_loiter_time.get() ) {
                // initiate landing or descent
                if(g.rtl_alt_final == 0) {
                    // land
                    do_land();
                    // override landing location (do_land defaults to current location)
                    next_WP.lat = home.lat;
                    next_WP.lng = home.lng;
                    // update RTL state
                    rtl_state = RTL_STATE_LAND;
                }else{
                    // descend
                    if(current_loc.alt > g.rtl_alt_final) {
                        set_new_altitude(g.rtl_alt_final);
                    }
                    // update RTL state
                    rtl_state = RTL_STATE_FINAL_DESCENT;
                }
            }
            break;

        case RTL_STATE_FINAL_DESCENT:
            // rely on altitude check to confirm we have reached final altitude
            if(current_loc.alt <= g.rtl_alt_final || alt_change_flag == REACHED_ALT) {
                // switch to regular loiter mode
                set_mode(LOITER);
                // override location and altitude
                set_next_WP(&home);
                // override altitude to RTL altitude
                set_new_altitude(g.rtl_alt_final);
                retval = true;
            }
            break;

        case RTL_STATE_LAND:
            // rely on verify_land to return correct status
            retval = verify_land();
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully completed RTL
    return retval;
}

/********************************************************************************/
//	Condition (May) commands
/********************************************************************************/

static void do_wait_delay()
{
    //cliSerial->print("dwd ");
    condition_start = millis();
    condition_value = command_cond_queue.lat * 1000;     // convert to milliseconds
    //cliSerial->println(condition_value,DEC);
}

static void do_change_alt()
{
    Location temp   = next_WP;
    condition_start = current_loc.alt;
    //condition_value	= command_cond_queue.alt;
    temp.alt                = command_cond_queue.alt;
    set_next_WP(&temp);
}

static void do_within_distance()
{
    condition_value  = command_cond_queue.lat * 100;
}

static void do_yaw()
{
    //cliSerial->println("dyaw ");
    auto_yaw_tracking = MAV_ROI_NONE;

    // target angle in degrees
    command_yaw_start               = nav_yaw;     // current position
    command_yaw_start_time  = millis();

    command_yaw_dir                 = command_cond_queue.p1;                            // 1 = clockwise,	 0 = counterclockwise
    command_yaw_speed               = command_cond_queue.lat * 100;             // ms * 100
    command_yaw_relative    = command_cond_queue.lng;                           // 1 = Relative,	 0 = Absolute

    // if unspecified turn at 30° per second
    if(command_yaw_speed == 0)
        command_yaw_speed = 3000;

    // ensure direction is valid, if invalid default to counter clockwise
    if(command_yaw_dir > 1)
        command_yaw_dir = 0;            // 0 = counter clockwise, 1 = clockwise

    if(command_yaw_relative == 1) {
        // relative
        command_yaw_delta       = command_cond_queue.alt * 100;
        if(command_yaw_dir == 0) {              // 0 = counter clockwise, 1 = clockwise
            command_yaw_end = command_yaw_start - command_yaw_delta;
        }else{
            command_yaw_end = command_yaw_start + command_yaw_delta;
        }
        command_yaw_end = wrap_360(command_yaw_end);
    }else{
        // absolute
        command_yaw_end         = command_cond_queue.alt * 100;

        // calculate the delta travel in deg * 100
        if(command_yaw_dir == 0) {              // 0 = counter clockwise, 1 = clockwise
            if(command_yaw_start > command_yaw_end) {
                command_yaw_delta = command_yaw_start - command_yaw_end;
            }else{
                command_yaw_delta = 36000 + (command_yaw_start - command_yaw_end);
            }
        }else{
            if(command_yaw_start >= command_yaw_end) {
                command_yaw_delta = 36000 - (command_yaw_start - command_yaw_end);
            }else{
                command_yaw_delta = command_yaw_end - command_yaw_start;
            }
        }
        command_yaw_delta = wrap_360(command_yaw_delta);
    }

    // rate to turn deg per second - default is ten
    command_yaw_time        = (command_yaw_delta / command_yaw_speed) * 1000;
}


/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
    //cliSerial->print("vwd");
    if ((unsigned)(millis() - condition_start) > (unsigned)condition_value) {
        //cliSerial->println("y");
        condition_value = 0;
        return true;
    }
    //cliSerial->println("n");
    return false;
}

static bool verify_change_alt()
{
    //cliSerial->printf("change_alt, ca:%d, na:%d\n", (int)current_loc.alt, (int)next_WP.alt);
    if ((int32_t)condition_start < next_WP.alt) {
        // we are going higer
        if(current_loc.alt > next_WP.alt) {
            return true;
        }
    }else{
        // we are going lower
        if(current_loc.alt < next_WP.alt) {
            return true;
        }
    }
    return false;
}

static bool verify_within_distance()
{
    //cliSerial->printf("cond dist :%d\n", (int)condition_value);
    if (wp_distance < condition_value) {
        condition_value = 0;
        return true;
    }
    return false;
}

static bool verify_yaw()
{
    //cliSerial->printf("vyaw %d\n", (int)(nav_yaw/100));

    if((millis() - command_yaw_start_time) > command_yaw_time) {
        // time out
        // make sure we hold at the final desired yaw angle
        nav_yaw         = command_yaw_end;
        auto_yaw        = nav_yaw;

        // TO-DO: there's still a problem with Condition_yaw, it will do it two times(probably more) sometimes, if it hasn't reached the next waypoint yet.
        // it should only do it one time so there should be code here to prevent another Condition_Yaw.

        //cliSerial->println("Y");
        return true;

    }else{
        // else we need to be at a certain place
        // power is a ratio of the time : .5 = half done
        float power = (float)(millis() - command_yaw_start_time) / (float)command_yaw_time;

        if(command_yaw_dir == 0) {              // 0 = counter clockwise, 1 = clockwise
            nav_yaw         = command_yaw_start - ((float)command_yaw_delta * power );
        }else{
            nav_yaw         = command_yaw_start + ((float)command_yaw_delta * power );
        }
        nav_yaw         = wrap_360(nav_yaw);
        auto_yaw        = nav_yaw;
        //cliSerial->printf("ny %ld\n",nav_yaw);
        return false;
    }
}

// verify_nav_roi - verifies that actions required by MAV_CMD_NAV_ROI have completed
//              we assume the camera command has been successfully implemented by the do_nav_roi command
//              so all we need to check is whether we needed to yaw the copter (due to the mount type) and
//              whether that yaw has completed
//	TO-DO: add support for other features of MAV_NAV_ROI including pointing at a given waypoint
static bool verify_nav_roi()
{
#if MOUNT == ENABLED
    // check if mount type requires us to rotate the quad
    if( camera_mount.get_mount_type() != AP_Mount::k_pan_tilt && camera_mount.get_mount_type() != AP_Mount::k_pan_tilt_roll ) {
        // ensure yaw has gotten to within 2 degrees of the target
        if( labs(wrap_180(ahrs.yaw_sensor-auto_yaw)) <= 200 ) {
            nav_yaw = auto_yaw;                 // ensure target yaw for YAW_HOLD is our desired yaw
            return true;
        }else{
            return false;
        }
    }else{
        // if no rotation required, assume the camera instruction was implemented immediately
        return true;
    }
#else
    // if we have no camera mount simply check we've reached the desired yaw
    // ensure yaw has gotten to within 2 degrees of the target
    if( abs(wrap_180(ahrs.yaw_sensor-auto_yaw)) <= 200 ) {
        nav_yaw = auto_yaw;             // ensure target yaw for YAW_HOLD is our desired yaw
        return true;
    }else{
        return false;
    }
#endif
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/

static void do_change_speed()
{
    g.waypoint_speed_max = command_cond_queue.p1 * 100;
}

static void do_target_yaw()
{
    auto_yaw_tracking = command_cond_queue.p1;

    if(auto_yaw_tracking == MAV_ROI_LOCATION) {
        target_WP = command_cond_queue;
    }
}

static void do_loiter_at_location()
{
    next_WP = current_loc;
}

static void do_jump()
{
    // Used to track the state of the jump command in Mission scripting
    // -10 is a value that means the register is unused
    // when in use, it contains the current remaining jumps
    static int8_t jump = -10;                                                                   // used to track loops in jump command

    //cliSerial->printf("do Jump: %d\n", jump);

    if(jump == -10) {
        //cliSerial->printf("Fresh Jump\n");
        // we use a locally stored index for jump
        jump = command_cond_queue.lat;
    }
    //cliSerial->printf("Jumps left: %d\n",jump);

    if(jump > 0) {
        //cliSerial->printf("Do Jump to %d\n",command_cond_queue.p1);
        jump--;
        change_command(command_cond_queue.p1);

    } else if (jump == 0) {
        //cliSerial->printf("Did last jump\n");
        // we're done, move along
        jump = -11;

    } else if (jump == -1) {
        //cliSerial->printf("jumpForever\n");
        // repeat forever
        change_command(command_cond_queue.p1);
    }
}

static void do_set_home()
{
    if(command_cond_queue.p1 == 1) {
        init_home();
    } else {
        home.id         = MAV_CMD_NAV_WAYPOINT;
        home.lng        = command_cond_queue.lng;                                       // Lon * 10**7
        home.lat        = command_cond_queue.lat;                                       // Lat * 10**7
        home.alt        = 0;
        //home_is_set 	= true;
        set_home_is_set(true);
    }
}

static void do_set_servo()
{
    APM_RC.OutputCh(command_cond_queue.p1 - 1, command_cond_queue.alt);
}

static void do_set_relay()
{
    if (command_cond_queue.p1 == 1) {
        relay.on();
    } else if (command_cond_queue.p1 == 0) {
        relay.off();
    }else{
        relay.toggle();
    }
}

static void do_repeat_servo()
{
    event_id = command_cond_queue.p1 - 1;

    if(command_cond_queue.p1 >= CH_5 + 1 && command_cond_queue.p1 <= CH_8 + 1) {

        event_timer             = 0;
        event_value             = command_cond_queue.alt;
        event_repeat    = command_cond_queue.lat * 2;
        event_delay             = command_cond_queue.lng * 500.0;         // /2 (half cycle time) * 1000 (convert to milliseconds)

        switch(command_cond_queue.p1) {
        case CH_5:
            event_undo_value = g.rc_5.radio_trim;
            break;
        case CH_6:
            event_undo_value = g.rc_6.radio_trim;
            break;
        case CH_7:
            event_undo_value = g.rc_7.radio_trim;
            break;
        case CH_8:
            event_undo_value = g.rc_8.radio_trim;
            break;
        }
        update_events();
    }
}

static void do_repeat_relay()
{
    event_id                = RELAY_TOGGLE;
    event_timer             = 0;
    event_delay             = command_cond_queue.lat * 500.0;     // /2 (half cycle time) * 1000 (convert to milliseconds)
    event_repeat    = command_cond_queue.alt * 2;
    update_events();
}

// do_nav_roi - starts actions required by MAV_CMD_NAV_ROI
//              this involves either moving the camera to point at the ROI (region of interest)
//              and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
//				Note: the ROI should already be in the command_nav_queue global variable
//	TO-DO: add support for other features of MAV_NAV_ROI including pointing at a given waypoint
static void do_nav_roi()
{
#if MOUNT == ENABLED

    // check if mount type requires us to rotate the quad
    if( camera_mount.get_mount_type() != AP_Mount::k_pan_tilt && camera_mount.get_mount_type() != AP_Mount::k_pan_tilt_roll ) {
        auto_yaw_tracking = MAV_ROI_LOCATION;
        target_WP = command_nav_queue;
        auto_yaw = get_bearing_cd(&current_loc, &target_WP);
    }
    // send the command to the camera mount
    camera_mount.set_roi_cmd(&command_nav_queue);

    // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
    //		0: do nothing
    //		1: point at next waypoint
    //		2: point at a waypoint taken from WP# parameter (2nd parameter?)
    //		3: point at a location given by alt, lon, lat parameters
    //		4: point at a target given a target id (can't be implmented)
#else
    // if we have no camera mount simply rotate the quad
    auto_yaw_tracking = MAV_ROI_LOCATION;
    target_WP = command_nav_queue;
    auto_yaw = get_bearing_cd(&current_loc, &target_WP);
#endif
}
