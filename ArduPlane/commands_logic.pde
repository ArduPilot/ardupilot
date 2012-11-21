/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
static void
handle_process_nav_cmd()
{
    // set land_complete to false to stop us zeroing the throttle
    land_complete = false;

    // set takeoff_complete to true so we don't add extra evevator
    // except in a takeoff 
    takeoff_complete = true;

    gcs_send_text_fmt(PSTR("Executing nav command ID #%i"),next_nav_command.id);
    switch(next_nav_command.id) {

    case MAV_CMD_NAV_TAKEOFF:
        do_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // Navigate to Waypoint
        do_nav_wp();
        break;

    case MAV_CMD_NAV_LAND:              // LAND to Waypoint
        do_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // Loiter indefinitely
        do_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              // Loiter N Times
        do_loiter_turns();
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        do_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        do_RTL();
        break;

    default:
        break;
    }
}

static void
handle_process_condition_command()
{
    gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nonnav_command.id);
    switch(next_nonnav_command.id) {

    case MAV_CMD_CONDITION_DELAY:
        do_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        do_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        do_change_alt();
        break;

    default:
        break;
    }
}

static void handle_process_do_command()
{
    gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nonnav_command.id);
    switch(next_nonnav_command.id) {

    case MAV_CMD_DO_JUMP:
        do_jump();
        break;

    case MAV_CMD_DO_CHANGE_SPEED:
        do_change_speed();
        break;

    case MAV_CMD_DO_SET_HOME:
        do_set_home();
        break;

    case MAV_CMD_DO_SET_SERVO:
        do_set_servo();
        break;

    case MAV_CMD_DO_SET_RELAY:
        do_set_relay();
        break;

    case MAV_CMD_DO_REPEAT_SERVO:
        do_repeat_servo(next_nonnav_command.p1, next_nonnav_command.alt,
                        next_nonnav_command.lat, next_nonnav_command.lng);
        break;

    case MAV_CMD_DO_REPEAT_RELAY:
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
    // Sets the region of interest (ROI) for a sensor set or the
    // vehicle itself. This can then be used by the vehicles control
    // system to control the vehicle attitude and the attitude of various
    // devices such as cameras.
    //    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
    case MAV_CMD_NAV_ROI:
 #if 0
        // send the command to the camera mount
        camera_mount.set_roi_cmd(&command_nav_queue);
 #else
        gcs_send_text_P(SEVERITY_LOW, PSTR("DO_SET_ROI not supported"));
 #endif
        break;

    case MAV_CMD_DO_MOUNT_CONFIGURE:                    // Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
        camera_mount.configure_cmd();
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
        camera_mount.control_cmd();
        break;
#endif
    }
}

static void handle_no_commands()
{
    gcs_send_text_fmt(PSTR("Returning to Home"));
    next_nav_command = home;
    next_nav_command.alt = read_alt_to_hold();
    next_nav_command.id = MAV_CMD_NAV_LOITER_UNLIM;
    nav_command_ID = MAV_CMD_NAV_LOITER_UNLIM;
    non_nav_command_ID = WAIT_COMMAND;
    handle_process_nav_cmd();

}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

static bool verify_nav_command()        // Returns true if command complete
{
    switch(nav_command_ID) {

    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();

    case MAV_CMD_NAV_LAND:
        return verify_land();

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp();

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlim();

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_loiter_turns();

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    default:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_nav: Invalid or no current Nav cmd"));
    }
    return false;
}

static bool verify_condition_command()          // Returns true if command complete
{
    switch(non_nav_command_ID) {
    case NO_COMMAND:
        break;

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return verify_change_alt();
        break;

    case WAIT_COMMAND:
        return 0;
        break;


    default:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_conditon: Invalid or no current Condition cmd"));
        break;
    }
    return false;
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

static void do_RTL(void)
{
    control_mode    = RTL;
    crash_timer     = 0;
    next_WP                 = home;

    // Altitude to hold over home
    // Set by configuration tool
    // -------------------------
    next_WP.alt = read_alt_to_hold();

    if (g.log_bitmask & MASK_LOG_MODE)
        Log_Write_Mode(control_mode);
}

static void do_takeoff()
{
    set_next_WP(&next_nav_command);
    // pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
    takeoff_pitch_cd                = (int)next_nav_command.p1 * 100;
    takeoff_altitude        = next_nav_command.alt;
    next_WP.lat             = home.lat + 1000;          // so we don't have bad calcs
    next_WP.lng             = home.lng + 1000;          // so we don't have bad calcs
    takeoff_complete        = false;                            // set flag to use gps ground course during TO.  IMU will be doing yaw drift correction
    // Flag also used to override "on the ground" throttle disable
}

static void do_nav_wp()
{
    set_next_WP(&next_nav_command);
}

static void do_land()
{
    set_next_WP(&next_nav_command);
}

static void do_loiter_unlimited()
{
    set_next_WP(&next_nav_command);
}

static void do_loiter_turns()
{
    set_next_WP(&next_nav_command);
    loiter_total = next_nav_command.p1 * 360;
}

static void do_loiter_time()
{
    set_next_WP(&next_nav_command);
    loiter_time_ms = millis();
    loiter_time_max_ms = next_nav_command.p1 * (uint32_t)1000;     // units are seconds
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
static bool verify_takeoff()
{
    if (ahrs.yaw_initialised()) {
        if (hold_course == -1) {
            // save our current course to take off
            hold_course = ahrs.yaw_sensor;
            gcs_send_text_fmt(PSTR("Holding course %ld"), hold_course);
        }
    }

    if (hold_course != -1) {
        // recalc bearing error with hold_course;
        nav_bearing_cd = hold_course;
        // recalc bearing error
        calc_bearing_error();
    }

    if (adjusted_altitude_cm() > takeoff_altitude)  {
        hold_course = -1;
        takeoff_complete = true;
        next_WP = current_loc;
        return true;
    } else {
        return false;
    }
}

// we are executing a landing
static bool verify_land()
{
    // we don't 'verify' landing in the sense that it never completes,
    // so we don't verify command completion. Instead we use this to
    // adjust final landing parameters

    // Set land_complete if we are within 2 seconds distance or within
    // 3 meters altitude of the landing point
    if (((wp_distance > 0) && (wp_distance <= (g.land_flare_sec*g_gps->ground_speed*0.01)))
        || (adjusted_altitude_cm() <= next_WP.alt + g.land_flare_alt*100)) {

        land_complete = true;

        if (hold_course == -1) {
            // we have just reached the threshold of to flare for landing.
            // We now don't want to do any radical
            // turns, as rolling could put the wings into the runway.
            // To prevent further turns we set hold_course to the
            // current heading. Previously we set this to
            // crosstrack_bearing, but the xtrack bearing can easily
            // be quite large at this point, and that could induce a
            // sudden large roll correction which is very nasty at
            // this point in the landing.
            hold_course = ahrs.yaw_sensor;
            gcs_send_text_fmt(PSTR("Land Complete - Hold course %ld"), hold_course);
        }

        // reload any airspeed or groundspeed parameters that may have
        // been set for landing
        g.airspeed_cruise_cm.load();
        g.min_gndspeed_cm.load();
        g.throttle_cruise.load();
    }

    if (hold_course != -1) {
        // recalc bearing error with hold_course;
        nav_bearing_cd = hold_course;
        // recalc bearing error
        calc_bearing_error();
    } else {
        update_crosstrack();
    }
    return false;
}

static bool verify_nav_wp()
{
    hold_course = -1;
    update_crosstrack();
    if ((wp_distance > 0) && (wp_distance <= g.waypoint_radius)) {
        gcs_send_text_fmt(PSTR("Reached Waypoint #%i dist %um"),
                          (unsigned)nav_command_index,
                          (unsigned)get_distance(&current_loc, &next_WP));
        return true;
    }

    // have we circled around the waypoint?
    if (loiter_sum > 300) {
        gcs_send_text_P(SEVERITY_MEDIUM,PSTR("Missed WP"));
        return true;
    }

    // have we flown past the waypoint?
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_fmt(PSTR("Passed Waypoint #%i dist %um"),
                          (unsigned)nav_command_index,
                          (unsigned)get_distance(&current_loc, &next_WP));
        return true;
    }

    return false;
}

static bool verify_loiter_unlim()
{
    update_loiter();
    calc_bearing_error();
    return false;
}

static bool verify_loiter_time()
{
    update_loiter();
    calc_bearing_error();
    if ((millis() - loiter_time_ms) > loiter_time_max_ms) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("verify_nav: LOITER time complete"));
        return true;
    }
    return false;
}

static bool verify_loiter_turns()
{
    update_loiter();
    calc_bearing_error();
    if(loiter_sum > loiter_total) {
        loiter_total = 0;
        gcs_send_text_P(SEVERITY_LOW,PSTR("verify_nav: LOITER orbits complete"));
        // clear the command queue;
        return true;
    }
    return false;
}

static bool verify_RTL()
{
    if (wp_distance <= g.waypoint_radius) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Reached home"));
        return true;
    }else{
        return false;
    }
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

static void do_wait_delay()
{
    condition_start = millis();
    condition_value  = next_nonnav_command.lat * 1000;          // convert to milliseconds
}

static void do_change_alt()
{
    condition_rate          = labs((int)next_nonnav_command.lat);
    condition_value         = next_nonnav_command.alt;
    if (condition_value < adjusted_altitude_cm()) {
        condition_rate = -condition_rate;
    }
    target_altitude_cm      = adjusted_altitude_cm() + (condition_rate / 10);                  // Divide by ten for 10Hz update
    next_WP.alt             = condition_value;                                                                  // For future nav calculations
    offset_altitude_cm      = 0;                                                                                        // For future nav calculations
}

static void do_within_distance()
{
    condition_value  = next_nonnav_command.lat;
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
    target_altitude_cm += condition_rate / 10;
    return false;
}

static bool verify_within_distance()
{
    if (wp_distance < condition_value) {
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
    next_WP = current_loc;
}

static void do_jump()
{
    if (next_nonnav_command.lat == 0) {
        // the jump counter has reached zero - ignore
        gcs_send_text_fmt(PSTR("Jumps left: 0 - skipping"));
        return;
    }
    if (next_nonnav_command.p1 >= g.command_total) {
        gcs_send_text_fmt(PSTR("Skipping invalid jump to %i"), next_nonnav_command.p1);
        return;        
    }

    struct Location temp;
    temp = get_cmd_with_index(g.command_index);

    gcs_send_text_fmt(PSTR("Jump to WP %u. Jumps left: %d"),
                      (unsigned)next_nonnav_command.p1,
                      (int)next_nonnav_command.lat);
    if (next_nonnav_command.lat > 0) {
        // Decrement repeat counter
        temp.lat                        = next_nonnav_command.lat - 1;                                          
        set_cmd_with_index(temp, g.command_index);
    }

    nav_command_ID          = NO_COMMAND;
    next_nav_command.id     = NO_COMMAND;
    non_nav_command_ID      = NO_COMMAND;

    gcs_send_text_fmt(PSTR("setting command index: %i"), next_nonnav_command.p1);
    g.command_index.set_and_save(next_nonnav_command.p1);
    nav_command_index       = next_nonnav_command.p1;
    // Need to back "next_WP" up as it was set to the next waypoint following the jump
    next_WP = prev_WP;

    temp = get_cmd_with_index(g.command_index);

    next_nav_command = temp;
    nav_command_ID = next_nav_command.id;
    non_nav_command_index = g.command_index;
    non_nav_command_ID = WAIT_COMMAND;

    if (g.log_bitmask & MASK_LOG_CMD) {
        Log_Write_Cmd(g.command_index, &next_nav_command);
    }
    handle_process_nav_cmd();
}

static void do_change_speed()
{
    switch (next_nonnav_command.p1)
    {
    case 0:             // Airspeed
        if (next_nonnav_command.alt > 0) {
            g.airspeed_cruise_cm.set(next_nonnav_command.alt * 100);
            gcs_send_text_fmt(PSTR("Set airspeed %u m/s"), (unsigned)next_nonnav_command.alt);
        }
        break;
    case 1:             // Ground speed
        gcs_send_text_fmt(PSTR("Set groundspeed %u"), (unsigned)next_nonnav_command.alt);
        g.min_gndspeed_cm.set(next_nonnav_command.alt * 100);
        break;
    }

    if (next_nonnav_command.lat > 0) {
        gcs_send_text_fmt(PSTR("Set throttle %u"), (unsigned)next_nonnav_command.lat);
        g.throttle_cruise.set(next_nonnav_command.lat);
    }
}

static void do_set_home()
{
    if (next_nonnav_command.p1 == 1 && g_gps->status() == GPS::GPS_OK) {
        init_home();
    } else {
        home.id         = MAV_CMD_NAV_WAYPOINT;
        home.lng        = next_nonnav_command.lng;                                      // Lon * 10**7
        home.lat        = next_nonnav_command.lat;                                      // Lat * 10**7
        home.alt        = max(next_nonnav_command.alt, 0);
        home_is_set = true;
    }
}

static void do_set_servo()
{
    APM_RC.enable_out(next_nonnav_command.p1 - 1);
    APM_RC.OutputCh(next_nonnav_command.p1 - 1, next_nonnav_command.alt);
}

static void do_set_relay()
{
    if (next_nonnav_command.p1 == 1) {
        relay.on();
    } else if (next_nonnav_command.p1 == 0) {
        relay.off();
    }else{
        relay.toggle();
    }
}

static void do_repeat_servo(uint8_t channel, uint16_t servo_value,
                            int16_t repeat, uint8_t delay_time)
{
    extern RC_Channel *rc_ch[NUM_CHANNELS];
    channel = channel - 1;
    if (channel < 5 || channel > 8) {
        // not allowed
        return;
    }
    event_state.rc_channel = channel;
    event_state.type = EVENT_TYPE_SERVO;

    event_state.start_time_ms  = 0;
    event_state.delay_ms    = delay_time * 500UL;
    event_state.repeat      = repeat * 2;
    event_state.servo_value = servo_value;
    event_state.undo_value  = rc_ch[channel]->radio_trim;
    update_events();
}

static void do_repeat_relay()
{
    event_state.type = EVENT_TYPE_RELAY;
    event_state.start_time_ms  = 0;
    // /2 (half cycle time) * 1000 (convert to milliseconds)
    event_state.delay_ms        = next_nonnav_command.lat * 500.0;
    event_state.repeat          = next_nonnav_command.alt * 2;
    update_events();
}
