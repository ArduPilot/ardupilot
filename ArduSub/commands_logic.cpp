#include "Sub.h"

static enum AutoSurfaceState auto_surface_state = AUTO_SURFACE_STATE_GO_TO_LOCATION;

// start_command - this function will be called when the ap_mission lib wishes to start a new command
bool Sub::start_command(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: logging when new commands start/end
    if (should_log(MASK_LOG_CMD)) {
        DataFlash.Log_Write_Mission_Cmd(mission, cmd);
    }

    switch (cmd.id) {

        ///
        /// navigation commands
        ///
    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_surface(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        do_RTL();
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

    case MAV_CMD_NAV_SPLINE_WAYPOINT:           // 82  Navigate to Waypoint using spline
        do_spline_wp(cmd);
        break;

#if NAV_GUIDED == ENABLED
    case MAV_CMD_NAV_GUIDED_ENABLE:             // 92  accept navigation commands from external nav computer
        do_nav_guided_enable(cmd);
        break;
#endif

    case MAV_CMD_NAV_DELAY:                    // 94 Delay the next navigation command
        do_nav_delay(cmd);
        break;

        //
        // conditional commands
        //
    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance(cmd);
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
        // point the vehicle and camera at a region of interest (ROI)
        do_roi(cmd);
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:          // 205
        // point the camera to a specified angle
        do_mount_control(cmd);
        break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        do_digicam_configure(cmd);
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        do_digicam_control(cmd);
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        camera.set_trigger_distance(cmd.content.cam_trigg_dist.meters);
        break;
#endif

#if GRIPPER_ENABLED == ENABLED
    case MAV_CMD_DO_GRIPPER:                            // Mission command to control gripper
        do_gripper(cmd);
        break;
#endif

#if NAV_GUIDED == ENABLED
    case MAV_CMD_DO_GUIDED_LIMITS:                      // 220  accept guided mode limits
        do_guided_limits(cmd);
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

// check to see if current command goal has been acheived
// called by mission library in mission.update()
bool Sub::verify_command_callback(const AP_Mission::Mission_Command& cmd)
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


// check if current mission command has completed
bool Sub::verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.id) {
        //
        // navigation commands
        //
    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_LAND:
        return verify_surface(cmd);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_circle(cmd);

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();

    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        return verify_spline_wp(cmd);

#if NAV_GUIDED == ENABLED
    case MAV_CMD_NAV_GUIDED_ENABLE:
        return verify_nav_guided_enable(cmd);
#endif

    case MAV_CMD_NAV_DELAY:
        return verify_nav_delay(cmd);

        ///
        /// conditional commands
        ///
    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();

    case MAV_CMD_CONDITION_YAW:
        return verify_yaw();

        // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_SERVO:
    case MAV_CMD_DO_SET_RELAY:
    case MAV_CMD_DO_REPEAT_SERVO:
    case MAV_CMD_DO_REPEAT_RELAY:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_MOUNT_CONTROL:
    case MAV_CMD_DO_CONTROL_VIDEO:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_GRIPPER:
    case MAV_CMD_DO_GUIDED_LIMITS:
        return true;

    default:
        // error message
        gcs_send_text_fmt(MAV_SEVERITY_WARNING,"Skipping invalid cmd #%i",cmd.id);
        // return true if we do not recognize the command so that we move on to the next command
        return true;
    }
}

// exit_mission - function that is called once the mission completes
void Sub::exit_mission()
{
    // play a tone
    AP_Notify::events.mission_complete = 1;

    // Try to enter loiter, if that fails, go to depth hold
    if (!auto_loiter_start()) {
        set_mode(ALT_HOLD, MODE_REASON_MISSION_END);
    }
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

// do_nav_wp - initiate move to next waypoint
void Sub::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    Location_Class target_loc(cmd.content.location);
    // use current lat, lon if zero
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        target_loc.lat = current_loc.lat;
        target_loc.lng = current_loc.lng;
    }
    // use current altitude if not provided
    if (target_loc.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            target_loc.set_alt_cm(current_loc.alt, current_loc.get_alt_frame());
        }
    }

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // Set wp navigation target
    auto_wp_start(target_loc);

    // if no delay set the waypoint as "fast"
    if (loiter_time_max == 0) {
        wp_nav.set_fast_waypoint(true);
    }
}

// do_surface - initiate surface procedure
void Sub::do_surface(const AP_Mission::Mission_Command& cmd)
{
    Location_Class target_location;

    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to go to location
        auto_surface_state = AUTO_SURFACE_STATE_GO_TO_LOCATION;

        // calculate and set desired location below surface target
        // convert to location class
        target_location = Location_Class(cmd.content.location);

        // decide if we will use terrain following
        int32_t curr_terr_alt_cm, target_terr_alt_cm;
        if (current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, curr_terr_alt_cm) &&
                target_location.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, target_terr_alt_cm)) {
            // if using terrain, set target altitude to current altitude above terrain
            target_location.set_alt_cm(curr_terr_alt_cm, Location_Class::ALT_FRAME_ABOVE_TERRAIN);
        } else {
            // set target altitude to current altitude above home
            target_location.set_alt_cm(current_loc.alt, Location_Class::ALT_FRAME_ABOVE_HOME);
        }
    } else {
        // set surface state to ascend
        auto_surface_state = AUTO_SURFACE_STATE_ASCEND;

        // Set waypoint destination to current location at zero depth
        target_location = Location_Class(current_loc.lat, current_loc.lng, 0, Location_Class::ALT_FRAME_ABOVE_HOME);
    }

    // Go to wp location
    auto_wp_start(target_location);
}

void Sub::do_RTL()
{
    auto_wp_start(ahrs.get_home());
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
void Sub::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    // convert back to location
    Location_Class target_loc(cmd.content.location);

    // use current location if not provided
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        // To-Do: make this simpler
        Vector3f temp_pos;
        wp_nav.get_wp_stopping_point_xy(temp_pos);
        Location_Class temp_loc(temp_pos);
        target_loc.lat = temp_loc.lat;
        target_loc.lng = temp_loc.lng;
    }

    // In mavproxy misseditor: Abs = 0 = ALT_FRAME_ABSOLUTE
    //                         Rel = 1 = ALT_FRAME_ABOVE_HOME
    //                         AGL = 3 = ALT_FRAME_ABOVE_TERRAIN
    //    2 = ALT_FRAME_ABOVE_ORIGIN, not an option in mavproxy misseditor

    // use current altitude if not provided
    // To-Do: use z-axis stopping point instead of current alt
    if (target_loc.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            target_loc.set_alt_cm(current_loc.alt, current_loc.get_alt_frame());
        }
    }

    // start way point navigator and provide it the desired location
    auto_wp_start(target_loc);
}

// do_circle - initiate moving in a circle
void Sub::do_circle(const AP_Mission::Mission_Command& cmd)
{
    Location_Class circle_center(cmd.content.location);

    // default lat/lon to current position if not provided
    // To-Do: use stopping point or position_controller's target instead of current location to avoid jerk?
    if (circle_center.lat == 0 && circle_center.lng == 0) {
        circle_center.lat = current_loc.lat;
        circle_center.lng = current_loc.lng;
    }

    // default target altitude to current altitude if not provided
    if (circle_center.alt == 0) {
        int32_t curr_alt;
        if (current_loc.get_alt_cm(circle_center.get_alt_frame(),curr_alt)) {
            // circle altitude uses frame from command
            circle_center.set_alt_cm(curr_alt,circle_center.get_alt_frame());
        } else {
            // default to current altitude above origin
            circle_center.set_alt_cm(current_loc.alt, current_loc.get_alt_frame());
            Log_Write_Error(ERROR_SUBSYSTEM_TERRAIN, ERROR_CODE_MISSING_TERRAIN_DATA);
        }
    }

    // calculate radius
    uint8_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1

    // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
    auto_circle_movetoedge_start(circle_center, circle_radius_m);
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
void Sub::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}

// do_spline_wp - initiate move to next waypoint
void Sub::do_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    Location_Class target_loc(cmd.content.location);
    // use current lat, lon if zero
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        target_loc.lat = current_loc.lat;
        target_loc.lng = current_loc.lng;
    }
    // use current altitude if not provided
    if (target_loc.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            target_loc.set_alt_cm(current_loc.alt, current_loc.get_alt_frame());
        }
    }

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // determine segment start and end type
    bool stopped_at_start = true;
    AC_WPNav::spline_segment_end_type seg_end_type = AC_WPNav::SEGMENT_END_STOP;
    AP_Mission::Mission_Command temp_cmd;

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
    Location_Class next_loc;
    if (cmd.p1 == 0 && mission.get_next_nav_cmd(cmd.index+1, temp_cmd)) {
        next_loc = temp_cmd.content.location;
        // default lat, lon to first waypoint's lat, lon
        if (next_loc.lat == 0 && next_loc.lng == 0) {
            next_loc.lat = target_loc.lat;
            next_loc.lng = target_loc.lng;
        }
        // default alt to first waypoint's alt but in next waypoint's alt frame
        if (next_loc.alt == 0) {
            int32_t next_alt;
            if (target_loc.get_alt_cm(next_loc.get_alt_frame(), next_alt)) {
                next_loc.set_alt_cm(next_alt, next_loc.get_alt_frame());
            } else {
                // default to first waypoints altitude
                next_loc.set_alt_cm(target_loc.alt, target_loc.get_alt_frame());
            }
        }
        // if the next nav command is a waypoint set end type to spline or straight
        if (temp_cmd.id == MAV_CMD_NAV_WAYPOINT) {
            seg_end_type = AC_WPNav::SEGMENT_END_STRAIGHT;
        } else if (temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT) {
            seg_end_type = AC_WPNav::SEGMENT_END_SPLINE;
        }
    }

    // set spline navigation target
    auto_spline_start(target_loc, stopped_at_start, seg_end_type, next_loc);
}

#if NAV_GUIDED == ENABLED
// do_nav_guided_enable - initiate accepting commands from external nav computer
void Sub::do_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 > 0) {
        // initialise guided limits
        guided_limit_init_time_and_pos();

        // set spline navigation target
        auto_nav_guided_start();
    }
}
#endif  // NAV_GUIDED

// do_nav_delay - Delay the next navigation command
void Sub::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay_time_start = millis();

    if (cmd.content.nav_delay.seconds > 0) {
        // relative delay
        nav_delay_time_max = cmd.content.nav_delay.seconds * 1000; // convert seconds to milliseconds
    } else {
        // absolute delay to utc time
        nav_delay_time_max = hal.util->get_time_utc(cmd.content.nav_delay.hour_utc, cmd.content.nav_delay.min_utc, cmd.content.nav_delay.sec_utc, 0);
    }
    gcs_send_text_fmt(MAV_SEVERITY_INFO, "Delaying %u sec",(unsigned int)(nav_delay_time_max/1000));
}

#if GRIPPER_ENABLED == ENABLED
// do_gripper - control gripper
void Sub::do_gripper(const AP_Mission::Mission_Command& cmd)
{
    // Note: we ignore the gripper num parameter because we only support one gripper
    switch (cmd.content.gripper.action) {
    case GRIPPER_ACTION_RELEASE:
        g2.gripper.release();
        Log_Write_Event(DATA_GRIPPER_RELEASE);
        break;
    case GRIPPER_ACTION_GRAB:
        g2.gripper.grab();
        Log_Write_Event(DATA_GRIPPER_GRAB);
        break;
    default:
        // do nothing
        break;
    }
}
#endif

#if NAV_GUIDED == ENABLED
// do_guided_limits - pass guided limits to guided controller
void Sub::do_guided_limits(const AP_Mission::Mission_Command& cmd)
{
    guided_limit_set(cmd.p1 * 1000, // convert seconds to ms
                     cmd.content.guided_limits.alt_min * 100.0f,    // convert meters to cm
                     cmd.content.guided_limits.alt_max * 100.0f,    // convert meters to cm
                     cmd.content.guided_limits.horiz_max * 100.0f); // convert meters to cm
}
#endif

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/

// verify_nav_wp - check if we have reached the next way point
bool Sub::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if (!wp_nav.reached_wp_destination()) {
        return false;
    }

    // play a tone
    AP_Notify::events.waypoint_complete = 1;

    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    } else {
        return false;
    }
}

// verify_surface - returns true if surface procedure has been completed
bool Sub::verify_surface(const AP_Mission::Mission_Command& cmd)
{
    bool retval = false;

    switch (auto_surface_state) {
        case AUTO_SURFACE_STATE_GO_TO_LOCATION:
            // check if we've reached the location
            if (wp_nav.reached_wp_destination()) {
                // Set target to current xy and zero depth
                // TODO get xy target from current wp destination, because current location may be acceptance-radius away from original destination
                Location_Class target_location(cmd.content.location.lat, cmd.content.location.lng, 0, Location_Class::ALT_FRAME_ABOVE_HOME);

                auto_wp_start(target_location);

                // advance to next state
                auto_surface_state = AUTO_SURFACE_STATE_ASCEND;
            }
            break;

        case AUTO_SURFACE_STATE_ASCEND:
            if (wp_nav.reached_wp_destination()) {
                retval = true;
            }
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully surfaced
    return retval;
}

bool Sub::verify_RTL() {
    return wp_nav.reached_wp_destination();
}

bool Sub::verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
bool Sub::verify_loiter_time()
{
    // return immediately if we haven't reached our destination
    if (!wp_nav.reached_wp_destination()) {
        return false;
    }

    // start our loiter timer
    if (loiter_time == 0) {
        loiter_time = millis();
    }

    // check if loiter timer has run out
    return (((millis() - loiter_time) / 1000) >= loiter_time_max);
}

// verify_circle - check if we have circled the point enough
bool Sub::verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // check if we've reached the edge
    if (auto_mode == Auto_CircleMoveToEdge) {
        if (wp_nav.reached_wp_destination()) {
            Vector3f curr_pos = inertial_nav.get_position();
            Vector3f circle_center = pv_location_to_vector(cmd.content.location);

            // set target altitude if not provided
            if (is_zero(circle_center.z)) {
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
    return fabsf(circle_nav.get_angle_total()/M_2PI) >= LOWBYTE(cmd.p1);
}

// verify_spline_wp - check if we have reached the next way point using spline
bool Sub::verify_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if (!wp_nav.reached_wp_destination()) {
        return false;
    }

    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    } else {
        return false;
    }
}

#if NAV_GUIDED == ENABLED
// verify_nav_guided - check if we have breached any limits
bool Sub::verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    // if disabling guided mode then immediately return true so we move to next command
    if (cmd.p1 == 0) {
        return true;
    }

    // check time and position limits
    return guided_limit_check();
}
#endif  // NAV_GUIDED

// verify_nav_delay - check if we have waited long enough
bool Sub::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (millis() - nav_delay_time_start > (uint32_t)MAX(nav_delay_time_max,0)) {
        nav_delay_time_max = 0;
        return true;
    }
    return false;
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

void Sub::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = cmd.content.delay.seconds * 1000;     // convert seconds to milliseconds
}

void Sub::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters * 100;
}

void Sub::do_yaw(const AP_Mission::Mission_Command& cmd)
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

bool Sub::verify_wait_delay()
{
    if (millis() - condition_start > (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

bool Sub::verify_within_distance()
{
    if (wp_nav.get_wp_distance_to_destination() < (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
bool Sub::verify_yaw()
{
    // set yaw mode if it has been changed (the waypoint controller often retakes control of yaw as it executes a new waypoint command)
    if (auto_yaw_mode != AUTO_YAW_LOOK_AT_HEADING) {
        set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
    }

    // check if we are within 2 degrees of the target heading
    if (labs(wrap_180_cd(ahrs.yaw_sensor-yaw_look_at_heading)) <= 200) {
        return true;
    } else {
        return false;
    }
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

// do_guided - start guided mode
bool Sub::do_guided(const AP_Mission::Mission_Command& cmd)
{
    // only process guided waypoint if we are in guided mode
    if (control_mode != GUIDED && !(control_mode == AUTO && auto_mode == Auto_NavGuided)) {
        return false;
    }

    // switch to handle different commands
    switch (cmd.id) {

    case MAV_CMD_NAV_WAYPOINT: {
        // set wp_nav's destination
        Location_Class dest(cmd.content.location);
        return guided_set_destination(dest);
        break;
    }

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

void Sub::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0) {
        wp_nav.set_speed_xy(cmd.content.speed.target_ms * 100.0f);
    }
}

void Sub::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 || (cmd.content.location.lat == 0 && cmd.content.location.lng == 0 && cmd.content.location.alt == 0)) {
        set_home_to_current_location();
    } else {
        if (!far_from_EKF_origin(cmd.content.location)) {
            set_home(cmd.content.location);
        }
    }
}

// do_roi - starts actions required by MAV_CMD_NAV_ROI
//          this involves either moving the camera to point at the ROI (region of interest)
//          and possibly rotating the vehicle to point at the ROI if our mount type does not support a yaw feature
//  TO-DO: add support for other features of MAV_CMD_DO_SET_ROI including pointing at a given waypoint
void Sub::do_roi(const AP_Mission::Mission_Command& cmd)
{
    set_auto_yaw_roi(cmd.content.location);
}

#if CAMERA == ENABLED
// do_digicam_configure Send Digicam Configure message with the camera library
void Sub::do_digicam_configure(const AP_Mission::Mission_Command& cmd)
{
    camera.configure(cmd.content.digicam_configure.shooting_mode,
                     cmd.content.digicam_configure.shutter_speed,
                     cmd.content.digicam_configure.aperture,
                     cmd.content.digicam_configure.ISO,
                     cmd.content.digicam_configure.exposure_type,
                     cmd.content.digicam_configure.cmd_id,
                     cmd.content.digicam_configure.engine_cutoff_time);
}

// do_digicam_control Send Digicam Control message with the camera library
void Sub::do_digicam_control(const AP_Mission::Mission_Command& cmd)
{
    if (camera.control(cmd.content.digicam_control.session,
                       cmd.content.digicam_control.zoom_pos,
                       cmd.content.digicam_control.zoom_step,
                       cmd.content.digicam_control.focus_lock,
                       cmd.content.digicam_control.shooting_cmd,
                       cmd.content.digicam_control.cmd_id)) {
        log_picture();
    }
}

// do_take_picture - take a picture with the camera library
void Sub::do_take_picture()
{
    camera.trigger_pic(true);
    log_picture();
}

// log_picture - log picture taken and send feedback to GCS
void Sub::log_picture()
{
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
}
#endif

// point the camera to a specified angle
void Sub::do_mount_control(const AP_Mission::Mission_Command& cmd)
{
#if MOUNT == ENABLED
    camera_mount.set_angle_targets(cmd.content.mount_control.roll, cmd.content.mount_control.pitch, cmd.content.mount_control.yaw);
#endif
}
