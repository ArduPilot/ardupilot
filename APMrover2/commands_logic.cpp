#include "Rover.h"

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
bool Rover::start_command(const AP_Mission::Mission_Command& cmd)
{
    // log when new commands start
    if (should_log(MASK_LOG_CMD)) {
        DataFlash.Log_Write_Mission_Cmd(mission, cmd);
    }

    // exit immediately if not in AUTO mode
    if (control_mode != AUTO) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Executing command ID #%i", cmd.id);

    // remember the course of our next navigation leg
    next_navigation_leg_cd = mission.get_next_ground_course_cd(0);

    switch (cmd.id) {
    case MAV_CMD_NAV_WAYPOINT:  // Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        do_RTL();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        do_loiter_time(cmd);
        break;

    // Conditional commands
    case MAV_CMD_CONDITION_DELAY:
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        do_within_distance(cmd);
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw(cmd);
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
            camera_mount.set_roi_target(cmd.content.location);
        }
        break;
#endif

    case MAV_CMD_DO_SET_REVERSE:
        do_set_reverse(cmd);
        break;

    default:
        // return false for unhandled commands
        return false;
    }

    // if we got this far we must have been successful
    return true;
}

// exit_mission - callback function called from ap-mission when the mission has completed
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
void Rover::exit_mission()
{
    if (control_mode == AUTO) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "No commands. Can't set AUTO. Setting HOLD");
        set_mode(HOLD);
    }
}

// verify_command_callback - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool Rover::verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    if (control_mode == AUTO) {
        const bool cmd_complete = verify_command(cmd);

        // send message to GCS
        if (cmd_complete) {
            gcs_send_mission_item_reached_message(cmd.index);
        }

        return cmd_complete;
    }
    return false;
}

/*******************************************************************************
Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
Return true if we do not recognize the command so that we move on to the next command
*******************************************************************************/

bool Rover::verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.id) {
    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited(cmd);

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time(cmd);

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
    case MAV_CMD_DO_CONTROL_VIDEO:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_SET_REVERSE:
        return true;

    default:
        // error message
        gcs().send_text(MAV_SEVERITY_WARNING, "Skipping invalid cmd #%i", cmd.id);
        // return true if we do not recognize the command so that we move on to the next command
        return true;
    }
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

void Rover::do_RTL(void)
{
    prev_WP = current_loc;
    control_mode = RTL;
    next_WP = home;
}

void Rover::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // just starting so we haven't previously reached the waypoint
    previously_reached_wp = false;

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_start_time = 0;

    // this is the delay, stored in seconds
    loiter_duration = cmd.p1;

    // this is the distance we travel past the waypoint - not there yet so 0 initially
    distance_past_wp = 0;

    Location cmdloc = cmd.content.location;
    location_sanitize(current_loc, cmdloc);
    set_auto_WP(cmdloc);
}

void Rover::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    active_loiter = true;
    do_nav_wp(cmd);
    loiter_duration = 100;  // an arbitrary large loiter time
}

// do_loiter_time - initiate loitering at a point for a given time period
// if the vehicle is moved off the loiter point (i.e. a boat in a current)
// then the vehicle will actively return to the loiter coords.
void Rover::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    active_loiter = true;
    do_nav_wp(cmd);
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
bool Rover::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // Have we reached the waypoint i.e. are we within waypoint_radius of the waypoint
    if ((wp_distance > 0) && (wp_distance <= g.waypoint_radius)) {
        // check if we are loitering at this waypoint - the message sent to the GCS is different
        if (loiter_duration > 0) {
            // Check if this is the first time we have reached the waypoint
            if (!previously_reached_wp) {
                gcs().send_text(MAV_SEVERITY_INFO, "Reached waypoint #%u. Loiter for %u seconds",
                        static_cast<uint32_t>(cmd.index),
                        static_cast<uint32_t>(loiter_duration));
                // record the current time i.e. start timer
                loiter_start_time = millis();
                previously_reached_wp = true;
            }

            distance_past_wp = wp_distance;

            // Check if we have loiter long enough
            if (((millis() - loiter_start_time) / 1000) < loiter_duration) {
                return false;
            }
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Reached waypoint #%u. Distance %dm",
                    static_cast<uint32_t>(cmd.index),
                    static_cast<int32_t>(fabsf(get_distance(current_loc, next_WP))));
        }
        // set loiter_duration to 0 so we know we aren't or have finished loitering
        loiter_duration = 0;
        return true;
    }

    // have we gone past the waypoint?
    // We should always go through the waypoint i.e. the above code
    // first before we go past it but sometimes we don't.
    // OR have we reached the waypoint previously be we aren't actively loitering
    // This second check is required for when we roll past the waypoint radius
    if (location_passed_point(current_loc, prev_WP, next_WP) ||
        (!active_loiter && previously_reached_wp)) {
        // As we have passed the waypoint navigation needs to be done from current location
        prev_WP = current_loc;
        // Check if this is the first time we have reached the waypoint even though we have gone past it
        if (!previously_reached_wp) {
            gcs().send_text(MAV_SEVERITY_INFO, "Reached waypoint #%u. Loiter for %u seconds",
                    static_cast<uint32_t>(cmd.index),
                    static_cast<uint32_t>(loiter_duration));
            // record the current time i.e. start timer
            loiter_start_time = millis();
            previously_reached_wp = true;
            distance_past_wp = wp_distance;
        }

        // check if distance to the WP has changed and output new message if it has
        const float dist_to_wp = get_distance(current_loc, next_WP);
        if (!is_equal(distance_past_wp, dist_to_wp)) {
            distance_past_wp = dist_to_wp;
            gcs().send_text(MAV_SEVERITY_INFO, "Passed waypoint #%u. Distance %dm",
                    static_cast<uint32_t>(cmd.index),
                    static_cast<int32_t>(fabsf(distance_past_wp)));
        }

        // Check if we need to loiter at this waypoint
        if (loiter_duration > 0) {
            if (((millis() - loiter_start_time) / 1000) < loiter_duration) {
                return false;
            }
        }
        // set loiter_duration to 0 so we know we aren't or have finished loitering
        loiter_duration = 0;
        return true;
    }

    return false;
}

bool Rover::verify_RTL()
{
    if (wp_distance <= g.waypoint_radius) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached destination");
        rtl_complete = true;
        return true;
    }

    // have we gone past the waypoint?
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached destination. Distance away %dm",
                static_cast<int32_t>(fabsf(get_distance(current_loc, next_WP))));
        rtl_complete = true;
        return true;
    }

    return false;
}

bool Rover::verify_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    // Continually set loiter start time to now so it never finishes
    loiter_start_time += millis();
    verify_nav_wp(cmd);
    return false;
}

// verify_loiter_time - check if we have loitered long enough
bool Rover::verify_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    const bool result = verify_nav_wp(cmd);
    if (result) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Finished active loiter\n");
        // if we have finished active loitering - turn it off
        active_loiter = false;
    }
    return result;
}

void Rover::nav_set_yaw_speed()
{
    // if we haven't received a MAV_CMD_NAV_SET_YAW_SPEED message within the last 3 seconds bring the rover to a halt
    if ((millis() - guided_control.msg_time_ms) > 3000) {
        gcs().send_text(MAV_SEVERITY_WARNING, "NAV_SET_YAW_SPEED not recvd last 3secs, stopping");
        g2.motors.set_throttle(g.throttle_min.get());
        g2.motors.set_steering(0.0f);
        lateral_acceleration = 0.0f;
        return;
    }

    const int32_t steering = steerController.get_steering_out_angle_error(guided_control.turn_angle);
    g2.motors.set_steering(steering);

    // speed param in the message gives speed as a proportion of cruise speed.
    // 0.5 would set speed to the cruise speed
    // 1 is double the cruise speed.
    const float target_speed = g.speed_cruise * guided_control.target_speed * 2.0f;
    calc_throttle(target_speed);

    Log_Write_GuidedTarget(guided_mode, Vector3f(steering, 0.0f, 0.0f), Vector3f(target_speed, 0.0f, 0.0f));
}

void Rover::nav_set_speed()
{
    // if we haven't received a MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED message within the last 3 seconds bring the rover to a halt
    if ((millis() - guided_control.msg_time_ms) > 3000) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SET_VELOCITY not recvd last 3secs, stopping");
        g2.motors.set_throttle(g.throttle_min.get());
        g2.motors.set_steering(0.0f);
        lateral_acceleration = 0.0f;
        prev_WP = current_loc;
        next_WP = current_loc;
        set_guided_WP(current_loc);  // exit Guided_Velocity to prevent spam
        return;
    }
    prev_WP = current_loc;
    next_WP = current_loc;

    const int32_t steer_value = steerController.get_steering_out_rate(guided_control.target_steer_speed);
    location_update(next_WP, (steer_value + ahrs.yaw_sensor) * 0.01f, 4.0f);  // put the next wp at 4m forward at steer direction
    nav_controller->update_waypoint(current_loc, next_WP);

    g2.motors.set_steering(steer_value);
    calc_throttle(guided_control.target_speed);

    Log_Write_GuidedTarget(guided_mode, Vector3f(steer_value, 0.0f, 0.0f), Vector3f(guided_control.target_speed, 0.0f, 0.0f));
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

void Rover::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = static_cast<int32_t>(cmd.content.delay.seconds * 1000);  // convert seconds to milliseconds
}

void Rover::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value = cmd.content.distance.meters;
}

void Rover::do_yaw(const AP_Mission::Mission_Command& cmd)
{
    // Only support target yaw for now
    condition_start = condition_value;  // save condition_value from current navigation wp loaded
    // get final angle, 1 = Relative, 0 = Absolute
    if (cmd.content.yaw.relative_angle == 0) {
        // absolute angle
        condition_value = cmd.content.yaw.angle_deg * 100;
    } else {
        // relative angle
        condition_value = cmd.content.yaw.angle_deg * 100;
        if (cmd.content.yaw.direction < 0) {
            condition_value = -condition_value;
        }

        condition_value = condition_value + ahrs.yaw_sensor;
    }

    // absolute angle error
    const int32_t error_to_target_yaw = abs((condition_value - ahrs.yaw_sensor));

    // Calculate the steering to apply base on error calculated
    const int32_t steering = steerController.get_steering_out_angle_error(error_to_target_yaw);
    g2.motors.set_steering(steering);
    next_navigation_leg_cd = condition_value;
    calc_throttle(g.speed_cruise);

    do_auto_rotation = true;
}

bool Rover::do_yaw_rotation()
{
    // absolute angle error
    const int32_t error_to_target_yaw = abs(condition_value - ahrs.yaw_sensor);

    // check if we are within 5 degrees of the target heading
    if (error_to_target_yaw <= 500) {
        g2.motors.set_steering(0.0f);  // stop the current rotation
        condition_value = condition_start;  // reset the condition value to its previous value
        g2.motors.set_throttle(0.0f);
        next_navigation_leg_cd = mission.get_next_ground_course_cd(0);
        do_auto_rotation = false;
        return true;
    } else {
        // Calculate the steering to apply base on error calculated
        const int32_t steering = steerController.get_steering_out_angle_error(error_to_target_yaw);
        g2.motors.set_steering(steering);
        calc_throttle(g.speed_cruise);
        do_auto_rotation = true;
        return false;
    }
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

bool Rover::verify_wait_delay()
{
    if (static_cast<uint32_t>(millis() - condition_start) > static_cast<uint32_t>(condition_value)) {
        condition_value = 0;
        return true;
    }
    return false;
}

bool Rover::verify_within_distance()
{
    if (wp_distance < condition_value) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
bool Rover::verify_yaw()
{
    // override by do_yaw_rotation()
    if (do_auto_rotation) {
        return false;
    } else {
        return true;
    }
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

void Rover::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0.0f) {
        g.speed_cruise.set(cmd.content.speed.target_ms);
        gcs().send_text(MAV_SEVERITY_INFO, "Cruise speed: %.1f m/s", static_cast<double>(g.speed_cruise.get()));
    }

    if (cmd.content.speed.throttle_pct > 0.0f && cmd.content.speed.throttle_pct <= 100.0f) {
        g.throttle_cruise.set(cmd.content.speed.throttle_pct);
        gcs().send_text(MAV_SEVERITY_INFO, "Cruise throttle: %.1f", static_cast<double>(g.throttle_cruise.get()));
    }
}

void Rover::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 && have_position) {
        set_home_to_current_location(false);
    } else {
        set_home(cmd.content.location, false);
    }
}

#if CAMERA == ENABLED

// do_digicam_configure Send Digicam Configure message with the camera library
void Rover::do_digicam_configure(const AP_Mission::Mission_Command& cmd)
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
void Rover::do_digicam_control(const AP_Mission::Mission_Command& cmd)
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
void Rover::do_take_picture()
{
    camera.trigger_pic(true);
    log_picture();
}

// log_picture - log picture taken and send feedback to GCS
void Rover::log_picture()
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

void Rover::do_set_reverse(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1) {
        in_auto_reverse = true;
        set_reverse(true);
    } else {
        in_auto_reverse = false;
        set_reverse(false);
    }
}
