#include "Rover.h"

#define AUTO_GUIDED_SEND_TARGET_MS 1000

bool ModeAuto::_enter()
{
    // fail to enter auto if no mission commands
    if (mission.num_commands() <= 1) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "No Mission. Can't set AUTO.");
        return false;
    }

    // initialise waypoint navigation library
    g2.wp_nav.init();

    // other initialisation
    auto_triggered = false;

    // clear guided limits
    rover.mode_guided.limit_clear();

    // initialise submode to stop or loiter
    if (rover.is_boat()) {
        if (!start_loiter()) {
            start_stop();
        }
    } else {
        start_stop();
    }

    // set flag to start mission
    waiting_to_start = true;

    return true;
}

void ModeAuto::_exit()
{
    // stop running the mission
    if (mission.state() == AP_Mission::MISSION_RUNNING) {
        mission.stop();
    }
}

void ModeAuto::update()
{
    // check if mission exists (due to being cleared while disarmed in AUTO,
    // if no mission, then stop...needs mode change out of AUTO, mission load,
    // and change back to AUTO to run a mission at this point
    if (!hal.util->get_soft_armed() && mission.num_commands() <= 1) {
        start_stop();
    }
    // start or update mission
    if (waiting_to_start) {
        // don't start the mission until we have an origin
        Location loc;
        if (ahrs.get_origin(loc)) {
            // start/resume the mission (based on MIS_RESTART parameter)
            mission.start_or_resume();
            waiting_to_start = false;

            // initialise mission change check
            IGNORE_RETURN(mis_change_detector.check_for_mission_change());
        }
    } else {
        // check for mission changes
        if (mis_change_detector.check_for_mission_change()) {
            // if mission is running restart the current command if it is a waypoint command
            if ((mission.state() == AP_Mission::MISSION_RUNNING) && (_submode == SubMode::WP)) {
                if (mission.restart_current_nav_cmd()) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto mission changed, restarted command");
                } else {
                    // failed to restart mission for some reason
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto mission changed but failed to restart command");
                }
            }
        }

        mission.update();
    }

    switch (_submode) {
        case SubMode::WP:
        {
            // boats loiter once the waypoint is reached
            bool keep_navigating = true;
            if (rover.is_boat() && g2.wp_nav.reached_destination() && !g2.wp_nav.is_fast_waypoint()) {
                keep_navigating = !start_loiter();
            }

            // update navigation controller
            if (keep_navigating) {
                navigate_to_waypoint();
            }
            break;
        }

        case SubMode::HeadingAndSpeed:
        {
            if (!_reached_heading) {
                // run steering and throttle controllers
                calc_steering_to_heading(_desired_yaw_cd);
                calc_throttle(calc_speed_nudge(_desired_speed, is_negative(_desired_speed)), true);
                // check if we have reached within 5 degrees of target
                _reached_heading = (fabsf(_desired_yaw_cd - ahrs.yaw_sensor) < 500);
            } else {
                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
            }
            break;
        }

        case SubMode::RTL:
            rover.mode_rtl.update();
            break;

        case SubMode::Loiter:
            rover.mode_loiter.update();
            break;

        case SubMode::Guided:
        {
            // send location target to offboard navigation system
            send_guided_position_target();
            rover.mode_guided.update();
            break;
        }

        case SubMode::Stop:
            stop_vehicle();
            break;

        case SubMode::NavScriptTime:
            rover.mode_guided.update();
            break;

        case SubMode::Circle:
            rover.g2.mode_circle.update();
            break;
    }
}

void ModeAuto::calc_throttle(float target_speed, bool avoidance_enabled)
{
    // If not autostarting set the throttle to minimum
    if (!check_trigger()) {
        stop_vehicle();
        return;
    }
    Mode::calc_throttle(target_speed, avoidance_enabled);
}

// return heading (in degrees) to target destination (aka waypoint)
float ModeAuto::wp_bearing() const
{
    switch (_submode) {
    case SubMode::WP:
        return g2.wp_nav.wp_bearing_cd() * 0.01f;
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        return 0.0f;
    case SubMode::RTL:
        return rover.mode_rtl.wp_bearing();
    case SubMode::Loiter:
        return rover.mode_loiter.wp_bearing();
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.wp_bearing();
    case SubMode::Circle:
        return rover.g2.mode_circle.wp_bearing();
    }

    // this line should never be reached
    return 0.0f;
}

// return short-term target heading in degrees (i.e. target heading back to line between waypoints)
float ModeAuto::nav_bearing() const
{
    switch (_submode) {
    case SubMode::WP:
        return g2.wp_nav.nav_bearing_cd() * 0.01f;
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        return 0.0f;
    case SubMode::RTL:
        return rover.mode_rtl.nav_bearing();
    case SubMode::Loiter:
        return rover.mode_loiter.nav_bearing();
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.nav_bearing();
    case SubMode::Circle:
        return rover.g2.mode_circle.nav_bearing();
    }

    // this line should never be reached
    return 0.0f;
}

// return cross track error (i.e. vehicle's distance from the line between waypoints)
float ModeAuto::crosstrack_error() const
{
    switch (_submode) {
    case SubMode::WP:
        return g2.wp_nav.crosstrack_error();
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        return 0.0f;
    case SubMode::RTL:
        return rover.mode_rtl.crosstrack_error();
    case SubMode::Loiter:
        return rover.mode_loiter.crosstrack_error();
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.crosstrack_error();
    case SubMode::Circle:
        return rover.g2.mode_circle.crosstrack_error();
    }

    // this line should never be reached
    return 0.0f;
}

// return desired lateral acceleration
float ModeAuto::get_desired_lat_accel() const
{
    switch (_submode) {
    case SubMode::WP:
        return g2.wp_nav.get_lat_accel();
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        return 0.0f;
    case SubMode::RTL:
        return rover.mode_rtl.get_desired_lat_accel();
    case SubMode::Loiter:
        return rover.mode_loiter.get_desired_lat_accel();
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.get_desired_lat_accel();
    case SubMode::Circle:
        return rover.g2.mode_circle.get_desired_lat_accel();
    }

    // this line should never be reached
    return 0.0f;
}

// return distance (in meters) to destination
float ModeAuto::get_distance_to_destination() const
{
    switch (_submode) {
    case SubMode::WP:
        return _distance_to_destination;
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        // no valid distance so return zero
        return 0.0f;
    case SubMode::RTL:
        return rover.mode_rtl.get_distance_to_destination();
    case SubMode::Loiter:
        return rover.mode_loiter.get_distance_to_destination();
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.get_distance_to_destination();
    case SubMode::Circle:
        return rover.g2.mode_circle.get_distance_to_destination();
    }

    // this line should never be reached
    return 0.0f;
}

// get desired location
bool ModeAuto::get_desired_location(Location& destination) const
{
    switch (_submode) {
    case SubMode::WP:
        if (g2.wp_nav.is_destination_valid()) {
            destination = g2.wp_nav.get_oa_destination();
            return true;
        }
        return false;
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        // no desired location for this submode
        return false;
    case SubMode::RTL:
        return rover.mode_rtl.get_desired_location(destination);
    case SubMode::Loiter:
        return rover.mode_loiter.get_desired_location(destination);
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.get_desired_location(destination);
    case SubMode::Circle:
        return rover.g2.mode_circle.get_desired_location(destination);
    }

    // we should never reach here but just in case
    return false;
}

// set desired location to drive to
bool ModeAuto::set_desired_location(const Location &destination, Location next_destination)
{
    // call parent
    if (!Mode::set_desired_location(destination, next_destination)) {
        return false;
    }

    _submode = SubMode::WP;

    return true;
}

// return true if vehicle has reached or even passed destination
bool ModeAuto::reached_destination() const
{
    switch (_submode) {
    case SubMode::WP:
        return g2.wp_nav.reached_destination();
        break;
    case SubMode::HeadingAndSpeed:
    case SubMode::Stop:
        // always return true because this is the safer option to allow missions to continue
        return true;
        break;
    case SubMode::RTL:
        return rover.mode_rtl.reached_destination();
        break;
    case SubMode::Loiter:
        return rover.mode_loiter.reached_destination();
        break;
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.reached_destination();
    case SubMode::Circle:
        return rover.g2.mode_circle.reached_destination();
    }

    // we should never reach here but just in case, return true to allow missions to continue
    return true;
}

// set desired speed in m/s
bool ModeAuto::set_desired_speed(float speed)
{
    switch (_submode) {
    case SubMode::WP:
    case SubMode::Stop:
        return g2.wp_nav.set_speed_max(speed);
    case SubMode::HeadingAndSpeed:
        _desired_speed = speed;
        return true;
    case SubMode::RTL:
        return rover.mode_rtl.set_desired_speed(speed);
    case SubMode::Loiter:
        return rover.mode_loiter.set_desired_speed(speed);
    case SubMode::Guided:
    case SubMode::NavScriptTime:
        return rover.mode_guided.set_desired_speed(speed);
    case SubMode::Circle:
        return rover.g2.mode_circle.set_desired_speed(speed);
    }
    return false;
}

// start RTL (within auto)
void ModeAuto::start_RTL()
{
    if (rover.mode_rtl.enter()) {
        _submode = SubMode::RTL;
    }
}

// lua scripts use this to retrieve the contents of the active command
bool ModeAuto::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
#if AP_SCRIPTING_ENABLED
    if (_submode == SubMode::NavScriptTime) {
        id = nav_scripting.id;
        cmd = nav_scripting.command;
        arg1 = nav_scripting.arg1;
        arg2 = nav_scripting.arg2;
        arg3 = nav_scripting.arg3;
        arg4 = nav_scripting.arg4;
        return true;
    }
#endif
    return false;
}

// lua scripts use this to indicate when they have complete the command
void ModeAuto::nav_script_time_done(uint16_t id)
{
#if AP_SCRIPTING_ENABLED
    if ((_submode == SubMode::NavScriptTime) && (id == nav_scripting.id)) {
        nav_scripting.done = true;
    }
#endif
}

// check for triggering of start of auto mode
bool ModeAuto::check_trigger(void)
{
    // check for user pressing the auto trigger to off
    if (auto_triggered && g.auto_trigger_pin != -1 && rover.check_digital_pin(g.auto_trigger_pin) == 1) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AUTO triggered off");
        auto_triggered = false;
        return false;
    }

    // if already triggered, then return true, so you don't
    // need to hold the switch down
    if (auto_triggered) {
        return true;
    }

    // return true if auto trigger and kickstart are disabled
    if (g.auto_trigger_pin == -1 && is_zero(g.auto_kickstart)) {
        // no trigger configured - let's go!
        auto_triggered = true;
        return true;
    }

    // check if trigger pin has been pushed
    if (g.auto_trigger_pin != -1 && rover.check_digital_pin(g.auto_trigger_pin) == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Triggered AUTO with pin");
        auto_triggered = true;
        return true;
    }

    // check if mission is started by giving vehicle a kick with acceleration > AUTO_KICKSTART
    if (!is_zero(g.auto_kickstart)) {
        const float xaccel = rover.ins.get_accel().x;
        if (xaccel >= g.auto_kickstart) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Triggered AUTO xaccel=%.1f", static_cast<double>(xaccel));
            auto_triggered = true;
            return true;
        }
    }

    return false;
}

bool ModeAuto::start_loiter()
{
    if (rover.mode_loiter.enter()) {
        _submode = SubMode::Loiter;
        return true;
    }
    return false;
}

// hand over control to external navigation controller in AUTO mode
void ModeAuto::start_guided(const Location& loc)
{
    if (rover.mode_guided.enter()) {
        _submode = SubMode::Guided;

        // initialise guided start time and position as reference for limit checking
        rover.mode_guided.limit_init_time_and_location();

        // sanity check target location
        if ((loc.lat != 0) || (loc.lng != 0)) {
            guided_target.loc = loc;
            guided_target.loc.sanitize(rover.current_loc);
            guided_target.valid = true;
        } else {
            guided_target.valid = false;
        }
    }
}

// start stopping vehicle as quickly as possible
void ModeAuto::start_stop()
{
    _submode = SubMode::Stop;
}

// send latest position target to offboard navigation system
void ModeAuto::send_guided_position_target()
{
    if (!guided_target.valid) {
        return;
    }

    // send at maximum of 1hz
    const uint32_t now_ms = AP_HAL::millis();
    if ((guided_target.last_sent_ms == 0) || (now_ms - guided_target.last_sent_ms > AUTO_GUIDED_SEND_TARGET_MS)) {
        guided_target.last_sent_ms = now_ms;

        // get system id and component id of offboard navigation system
        uint8_t sysid;
        uint8_t compid;
        mavlink_channel_t chan;
        if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_ONBOARD_CONTROLLER, sysid, compid, chan)) {
            gcs().chan(chan-MAVLINK_COMM_0)->send_set_position_target_global_int(sysid, compid, guided_target.loc);
        }
    }

}

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
bool ModeAuto::start_command(const AP_Mission::Mission_Command& cmd)
{
#if HAL_LOGGING_ENABLED
    // log when new commands start
    if (rover.should_log(MASK_LOG_CMD)) {
        rover.logger.Write_Mission_Cmd(mission, cmd);
    }
#endif

    switch (cmd.id) {
    case MAV_CMD_NAV_WAYPOINT:  // Navigate to Waypoint
        return do_nav_wp(cmd, false);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        do_RTL();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:  // Loiter indefinitely
    case MAV_CMD_NAV_LOITER_TIME:   // Loiter for specified time
        return do_nav_wp(cmd, true);

    case MAV_CMD_NAV_LOITER_TURNS:
        return do_circle(cmd);

    case MAV_CMD_NAV_GUIDED_ENABLE: // accept navigation commands from external nav computer
        do_nav_guided_enable(cmd);
        break;

    case MAV_CMD_NAV_SET_YAW_SPEED:
        do_nav_set_yaw_speed(cmd);
        break;

    case MAV_CMD_NAV_DELAY:                    // 93 Delay the next navigation command
        do_nav_delay(cmd);
        break;

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        do_nav_script_time(cmd);
        break;
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

#if HAL_MOUNT_ENABLED
    // Sets the region of interest (ROI) for a sensor set or the
    // vehicle itself. This can then be used by the vehicles control
    // system to control the vehicle attitude and the attitude of various
    // devices such as cameras.
    //    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
    case MAV_CMD_DO_SET_ROI:
        if (cmd.content.location.alt == 0 && cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
            // switch off the camera tracking if enabled
            if (rover.camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
                rover.camera_mount.set_mode_to_default();
            }
        } else {
            // send the command to the camera mount
            rover.camera_mount.set_roi_target(cmd.content.location);
        }
        break;
#endif

    case MAV_CMD_DO_SET_REVERSE:
        do_set_reverse(cmd);
        break;

    case MAV_CMD_DO_FENCE_ENABLE:
#if AP_FENCE_ENABLED
        if (cmd.p1 == 0) {  //disable
            rover.fence.enable(false);
            gcs().send_text(MAV_SEVERITY_INFO, "Fence Disabled");
        } else {  //enable fence
            rover.fence.enable(true);
            gcs().send_text(MAV_SEVERITY_INFO, "Fence Enabled");
        }
#endif
        break;

    case MAV_CMD_DO_GUIDED_LIMITS:
        do_guided_limits(cmd);
        break;

    default:
        // return false for unhandled commands
        return false;
    }

    // if we got this far we must have been successful
    return true;
}

// exit_mission - callback function called from ap-mission when the mission has completed
void ModeAuto::exit_mission()
{
    // play a tone
    AP_Notify::events.mission_complete = 1;
    // send message
    gcs().send_text(MAV_SEVERITY_NOTICE, "Mission Complete");

    if (g2.mis_done_behave == MIS_DONE_BEHAVE_LOITER && start_loiter()) {
        return;
    }

    if (g2.mis_done_behave == MIS_DONE_BEHAVE_ACRO && rover.set_mode(rover.mode_acro, ModeReason::MISSION_END)) {
        return;
    }

    if (g2.mis_done_behave == MIS_DONE_BEHAVE_MANUAL && rover.set_mode(rover.mode_manual, ModeReason::MISSION_END)) {
        return;
    }

    start_stop();
}

// verify_command_callback - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool ModeAuto::verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    const bool cmd_complete = verify_command(cmd);

    // send message to GCS
    if (cmd_complete) {
        gcs().send_mission_item_reached_message(cmd.index);
    }

    return cmd_complete;
}

/*******************************************************************************
Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
Return true if we do not recognize the command so that we move on to the next command
*******************************************************************************/

bool ModeAuto::verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.id) {
    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited(cmd);

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_circle(cmd);

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time(cmd);

    case MAV_CMD_NAV_GUIDED_ENABLE:
        return verify_nav_guided_enable(cmd);

    case MAV_CMD_NAV_DELAY:
        return verify_nav_delay(cmd);

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        return verify_nav_script_time();
#endif

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();

    case MAV_CMD_NAV_SET_YAW_SPEED:
        return verify_nav_set_yaw_speed();

    // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_SET_REVERSE:
    case MAV_CMD_DO_FENCE_ENABLE:
    case MAV_CMD_DO_GUIDED_LIMITS:
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

void ModeAuto::do_RTL(void)
{
    // start rtl in auto mode
    start_RTL();
}

bool ModeAuto::do_nav_wp(const AP_Mission::Mission_Command& cmd, bool always_stop_at_destination)
{
    // retrieve and sanitize target location
    Location cmdloc = cmd.content.location;
    cmdloc.sanitize(rover.current_loc);

    // delayed stored in p1 in seconds
    loiter_duration = ((int16_t) cmd.p1 < 0) ? 0 : cmd.p1;
    loiter_start_time = 0;
    if (loiter_duration > 0) {
        always_stop_at_destination = true;
    }

    // do not add next wp if there are no more navigation commands
    AP_Mission::Mission_Command next_cmd;
    if (always_stop_at_destination || !mission.get_next_nav_cmd(cmd.index+1, next_cmd)) {
        // single destination
        if (!set_desired_location(cmdloc)) {
            return false;
        }
    } else {
        // retrieve and sanitize next destination location
        Location next_cmdloc = next_cmd.content.location;
        next_cmdloc.sanitize(cmdloc);
        if (!set_desired_location(cmdloc, next_cmdloc)) {
            return false;
        }
    }

    // just starting so we haven't previously reached the waypoint
    previously_reached_wp = false;

    return true;
}

// do_nav_delay - Delay the next navigation command
void ModeAuto::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay_time_start_ms = millis();

    // boats loiter, cars and balancebots stop
    if (rover.is_boat()) {
        if (!start_loiter()) {
            start_stop();
        }
    } else {
        start_stop();
    }

    if (cmd.content.nav_delay.seconds > 0) {
        // relative delay
        nav_delay_time_max_ms = cmd.content.nav_delay.seconds * 1000; // convert seconds to milliseconds
    } else {
        // absolute delay to utc time
#if AP_RTC_ENABLED
        nav_delay_time_max_ms = AP::rtc().get_time_utc(cmd.content.nav_delay.hour_utc, cmd.content.nav_delay.min_utc, cmd.content.nav_delay.sec_utc, 0);
#else
        nav_delay_time_max_ms = 0;
#endif
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Delaying %u sec", (unsigned)(nav_delay_time_max_ms/1000));
}

// start guided within auto to allow external navigation system to control vehicle
void ModeAuto::do_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 > 0) {
        start_guided(cmd.content.location);
    }
}

// do_set_yaw_speed - turn to a specified heading and achieve a given speed
void ModeAuto::do_nav_set_yaw_speed(const AP_Mission::Mission_Command& cmd)
{
    float desired_heading_cd;

    // get final angle, 1 = Relative, 0 = Absolute
    if (cmd.content.set_yaw_speed.relative_angle > 0) {
        // relative angle
        desired_heading_cd = wrap_180_cd(ahrs.yaw_sensor + cmd.content.set_yaw_speed.angle_deg * 100.0f);
    } else {
        // absolute angle
        desired_heading_cd = cmd.content.set_yaw_speed.angle_deg * 100.0f;
    }

    // set targets
    const float speed_max = g2.wp_nav.get_default_speed();
    _desired_speed = constrain_float(cmd.content.set_yaw_speed.speed, -speed_max, speed_max);
    _desired_yaw_cd = desired_heading_cd;
    _reached_heading = false;
    _submode = SubMode::HeadingAndSpeed;
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
bool ModeAuto::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // exit immediately if we haven't reached the destination
    if (!reached_destination()) {
        return false;
    }

    // Check if this is the first time we have noticed reaching the waypoint
    if (!previously_reached_wp) {
        previously_reached_wp = true;

        // check if we are loitering at this waypoint - the message sent to the GCS is different
        if (loiter_duration > 0) {
            // send message including loiter time
            gcs().send_text(MAV_SEVERITY_INFO, "Reached waypoint #%u. Loiter for %u seconds",
                            (unsigned int)cmd.index,
                            (unsigned int)loiter_duration);
            // record the current time i.e. start timer
            loiter_start_time = millis();
        } else {
            // send simpler message to GCS
            gcs().send_text(MAV_SEVERITY_INFO, "Reached waypoint #%u", (unsigned int)cmd.index);
        }
    }

    // Check if we have loitered long enough
    if (loiter_duration == 0) {
        return true;
    } else {
        return (((millis() - loiter_start_time) / 1000) >= loiter_duration);
    }
}

// verify_nav_delay - check if we have waited long enough
bool ModeAuto::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (millis() - nav_delay_time_start_ms > nav_delay_time_max_ms) {
        nav_delay_time_max_ms = 0;
        return true;
    }

    return false;
}

bool ModeAuto::verify_RTL() const
{
    return reached_destination();
}

bool ModeAuto::verify_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    verify_nav_wp(cmd);
    return false;
}

// verify_loiter_time - check if we have loitered long enough
bool ModeAuto::verify_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    const bool result = verify_nav_wp(cmd);
    if (result) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Finished active loiter");
    }
    return result;
}

// check if guided has completed
bool ModeAuto::verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    // if we failed to enter guided or this command disables guided
    // return true so we move to next command
    if (_submode != SubMode::Guided || cmd.p1 == 0) {
        return true;
    }

    // if a location target was set, return true once vehicle is close
    if (guided_target.valid) {
        if (rover.current_loc.get_distance(guided_target.loc) <= rover.g2.wp_nav.get_radius()) {
            return true;
        }
    }

    // guided command complete once a limit is breached
    return rover.mode_guided.limit_breached();
}

// verify_yaw - return true if we have reached the desired heading
bool ModeAuto::verify_nav_set_yaw_speed()
{
    if (_submode == SubMode::HeadingAndSpeed) {
        return _reached_heading;
    }
    // we should never reach here but just in case, return true to allow missions to continue
    return true;
}

bool ModeAuto::do_circle(const AP_Mission::Mission_Command& cmd)
{
    // retrieve and sanitize target location
    Location circle_center = cmd.content.location;
    circle_center.sanitize(rover.current_loc);

    // calculate radius
    uint16_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1
    if (cmd.id == MAV_CMD_NAV_LOITER_TURNS &&
        cmd.type_specific_bits & (1U << 0)) {
        // special storage handling allows for larger radii
        circle_radius_m *= 10;
    }

    // initialise circle mode
    if (g2.mode_circle.set_center(circle_center, circle_radius_m, cmd.content.location.loiter_ccw)) {
        _submode = SubMode::Circle;
        return true;
    }
    return false;
}

bool ModeAuto::verify_circle(const AP_Mission::Mission_Command& cmd)
{
    const float turns = cmd.get_loiter_turns();
    // check if we have completed circling
    return ((g2.mode_circle.get_angle_total_rad() / M_2PI) >= turns);
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

void ModeAuto::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = static_cast<int32_t>(cmd.content.delay.seconds * 1000);  // convert seconds to milliseconds
}

void ModeAuto::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value = cmd.content.distance.meters;
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

bool ModeAuto::verify_wait_delay()
{
    if (static_cast<uint32_t>(millis() - condition_start) > static_cast<uint32_t>(condition_value)) {
        condition_value = 0;
        return true;
    }
    return false;
}

bool ModeAuto::verify_within_distance()
{
    if (get_distance_to_destination() < condition_value) {
        condition_value = 0;
        return true;
    }
    return false;
}


/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

void ModeAuto::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    // set speed for active mode
    if (set_desired_speed(cmd.content.speed.target_ms)) {
        gcs().send_text(MAV_SEVERITY_INFO, "speed: %.1f m/s", static_cast<double>(cmd.content.speed.target_ms));
    }
}

void ModeAuto::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 && rover.have_position) {
        if (!rover.set_home_to_current_location(false)) {
            // ignored...
        }
    } else {
        if (!rover.set_home(cmd.content.location, false)) {
            // ignored...
        }
    }
}

void ModeAuto::do_set_reverse(const AP_Mission::Mission_Command& cmd)
{
    set_reversed(cmd.p1 == 1);
}

// set timeout and position limits for guided within auto
void ModeAuto::do_guided_limits(const AP_Mission::Mission_Command& cmd)
{
    rover.mode_guided.limit_set(
        cmd.p1 * 1000, // convert seconds to ms
        cmd.content.guided_limits.horiz_max);
}

#if AP_SCRIPTING_ENABLED
// start accepting position, velocity and acceleration targets from lua scripts
void ModeAuto::do_nav_script_time(const AP_Mission::Mission_Command& cmd)
{
    // call regular guided flight mode initialisation
    if (rover.mode_guided.enter()) {
        _submode = SubMode::NavScriptTime;
        nav_scripting.done = false;
        nav_scripting.id++;
        nav_scripting.start_ms = millis();
        nav_scripting.command = cmd.content.nav_script_time.command;
        nav_scripting.timeout_s = cmd.content.nav_script_time.timeout_s;
        nav_scripting.arg1 = cmd.content.nav_script_time.arg1.get();
        nav_scripting.arg2 = cmd.content.nav_script_time.arg2.get();
        nav_scripting.arg3 = cmd.content.nav_script_time.arg3;
        nav_scripting.arg4 = cmd.content.nav_script_time.arg4;
    } else {
        // for safety we set nav_scripting to done to protect against the mission getting stuck
        nav_scripting.done = true;
    }
}

// check if verify_nav_script_time command has completed
bool ModeAuto::verify_nav_script_time()
{
    // if done or timeout then return true
    if (nav_scripting.done ||
        ((nav_scripting.timeout_s > 0) &&
         (AP_HAL::millis() - nav_scripting.start_ms) > (nav_scripting.timeout_s * 1000))) {
        return true;
    }
    return false;
}
#endif
