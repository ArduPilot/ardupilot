#include "mode.h"
#include "Rover.h"

#define AUTO_GUIDED_SEND_TARGET_MS 1000

bool ModeAuto::_enter()
{
    // fail to enter auto if no mission commands
    if (mission.num_commands() <= 1) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "No Mission. Can't set AUTO.");
        return false;
    }

    // init location target
    if (!g2.wp_nav.set_desired_location(rover.current_loc)) {
        return false;
    }

    // initialise waypoint speed
    g2.wp_nav.set_desired_speed_to_default();

    // other initialisation
    auto_triggered = false;

    // clear guided limits
    rover.mode_guided.limit_clear();

    // restart mission processing
    mission.start_or_resume();
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
    switch (_submode) {
        case Auto_WP:
        {
            if (!g2.wp_nav.reached_destination()) {
                // update navigation controller
                navigate_to_waypoint();
            } else {
                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
                // update distance to destination
                _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
            }
            break;
        }

        case Auto_HeadingAndSpeed:
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

        case Auto_RTL:
            rover.mode_rtl.update();
            break;

        case Auto_Loiter:
            rover.mode_loiter.update();
            break;

        case Auto_Guided:
        {
            // send location target to offboard navigation system
            send_guided_position_target();
            rover.mode_guided.update();
            break;
        }

        case Auto_Stop:
            stop_vehicle();
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

// return distance (in meters) to destination
float ModeAuto::get_distance_to_destination() const
{
    switch (_submode) {
    case Auto_WP:
        return _distance_to_destination;
    case Auto_HeadingAndSpeed:
    case Auto_Stop:
        // no valid distance so return zero
        return 0.0f;
    case Auto_RTL:
        return rover.mode_rtl.get_distance_to_destination();
    case Auto_Loiter:
        return rover.mode_loiter.get_distance_to_destination();
    case Auto_Guided:
        return rover.mode_guided.get_distance_to_destination();
    }

    // this line should never be reached
    return 0.0f;
}

// get desired location
bool ModeAuto::get_desired_location(Location& destination) const
{
    switch (_submode) {
    case Auto_WP:
        if (g2.wp_nav.is_destination_valid()) {
            destination = g2.wp_nav.get_oa_destination();
            return true;
        }
        return false;
    case Auto_HeadingAndSpeed:
    case Auto_Stop:
        // no desired location for this submode
        return false;
    case Auto_RTL:
        return rover.mode_rtl.get_desired_location(destination);
    case Auto_Loiter:
        return rover.mode_loiter.get_desired_location(destination);
    case Auto_Guided:
        return rover.mode_guided.get_desired_location(destination);\
    }

    // we should never reach here but just in case
    return false;
}

// set desired location to drive to
bool ModeAuto::set_desired_location(const struct Location& destination, float next_leg_bearing_cd)
{
    // call parent
    if (!Mode::set_desired_location(destination, next_leg_bearing_cd)) {
        return false;
    }

    _submode = Auto_WP;

    return true;
}

// return true if vehicle has reached or even passed destination
bool ModeAuto::reached_destination() const
{
    switch (_submode) {
    case Auto_WP:
        return g2.wp_nav.reached_destination();
        break;
    case Auto_HeadingAndSpeed:
    case Auto_Stop:
        // always return true because this is the safer option to allow missions to continue
        return true;
        break;
    case Auto_RTL:
        return rover.mode_rtl.reached_destination();
        break;
    case Auto_Loiter:
        return rover.mode_loiter.reached_destination();
        break;
    case Auto_Guided:
        return rover.mode_guided.reached_destination();
        break;
    }

    // we should never reach here but just in case, return true to allow missions to continue
    return true;
}

// set desired heading in centidegrees (vehicle will turn to this heading)
void ModeAuto::set_desired_heading_and_speed(float yaw_angle_cd, float target_speed)
{
    // call parent
    Mode::set_desired_heading_and_speed(yaw_angle_cd, target_speed);

    _submode = Auto_HeadingAndSpeed;
    _reached_heading = false;
}

// return true if vehicle has reached desired heading
bool ModeAuto::reached_heading()
{
    if (_submode == Auto_HeadingAndSpeed) {
        return _reached_heading;
    }
    // we should never reach here but just in case, return true to allow missions to continue
    return true;
}

// start RTL (within auto)
void ModeAuto::start_RTL()
{
    if (rover.mode_rtl.enter()) {
        _submode = Auto_RTL;
    }
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
        _submode = Auto_Loiter;
        return true;
    }
    return false;
}

// hand over control to external navigation controller in AUTO mode
void ModeAuto::start_guided(const Location& loc)
{
    if (rover.mode_guided.enter()) {
        _submode = Auto_Guided;

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
    _submode = Auto_Stop;
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
    // log when new commands start
    if (rover.should_log(MASK_LOG_CMD)) {
        rover.logger.Write_Mission_Cmd(mission, cmd);
    }

    switch (cmd.id) {
    case MAV_CMD_NAV_WAYPOINT:  // Navigate to Waypoint
        return do_nav_wp(cmd, false);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        do_RTL();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:  // Loiter indefinitely
    case MAV_CMD_NAV_LOITER_TIME:   // Loiter for specified time
        return do_nav_wp(cmd, true);

    case MAV_CMD_NAV_GUIDED_ENABLE: // accept navigation commands from external nav computer
        do_nav_guided_enable(cmd);
        break;

    case MAV_CMD_NAV_SET_YAW_SPEED:
        do_nav_set_yaw_speed(cmd);
        break;

    case MAV_CMD_NAV_DELAY:                    // 93 Delay the next navigation command
        do_nav_delay(cmd);
        break;

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

#if MOUNT == ENABLED
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
        if (cmd.p1 == 0) {  //disable
            g2.fence.enable(false);
            gcs().send_text(MAV_SEVERITY_INFO, "Fence Disabled");
        } else {  //enable fence
            g2.fence.enable(true);
            gcs().send_text(MAV_SEVERITY_INFO, "Fence Enabled");
        }
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

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time(cmd);

    case MAV_CMD_NAV_GUIDED_ENABLE:
        return verify_nav_guided_enable(cmd);

    case MAV_CMD_NAV_DELAY:
       return verify_nav_delay(cmd);

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();

    case MAV_CMD_NAV_SET_YAW_SPEED:
        return verify_nav_set_yaw_speed();

    // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_CONTROL_VIDEO:
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
    // get heading to following waypoint (auto mode reduces speed to allow corning without large overshoot)
    // in case of non-zero loiter duration, we provide heading-unknown to signal we should stop at the point
    float next_leg_bearing_cd = AR_WPNAV_HEADING_UNKNOWN;
    if (!always_stop_at_destination && loiter_duration == 0) {
        next_leg_bearing_cd = mission.get_next_ground_course_cd(AR_WPNAV_HEADING_UNKNOWN);
    }

    // retrieve and sanitize target location
    Location cmdloc = cmd.content.location;
    cmdloc.sanitize(rover.current_loc);
    if (!set_desired_location(cmdloc, next_leg_bearing_cd)) {
        return false;
    }

    // just starting so we haven't previously reached the waypoint
    previously_reached_wp = false;

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_start_time = 0;

    // this is the delay, stored in seconds
    loiter_duration = cmd.p1;

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
        nav_delay_time_max_ms = AP::rtc().get_time_utc(cmd.content.nav_delay.hour_utc, cmd.content.nav_delay.min_utc, cmd.content.nav_delay.sec_utc, 0);
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

// do_set_yaw_speed - turn to a specified heading and achieve and given speed
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

    // set auto target
    const float speed_max = g2.wp_nav.get_default_speed();
    set_desired_heading_and_speed(desired_heading_cd, constrain_float(cmd.content.set_yaw_speed.speed, -speed_max, speed_max));
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

bool ModeAuto::verify_RTL()
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
    if (_submode != Auto_Guided || cmd.p1 == 0) {
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
    return reached_heading();
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
