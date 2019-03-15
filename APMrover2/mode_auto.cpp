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

    // initialise waypoint speed
    set_desired_speed_to_default();

    // init location target
    set_desired_location(rover.current_loc);

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
            _distance_to_destination = rover.current_loc.get_distance(_destination);
            const bool near_wp = _distance_to_destination <= rover.g.waypoint_radius;
            // check if we've reached the destination
            if (!_reached_destination && (near_wp || location_passed_point(rover.current_loc, _origin, _destination))) {
                // trigger reached
                _reached_destination = true;
            }
            // determine if we should keep navigating
            if (!_reached_destination) {
                // continue driving towards destination
                calc_steering_to_waypoint(_reached_destination ? rover.current_loc : _origin, _destination, _reversed);
                calc_throttle(calc_reduced_speed_for_turn_or_distance(_reversed ? -_desired_speed : _desired_speed), true, true);
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

        case Auto_HeadingAndSpeed:
        {
            if (!_reached_heading) {
                // run steering and throttle controllers
                calc_steering_to_heading(_desired_yaw_cd);
                calc_throttle(_desired_speed, true, true);
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
    }
}

// return distance (in meters) to destination
float ModeAuto::get_distance_to_destination() const
{
    if (_submode == Auto_RTL) {
        return rover.mode_rtl.get_distance_to_destination();
    }
    return _distance_to_destination;
}

// set desired location to drive to
void ModeAuto::set_desired_location(const struct Location& destination, float next_leg_bearing_cd)
{
    // call parent
    Mode::set_desired_location(destination, next_leg_bearing_cd);

    _submode = Auto_WP;
}

// return true if vehicle has reached or even passed destination
bool ModeAuto::reached_destination() const
{
    switch (_submode) {
    case Auto_WP:
        return _reached_destination;
        break;
    case Auto_HeadingAndSpeed:
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
            location_sanitize(rover.current_loc, guided_target.loc);
            guided_target.valid = true;
        } else {
            guided_target.valid = false;
        }
    }
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
            gcs().chan(chan-MAVLINK_COMM_0).send_set_position_target_global_int(sysid, compid, guided_target.loc);
        }
    }

}

/*
    check for triggering of start of auto mode
*/
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

void ModeAuto::calc_throttle(float target_speed, bool nudge_allowed, bool avoidance_enabled)
{
    // If not autostarting set the throttle to minimum
    if (!check_trigger()) {
        stop_vehicle();
        return;
    }
    Mode::calc_throttle(target_speed, nudge_allowed, avoidance_enabled);
}

bool ModeAuto::start_loiter()
{
    if (rover.mode_loiter.enter()) {
        _submode = Auto_Loiter;
        return true;
    }
    return false;
}
