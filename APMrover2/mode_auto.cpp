#include "mode.h"
#include "Rover.h"

bool ModeAuto::_enter()
{
    // fail to enter auto if no mission commands
    if (mission.num_commands() == 0) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "No Mission. Can't set AUTO.");
        return false;
    }

    // init location target
    set_desired_location(rover.current_loc);

    // other initialisation
    auto_triggered = false;
    g2.motors.slew_limit_throttle(true);

    // initialise reversed to be false
    set_reversed(false);

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
            _distance_to_destination = get_distance(rover.current_loc, _destination);
            // check if we've reached the destination
            if (!_reached_destination) {
                if (_distance_to_destination <= rover.g.waypoint_radius || location_passed_point(rover.current_loc, _origin, _destination)) {
                    // trigger reached
                    _reached_destination = true;
                }
            }
            // stay active at destination if caller requested this behaviour and outside the waypoint radius
            bool active_at_destination = _reached_destination && _stay_active_at_dest && (_distance_to_destination > rover.g.waypoint_radius);
            if (!_reached_destination || active_at_destination) {
                // continue driving towards destination
                calc_lateral_acceleration(active_at_destination ? rover.current_loc : _origin, _destination, _reversed);
                calc_nav_steer(_reversed);
                calc_throttle(calc_reduced_speed_for_turn_or_distance(_reversed ? -_desired_speed : _desired_speed), true);
            } else {
                // we have reached the destination so stop
                stop_vehicle();
                lateral_acceleration = 0.0f;
            }
            break;
        }

        case Auto_HeadingAndSpeed:
        {
            if (!_reached_heading) {
                // run steering and throttle controllers
                const float yaw_error = wrap_PI(radians((_desired_yaw_cd - ahrs.yaw_sensor) * 0.01f));
                const float steering_out = attitude_control.get_steering_out_angle_error(yaw_error, g2.motors.have_skid_steering(), g2.motors.limit.steer_left, g2.motors.limit.steer_right);
                g2.motors.set_steering(steering_out * 4500.0f);
                calc_throttle(_desired_speed, true);
                // check if we have reached target
                _reached_heading = (fabsf(yaw_error) < radians(5));
            } else {
                stop_vehicle();
            }
            break;
        }
    }
}

// set desired location to drive to
void ModeAuto::set_desired_location(const struct Location& destination, float next_leg_bearing_cd, bool stay_active_at_dest)
{
    // call parent
    Mode::set_desired_location(destination, next_leg_bearing_cd);

    _submode = Auto_WP;
    _stay_active_at_dest = stay_active_at_dest;
}

// return true if vehicle has reached or even passed destination
bool ModeAuto::reached_destination()
{
    if (_submode == Auto_WP) {
        return _reached_destination;
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

// execute the mission in reverse (i.e. backing up)
void ModeAuto::set_reversed(bool value)
{
    if (_reversed != value) {
        _reversed = value;
        rover.set_reverse(_reversed);
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

void ModeAuto::calc_throttle(float target_speed, bool nudge_allowed)
{
    // If not autostarting set the throttle to minimum
    if (!check_trigger()) {
        stop_vehicle();
        return;
    }
    Mode::calc_throttle(target_speed, nudge_allowed);
}
