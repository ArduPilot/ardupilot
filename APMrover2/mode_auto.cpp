#include "mode.h"
#include "Rover.h"

bool ModeAuto::_enter()
{
    // fail to enter auto if no mission commands
    if (mission.num_commands() == 0) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "No Mission. Can't set AUTO.");
        return false;
    }

    auto_triggered = false;
    rover.restart_nav();
    rover.loiter_start_time = 0;
    g2.motors.slew_limit_throttle(true);
    return true;
}

void ModeAuto::_exit()
{
    // If we are changing out of AUTO mode reset the loiter timer
    rover.loiter_start_time = 0;
    // ... and stop running the mission
    if (mission.state() == AP_Mission::MISSION_RUNNING) {
        mission.stop();
    }
}

void ModeAuto::update()
{
    if (!rover.in_auto_reverse) {
        rover.set_reverse(false);
    }
    if (!rover.do_auto_rotation) {
        calc_lateral_acceleration();
        calc_nav_steer();
        calc_throttle(g.speed_cruise);
    } else {
        rover.do_yaw_rotation();
    }
}

void ModeAuto::update_navigation()
{
    mission.update();
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

void ModeAuto::calc_throttle(float target_speed)
{
    // If not autostarting OR we are loitering at a waypoint
    // then set the throttle to minimum
    if (!check_trigger() || rover.in_stationary_loiter()) {
        g2.motors.set_throttle(g.throttle_min.get());
        // Stop rotation in case of loitering and skid steering
        if (g2.motors.have_skid_steering()) {
            g2.motors.set_steering(0.0f);
        }
        return;
    }
    Mode::calc_throttle(target_speed);
}

void ModeAuto::calc_lateral_acceleration()
{
    // If we have reached the waypoint previously navigate
    // back to it from our current position
    if (rover.previously_reached_wp && (rover.loiter_duration > 0)) {
        Mode::calc_lateral_acceleration(rover.current_loc, rover.next_WP);
    } else {
        Mode::calc_lateral_acceleration(rover.prev_WP, rover.next_WP);
    }
}

void ModeAuto::calc_nav_steer()
{
    // check to see if the rover is loitering
    if (rover.in_stationary_loiter()) {
        g2.motors.set_steering(0.0f);
        return;
    }
    Mode::calc_nav_steer();
}
