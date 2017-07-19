#include "Rover.h"

/*
    work out if we are going to use pivot steering
*/
bool Rover::use_pivot_steering(void)
{
    // check cases where we clearly cannot use pivot steering
    if (control_mode->is_autopilot_mode() || !g2.motors.have_skid_steering() || g.pivot_turn_angle <= 0) {
        pivot_steering_active = false;
        return false;
    }

    // calc bearing error
    const int16_t bearing_error = wrap_180_cd(nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100;

    // if error is larger than pivot_turn_angle start pivot steering
    if (bearing_error > g.pivot_turn_angle) {
        pivot_steering_active = true;
        return true;
    }

    // if within 10 degrees of the target heading, exit pivot steering
    if (bearing_error < 10) {
        pivot_steering_active = false;
        return false;
    }

    // by default stay in
    return pivot_steering_active;
}

/*
  test if we are loitering AND should be stopped at a waypoint
*/
bool Rover::in_stationary_loiter()
{
    // Confirm we are in AUTO mode and need to loiter for a time period
    if ((loiter_start_time > 0) && (control_mode == &mode_auto)) {
        // Check if active loiter is enabled AND we are outside the waypoint loiter radius
        // then the vehicle still needs to move so return false
        if (active_loiter && (wp_distance > g.waypoint_radius)) {
            return false;
        }
        return true;
    }

    return false;
}

/*****************************************
    Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void)
{
    // send output signals to motors
    if (motor_test) {
        g2.motors.slew_limit_throttle(false);
        motor_test_output();
    } else {
        g2.motors.output(arming.is_armed() && hal.util->get_soft_armed(), G_Dt);
    }
}
