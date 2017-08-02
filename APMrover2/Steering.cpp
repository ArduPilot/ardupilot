#include "Rover.h"

/*
    work out if we are going to use pivot steering
*/
bool Rover::use_pivot_steering(float yaw_error_cd)
{
    // check cases where we clearly cannot use pivot steering
    if (control_mode->is_autopilot_mode() || !g2.motors.have_skid_steering() || g.pivot_turn_angle <= 0) {
        pivot_steering_active = false;
        return false;
    }

    // calc bearing error
    const float yaw_error = yaw_error_cd / 100.0f;

    // if error is larger than pivot_turn_angle start pivot steering
    if (yaw_error > g.pivot_turn_angle) {
        pivot_steering_active = true;
        return true;
    }

    // if within 10 degrees of the target heading, exit pivot steering
    if (yaw_error < 10.0f) {
        pivot_steering_active = false;
        return false;
    }

    // by default stay in
    return pivot_steering_active;
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
