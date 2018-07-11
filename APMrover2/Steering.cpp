#include "Rover.h"

/*
    work out if we are going to use pivot steering at next waypoint
*/
bool Rover::use_pivot_steering_at_next_WP(float yaw_error_cd)
{
    // check cases where we clearly cannot use pivot steering
    if (!g2.motors.have_skid_steering() || g.pivot_turn_angle <= 0) {
        return false;
    }

    // if error is larger than pivot_turn_angle then use pivot steering at next WP
    if (fabsf(yaw_error_cd) * 0.01f > g.pivot_turn_angle) {
        return true;
    }

    return false;
}

/*
    work out if we are going to use pivot steering
*/
bool Rover::use_pivot_steering(float yaw_error_cd)
{
    // check cases where we clearly cannot use pivot steering
    if (!g2.motors.have_skid_steering() || g.pivot_turn_angle <= 0) {
        pivot_steering_active = false;
        return false;
    }

    // calc bearing error
    const float yaw_error = fabsf(yaw_error_cd) * 0.01f;

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
        motor_test_output();
    } else {
        // get ground speed
        float speed;
        if (!g2.attitude_control.get_forward_speed(speed)) {
            speed = 0.0f;
        }

        g2.motors.output(arming.is_armed(), speed, G_Dt);
    }
}
