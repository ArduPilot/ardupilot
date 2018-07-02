#include "mode.h"
#include "Rover.h"

void ModeSimple::update()
{
    float desired_heading, desired_steering, desired_speed;

    // initial heading simple mode
    if (g2.simple_type == Simple_InitialHeading) {

        // get piot input
        get_pilot_desired_steering_and_speed(desired_steering, desired_speed);

        float simple_steering;
        if (desired_steering == 0) {
            simple_steering = ((rover.simple_sin_yaw - ahrs.sin_yaw()) * 4500.0f);
        } else {
            simple_steering = desired_steering;
        }

        // run throttle and steering controllers
        calc_throttle(desired_speed, true, false);
        g2.motors.set_steering(simple_steering, false);
    }

    // cardinal directions simple mode
    if (g2.simple_type == Simple_CardinalDirections) {

        // get pilot input
        get_pilot_desired_heading_and_speed(desired_heading, desired_speed);

        // call heading controller
        const float steering_out = attitude_control.get_steering_out_rate(radians(desired_heading * 0.01f),
                                                                          g2.motors.limit.steer_left,
                                                                          g2.motors.limit.steer_right,
                                                                          rover.G_Dt);
        // run throttle and steering controllers
        g2.motors.set_steering(steering_out * 4500.0f, false);
        calc_throttle(desired_speed, false, true);
    }
}
