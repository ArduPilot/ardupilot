#include "mode.h"
#include "Rover.h"

void ModeSimple::init_simple_heading()
{
    simple_initial_heading = ahrs.yaw;
}

void ModeSimple::update()
{
    float desired_heading, desired_steering, desired_speed;

    // initial heading simple mode
    if (g2.simple_type == Simple_InitialHeading) {

        // get piot input
        get_pilot_desired_steering_and_speed(desired_steering, desired_speed);

        float simple_heading;
        if (is_zero(desired_steering)) {
            simple_heading = ((simple_initial_heading - ahrs.yaw) * 4500.0f);
        } else {
            simple_heading = desired_steering;
        }

        // run throttle and steering controllers
        calc_steering_to_heading(simple_heading, false);
        calc_throttle(desired_speed, true, false);
    }

    // cardinal directions simple mode
    if (g2.simple_type == Simple_CardinalDirections) {

        // get pilot input
        get_pilot_desired_heading_and_speed(desired_heading, desired_speed);

        // run throttle and steering controllers
        calc_steering_to_heading(desired_heading, false);
        calc_throttle(desired_speed, false, true);
    }
}
