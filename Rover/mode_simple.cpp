#include "Rover.h"

void ModeSimple::init_heading()
{
    _initial_heading_cd = ahrs.yaw_sensor;
    _desired_heading_cd = ahrs.yaw_sensor;
}

void ModeSimple::update()
{
    float desired_heading_cd, desired_speed;

    // get pilot input
    get_pilot_desired_heading_and_speed(desired_heading_cd, desired_speed);

    // rotate heading around based on initial heading
    if (g2.simple_type == Simple_InitialHeading) {
        desired_heading_cd = wrap_360_cd(_initial_heading_cd + desired_heading_cd);
    }

    // if sticks in middle, use previous desired heading (important when vehicle is slowing down)
    if (!is_positive(desired_speed)) {
        desired_heading_cd = _desired_heading_cd;
    } else {
        // record desired heading for next iteration
        _desired_heading_cd = desired_heading_cd;
    }

    // run throttle and steering controllers
    calc_steering_to_heading(desired_heading_cd);
    calc_throttle(desired_speed, true);
}
