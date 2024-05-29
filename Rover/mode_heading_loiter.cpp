#include "Rover.h"

bool ModeHeadingLoiter::_enter()
{
    // set _destination to reasonable stopping point
    if (!g2.wp_nav.get_stopping_location(_destination)) {
        return false;
    }

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;

    return true;
}

void ModeHeadingLoiter::update()
{
    // get distance (in meters) to destination
    _distance_to_destination = rover.current_loc.get_distance(_destination);

    const float loiter_radius = g2.loit_radius;

    // if within loiter radius slew desired speed towards zero and use existing desired heading
    if (_distance_to_destination <= loiter_radius) {
        const float desired_speed_within_radius = 0.0f;
        _desired_speed = attitude_control.get_desired_speed_accel_limited(desired_speed_within_radius, rover.G_Dt);
    } else {
        // P controller with hard-coded gain to convert distance to desired speed
        _desired_speed = MIN((_distance_to_destination - loiter_radius) * g2.loiter_speed_gain, g2.wp_nav.get_default_speed());
    }

    // 0 turn rate is no limit
    float turn_rate = 0.0;

    // run steering and throttle controllers
    calc_steering_to_heading(_desired_yaw_cd, turn_rate);
    calc_throttle(_desired_speed, true);
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
