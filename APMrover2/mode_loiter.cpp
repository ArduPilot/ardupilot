#include "mode.h"
#include "Rover.h"

bool ModeLoiter::_enter()
{
    // set _destination to reasonable stopping point
    calc_stopping_location(_destination);

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading to current heading
    _desired_yaw_cd = ahrs.yaw_sensor;

    return true;
}

void ModeLoiter::update()
{
    // get distance (in meters) to destination
    _distance_to_destination = rover.current_loc.get_distance(_destination);

    // if within loiter radius slew desired speed towards zero and use existing desired heading
    if (_distance_to_destination <= g2.loit_radius) {
        // sailboats do not stop
        const float desired_speed_within_radius = rover.g2.sailboat.enabled() ? 0.1f : 0.0f;
        _desired_speed = attitude_control.get_desired_speed_accel_limited(desired_speed_within_radius, rover.G_Dt);
    } else {
        // P controller with hard-coded gain to convert distance to desired speed
        // To-Do: make gain configurable or calculate from attitude controller's maximum accelearation
        _desired_speed = MIN((_distance_to_destination - g2.loit_radius) * 0.5f, g2.wp_nav.get_default_speed());

        // calculate bearing to destination
        _desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
        float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
        // if destination is behind vehicle, reverse towards it
        if (fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) {
            _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
            yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
            _desired_speed = -_desired_speed;
        }

        // reduce desired speed if yaw_error is large
        // 45deg of error reduces speed to 75%, 90deg of error reduces speed to 50%
        float yaw_error_ratio = 1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
        _desired_speed *= yaw_error_ratio;
    }

    // run steering and throttle controllers
    calc_steering_to_heading(_desired_yaw_cd);
    calc_throttle(_desired_speed, true);
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}
