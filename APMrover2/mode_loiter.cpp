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
    _yaw_error_cd = 0.0f;

    // set reversed based on speed
    rover.set_reverse(is_negative(_desired_speed));

    return true;
}

void ModeLoiter::update()
{
    // get distance (in meters) to destination
    _distance_to_destination = get_distance(rover.current_loc, _destination);

    // if within waypoint radius slew desired speed towards zero and use existing desired heading
    if (_distance_to_destination <= g.waypoint_radius) {
        if (is_negative(_desired_speed)) {
            _desired_speed = MIN(_desired_speed + attitude_control.get_decel_max() * rover.G_Dt, 0.0f);
        } else {
            _desired_speed = MAX(_desired_speed - attitude_control.get_decel_max() * rover.G_Dt, 0.0f);
        }
        _yaw_error_cd = 0.0f;
    } else {
        // P controller with hard-coded gain to convert distance to desired speed
        // To-Do: make gain configurable or calculate from attitude controller's maximum accelearation
        _desired_speed = MIN((_distance_to_destination - g.waypoint_radius) * 0.5f, g.speed_cruise);

        // calculate bearing to destination
        _desired_yaw_cd = get_bearing_cd(rover.current_loc, _destination);
        _yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
        // if destination is behind vehicle, reverse towards it
        if (fabsf(_yaw_error_cd) > 9000) {
            _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
            _yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
            _desired_speed = -_desired_speed;
        }

        // reduce desired speed if yaw_error is large
        // note: copied from calc_reduced_speed_for_turn_or_distance
        float yaw_error_ratio = 1.0f - constrain_float(fabsf(_yaw_error_cd / 9000.0f), 0.0f, 1.0f) * ((100.0f - g.speed_turn_gain) * 0.01f);
        _desired_speed *= yaw_error_ratio;
    }

    // run steering and throttle controllers
    calc_steering_to_heading(_desired_yaw_cd, _desired_speed < 0);
    calc_throttle(_desired_speed, false, true);

    // mark us as in_reverse when using a negative throttle
    // To-Do: only in reverse if vehicle is actually travelling backwards?
    rover.set_reverse(_desired_speed < 0);
}
