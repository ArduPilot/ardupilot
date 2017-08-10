#include "mode.h"
#include "Rover.h"

bool ModeRTL::_enter()
{
    // refuse RTL if home has not been set
    if (rover.home_is_set == HOME_UNSET) {
        return false;
    }

    // set destination
    set_desired_location(rover.home);

    // RTL never reverses
    rover.set_reverse(false);

    g2.motors.slew_limit_throttle(true);
    return true;
}

void ModeRTL::update()
{
    if (!_reached_destination) {
        // calculate distance to home
        _distance_to_destination = get_distance(rover.current_loc, _destination);
        // check if we've reached the destination
        if (_distance_to_destination <= rover.g.waypoint_radius || location_passed_point(rover.current_loc, _origin, _destination)) {
            // trigger reached
            _reached_destination = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Reached destination");
        }
        // continue driving towards destination
        calc_lateral_acceleration(_origin, _destination);
        calc_nav_steer();
        calc_throttle(calc_reduced_speed_for_turn_or_distance(_desired_speed), true);
    } else {
        // we've reached destination so stop
        stop_vehicle();
        lateral_acceleration = 0.0f;
    }
}
