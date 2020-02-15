#include "AC_WPNav_OA.h"

AC_WPNav_OA::AC_WPNav_OA(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    AC_WPNav(inav, ahrs, pos_control, attitude_control)
{
}

// returns object avoidance adjusted wp location using location class
// returns false if unable to convert from target vector to global coordinates
bool AC_WPNav_OA::get_oa_wp_destination(Location& destination) const
{
    // if oa inactive return unadjusted location
    if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
        return get_wp_destination(destination);
    }

    // return latest destination provided by oa path planner
    destination = _oa_destination;
    return true;
}

/// set_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
///     terrain_alt should be true if origin.z and destination.z are desired altitudes above terrain (false if these are alt-above-ekf-origin)
///     returns false on failure (likely caused by missing terrain data)
bool AC_WPNav_OA::set_wp_origin_and_destination(const Vector3f& origin, const Vector3f& destination, bool terrain_alt)
{
    const bool ret = AC_WPNav::set_wp_origin_and_destination(origin, destination, terrain_alt);

    if (ret) {
        // reset object avoidance state
        _oa_state = AP_OAPathPlanner::OA_NOT_REQUIRED;
    }

    return ret;
}

/// get_wp_distance_to_destination - get horizontal distance to destination in cm
/// always returns distance to final destination (i.e. does not use oa adjusted destination)
float AC_WPNav_OA::get_wp_distance_to_destination() const
{
    if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
        return AC_WPNav::get_wp_distance_to_destination();
    }

    // get current location
    const Vector3f &curr = _inav.get_position();
    return norm(_destination_oabak.x-curr.x, _destination_oabak.y-curr.y);
}

/// get_wp_bearing_to_destination - get bearing to next waypoint in centi-degrees
/// always returns bearing to final destination (i.e. does not use oa adjusted destination)
int32_t AC_WPNav_OA::get_wp_bearing_to_destination() const
{
    if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
        return AC_WPNav::get_wp_bearing_to_destination();
    }

    return get_bearing_cd(_inav.get_position(), _destination_oabak);
}

/// true when we have come within RADIUS cm of the waypoint
bool AC_WPNav_OA::reached_wp_destination() const
{
    return (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) && AC_WPNav::reached_wp_destination();
}

/// update_wpnav - run the wp controller - should be called at 100hz or higher
bool AC_WPNav_OA::update_wpnav()
{
    // run path planning around obstacles
    AP_OAPathPlanner *oa_ptr = AP_OAPathPlanner::get_singleton();
    Location current_loc;
    if ((oa_ptr != nullptr) && AP::ahrs().get_position(current_loc)) {

        // backup _origin and _destination when not doing oa
        if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
            _origin_oabak = _origin;
            _destination_oabak = _destination;
        }

        // convert origin and destination to Locations and pass into oa
        const Location origin_loc(_origin_oabak);
        const Location destination_loc(_destination_oabak);
        Location oa_origin_new, oa_destination_new;
        const AP_OAPathPlanner::OA_RetState oa_retstate = oa_ptr->mission_avoidance(current_loc, origin_loc, destination_loc, oa_origin_new, oa_destination_new);
        switch (oa_retstate) {
        case AP_OAPathPlanner::OA_NOT_REQUIRED:
            if (_oa_state != oa_retstate) {
                // object avoidance has become inactive so reset target to original destination
                set_wp_destination(_destination_oabak, _terrain_alt);
                _oa_state = oa_retstate;
            }
            break;
        case AP_OAPathPlanner::OA_PROCESSING:
        case AP_OAPathPlanner::OA_ERROR:
            // during processing or in case of error stop the vehicle
            // by setting the oa_destination to a stopping point
            if ((_oa_state != AP_OAPathPlanner::OA_PROCESSING) && (_oa_state != AP_OAPathPlanner::OA_ERROR)) {
                // calculate stopping point
                Vector3f stopping_point;
                get_wp_stopping_point(stopping_point);
                _oa_destination = Location(stopping_point);
                if (set_wp_destination(stopping_point, false)) {
                    _oa_state = oa_retstate;
                }
            }
            break;
        case AP_OAPathPlanner::OA_SUCCESS:
            // if oa destination has become active or destination has changed update wpnav
            if ((_oa_state != AP_OAPathPlanner::OA_SUCCESS) || !oa_destination_new.same_latlon_as(_oa_destination)) {
                _oa_destination = oa_destination_new;
                // convert Location to offset from EKF origin
                Vector3f dest_NEU;
                if (_oa_destination.get_vector_from_origin_NEU(dest_NEU)) {
                    // calculate target altitude by calculating OA adjusted destination's distance along the original track
                    // and then linear interpolate using the original track's origin and destination altitude
                    const float dist_along_path = constrain_float(oa_destination_new.line_path_proportion(origin_loc, destination_loc), 0.0f, 1.0f);
                    dest_NEU.z = linear_interpolate(_origin_oabak.z, _destination_oabak.z, dist_along_path, 0.0f, 1.0f);
                    if (set_wp_destination(dest_NEU, _terrain_alt)) {
                        _oa_state = oa_retstate;
                    }
                }
            }
            break;
        }
    }

    // run the non-OA update
    return AC_WPNav::update_wpnav();
}
