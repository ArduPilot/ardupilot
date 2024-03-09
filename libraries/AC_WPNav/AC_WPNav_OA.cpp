#include "AC_WPNav_config.h"

#if AC_WPNAV_OA_ENABLED

#include <AP_Math/control.h>
#include <AP_InternalError/AP_InternalError.h>
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
        return get_wp_destination_loc(destination);
    }

    // return latest destination provided by oa path planner
    destination = _oa_destination;
    return true;
}

/// set_wp_destination waypoint using position vector (distance from ekf origin in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain
///     returns false on failure (likely caused by missing terrain data)
bool AC_WPNav_OA::set_wp_destination(const Vector3f& destination, bool terrain_alt)
{
    const bool ret = AC_WPNav::set_wp_destination(destination, terrain_alt);

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

    return get_horizontal_distance_cm(_inav.get_position_xy_cm(), _destination_oabak.xy());
}

/// get_wp_bearing_to_destination - get bearing to next waypoint in centi-degrees
/// always returns bearing to final destination (i.e. does not use oa adjusted destination)
int32_t AC_WPNav_OA::get_wp_bearing_to_destination() const
{
    if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
        return AC_WPNav::get_wp_bearing_to_destination();
    }

    return get_bearing_cd(_inav.get_position_xy_cm(), _destination_oabak.xy());
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
    if ((oa_ptr != nullptr) && AP::ahrs().get_location(current_loc)) {

        // backup _origin and _destination when not doing oa
        if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
            _origin_oabak = _origin;
            _destination_oabak = _destination;
            _terrain_alt_oabak = _terrain_alt;
            _next_destination_oabak = _next_destination;
        }

        // convert origin, destination and next_destination to Locations and pass into oa
        const Location origin_loc(_origin_oabak, _terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
        const Location destination_loc(_destination_oabak, _terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
        const Location next_destination_loc(_next_destination_oabak, _terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
        Location oa_origin_new, oa_destination_new, oa_next_destination_new;
        bool dest_to_next_dest_clear = true;
        AP_OAPathPlanner::OAPathPlannerUsed path_planner_used = AP_OAPathPlanner::OAPathPlannerUsed::None;
        const AP_OAPathPlanner::OA_RetState oa_retstate = oa_ptr->mission_avoidance(current_loc,
                                                                                    origin_loc,
                                                                                    destination_loc,
                                                                                    next_destination_loc,
                                                                                    oa_origin_new,
                                                                                    oa_destination_new,
                                                                                    oa_next_destination_new,
                                                                                    dest_to_next_dest_clear,
                                                                                    path_planner_used);

        switch (oa_retstate) {

        case AP_OAPathPlanner::OA_NOT_REQUIRED:
            if (_oa_state != oa_retstate) {
                // object avoidance has become inactive so reset target to original destination
                if (!set_wp_destination(_destination_oabak, _terrain_alt_oabak)) {
                    // trigger terrain failsafe
                    return false;
                }

                // if path from destination to next_destination is clear
                if (dest_to_next_dest_clear && (oa_ptr->get_options() & AP_OAPathPlanner::OA_OPTION_FAST_WAYPOINTS)) {
                    // set next destination if non-zero
                    if (!_next_destination_oabak.is_zero()) {
                        set_wp_destination_next(_next_destination_oabak);
                    }
                }
                _oa_state = oa_retstate;
            }

            // ensure we stop at next waypoint
            // Note that this check is run on every iteration even if the path planner is not active
            if (!dest_to_next_dest_clear) {
                force_stop_at_next_wp();
            }
            break;

        case AP_OAPathPlanner::OA_PROCESSING:
            if (oa_ptr->get_options() & AP_OAPathPlanner::OA_OPTION_FAST_WAYPOINTS) {
                // if fast waypoint option is set, proceed during processing
                break;
            }
            FALLTHROUGH;

        case AP_OAPathPlanner::OA_ERROR:
            // during processing or in case of error stop the vehicle
            // by setting the oa_destination to a stopping point
            if ((_oa_state != AP_OAPathPlanner::OA_PROCESSING) && (_oa_state != AP_OAPathPlanner::OA_ERROR)) {
                // calculate stopping point
                Vector3f stopping_point;
                get_wp_stopping_point(stopping_point);
                _oa_destination = Location(stopping_point, Location::AltFrame::ABOVE_ORIGIN);
                _oa_next_destination.zero();
                if (set_wp_destination(stopping_point, false)) {
                    _oa_state = oa_retstate;
                }
            }
            break;

        case AP_OAPathPlanner::OA_SUCCESS:

            // handling of returned destination depends upon path planner used
            switch (path_planner_used) {

            case AP_OAPathPlanner::OAPathPlannerUsed::None:
                // this should never happen.  this means the path planner has returned success but has failed to set the path planner used
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return false;

            case AP_OAPathPlanner::OAPathPlannerUsed::Dijkstras:
                // Dijkstra's.  Action is only needed if path planner has just became active or the target destination's lat or lon has changed
                if ((_oa_state != AP_OAPathPlanner::OA_SUCCESS) || !oa_destination_new.same_latlon_as(_oa_destination)) {
                    Location origin_oabak_loc(_origin_oabak, _terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
                    Location destination_oabak_loc(_destination_oabak, _terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
                    oa_destination_new.linearly_interpolate_alt(origin_oabak_loc, destination_oabak_loc);

                    // set new OA adjusted destination
                    if (!set_wp_destination_loc(oa_destination_new)) {
                        // trigger terrain failsafe
                        return false;
                    }
                    // if new target set successfully, update oa state and destination
                    _oa_state = oa_retstate;
                    _oa_destination = oa_destination_new;

                    // if a next destination was provided then use it
                    if ((oa_ptr->get_options() & AP_OAPathPlanner::OA_OPTION_FAST_WAYPOINTS) && !oa_next_destination_new.is_zero()) {
                        // calculate oa_next_destination_new's altitude using linear interpolation between original origin and destination
                        // this "next destination" is still an intermediate point between the origin and destination
                        Location next_destination_oabak_loc(_next_destination_oabak, _terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
                        oa_next_destination_new.linearly_interpolate_alt(origin_oabak_loc, destination_oabak_loc);
                        if (set_wp_destination_next_loc(oa_next_destination_new)) {
                            _oa_next_destination = oa_next_destination_new;
                        }
                    }
                }
                break;

            case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerHorizontal: {
                _oa_state = oa_retstate;
                _oa_destination = oa_destination_new;

                // altitude target interpolated from current_loc's distance along the original path
                Location target_alt_loc = current_loc;
                target_alt_loc.linearly_interpolate_alt(origin_loc, destination_loc);

                // correct target_alt_loc's alt-above-ekf-origin if using terrain altitudes
                // positive terr_offset means terrain below vehicle is above ekf origin's altitude
                float terr_offset = 0;
                if (_terrain_alt_oabak && !get_terrain_offset(terr_offset)) {
                    // trigger terrain failsafe
                    return false;
                }

                // calculate final destination as an offset from EKF origin in NEU
                Vector2f dest_NE;
                if (!_oa_destination.get_vector_xy_from_origin_NE(dest_NE)) {
                    // this should never happen because we can only get here if we have an EKF origin
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                    return false;
                }
                Vector3p dest_NEU{dest_NE.x, dest_NE.y, (float)target_alt_loc.alt};

                // pass the desired position directly to the position controller
                _pos_control.input_pos_xyz(dest_NEU, terr_offset, 1000.0);

                // update horizontal position controller (vertical is updated in vehicle code)
                _pos_control.update_xy_controller();

                // return success without calling parent AC_WPNav
                return true;
            }

            case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerVertical: {
                _oa_state = oa_retstate;
                _oa_destination = oa_destination_new;

                // calculate final destination as an offset from EKF origin in NEU
                Vector3f dest_NEU;
                if (!_oa_destination.get_vector_from_origin_NEU(dest_NEU)) {
                    // this should never happen because we can only get here if we have an EKF origin
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                    return false;
                }

                // pass the desired position directly to the position controller as an offset from EKF origin in NEU
                Vector3p dest_NEU_p{dest_NEU.x, dest_NEU.y, dest_NEU.z};
                _pos_control.input_pos_xyz(dest_NEU_p, 0, 1000.0);

                // update horizontal position controller (vertical is updated in vehicle code)
                _pos_control.update_xy_controller();

                // return success without calling parent AC_WPNav
                return true;
            }

            }
        }
    }

    // run the non-OA update
    return AC_WPNav::update_wpnav();
}

#endif  // Ac_WPNAV_OA_ENABLED
