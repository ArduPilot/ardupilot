#include "AC_WPNav_config.h"

#if AC_WPNAV_OA_ENABLED

#include <AP_Math/control.h>
#include <AP_InternalError/AP_InternalError.h>
#include "AC_WPNav_OA.h"

AC_WPNav_OA::AC_WPNav_OA(const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    AC_WPNav(ahrs, pos_control, attitude_control)
{
}

// Returns the object-avoidance-adjusted waypoint location (in global coordinates).
// Falls back to original destination if OA is not active.
bool AC_WPNav_OA::get_oa_wp_destination(Location& destination) const
{
    // Return unmodified global destination if OA is not active
    if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
        return get_wp_destination_loc(destination);
    }

    // OA is active — return path-planner-adjusted intermediate destination
    destination = _oa_destination;
    return true;
}

// Sets the waypoint destination using NEU coordinates in centimeters.
// See set_wp_destination_NED_m() for full details.
bool AC_WPNav_OA::set_wp_destination_NEU_cm(const Vector3f& destination_neu_cm, bool is_terrain_alt)
{
    // Convert input from NEU centimeters to NED meters and delegate to meter version
    Vector3p destination_ned_m = Vector3p(destination_neu_cm.x, destination_neu_cm.y, -destination_neu_cm.z) * 0.01;
    return set_wp_destination_NED_m(destination_ned_m, is_terrain_alt);
}

// Sets the waypoint destination using NED coordinates in meters.
// - destination_ned_m: NED offset from EKF origin in meters.
// - is_terrain_alt: true if the destination_ned_m is relative to the terrain surface.
// - Resets OA state on success.
bool AC_WPNav_OA::set_wp_destination_NED_m(const Vector3p& destination_ned_m, bool is_terrain_alt, float arc_ang_rad)
{
    // Call base implementation to set destination and terrain-altitude flag
    const bool ret = AC_WPNav::set_wp_destination_NED_m(destination_ned_m, is_terrain_alt, arc_ang_rad);

    // If destination set successfully, reset OA state to inactive
    if (ret) {
        // reset object avoidance state
        _oa_state = AP_OAPathPlanner::OA_NOT_REQUIRED;
    }

    return ret;
}

// Returns the horizontal distance to the final destination in centimeters.
// See get_wp_distance_to_destination_m() for full details.
float AC_WPNav_OA::get_wp_distance_to_destination_cm() const
{
    // Convert horizontal distance from meters to centimeters
    return get_wp_distance_to_destination_m() * 100.0;
}

// Returns the horizontal distance to the final destination in meters.
// Ignores OA-adjusted targets and always measures to the original final destination.
float AC_WPNav_OA::get_wp_distance_to_destination_m() const
{
    // Return horizontal distance to final destination (ignoring OA intermediate goals)
    if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
        return AC_WPNav::get_wp_distance_to_destination_m();
    }

    // Compute distance to original destination using backed-up NEU position
    return get_horizontal_distance(_pos_control.get_pos_estimate_NED_m().xy(), _destination_oabak_ned_m.xy());
}

// Returns the bearing to the final destination in centidegrees.
// See get_wp_bearing_to_destination_rad() for full details.
int32_t AC_WPNav_OA::get_wp_bearing_to_destination_cd() const
{
    // Convert bearing to destination (in radians) to centidegrees
    return rad_to_cd(get_wp_bearing_to_destination_rad());
}

// Returns the bearing to the final destination in radians.
// Ignores OA-adjusted targets and always calculates from original final destination.
float AC_WPNav_OA::get_wp_bearing_to_destination_rad() const
{
    // Use base class method if object avoidance is inactive
    if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
        return AC_WPNav::get_wp_bearing_to_destination_rad();
    }

    // Return bearing to the original destination, not the OA-adjusted one
    return get_bearing_rad(_pos_control.get_pos_estimate_NED_m().xy().tofloat(), _destination_oabak_ned_m.xy().tofloat());
}

// Returns true if the vehicle has reached the final destination within radius threshold.
// Ignores OA-adjusted intermediate destinations.
bool AC_WPNav_OA::reached_wp_destination() const
{
    // Only consider the waypoint reached if OA is inactive and base class condition is met
    return (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) && AC_WPNav::reached_wp_destination();
}

// Runs the waypoint navigation update loop, including OA path planning logic.
// Delegates to parent class if OA is not active or not required.
bool AC_WPNav_OA::update_wpnav()
{
    // Run path planning logic using the active OA planner
    AP_OAPathPlanner *oa_ptr = AP_OAPathPlanner::get_singleton();
    Location current_loc;
    if ((oa_ptr != nullptr) && AP::ahrs().get_location(current_loc)) {

        // Backup current path state before OA modifies it
        if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
            _origin_oabak_ned_m = _origin_ned_m;
            _destination_oabak_ned_m = _destination_ned_m;
            _is_terrain_alt_oabak = _is_terrain_alt;
            _next_destination_oabak_ned_m = _next_destination_ned_m;
        }

        // Convert backup path state to global Location objects for planner input
        const Location origin_loc = Location::from_ekf_offset_NED_m(_origin_oabak_ned_m, _is_terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
        const Location destination_loc = Location::from_ekf_offset_NED_m(_destination_oabak_ned_m, _is_terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
        const Location next_destination_loc = Location::from_ekf_offset_NED_m(_next_destination_oabak_ned_m, _is_terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
        Location oa_origin_new, oa_destination_new, oa_next_destination_new;
        bool dest_to_next_dest_clear = true;
        AP_OAPathPlanner::OAPathPlannerUsed path_planner_used = AP_OAPathPlanner::OAPathPlannerUsed::None;

        // Request obstacle-avoidance-adjusted path from planner
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
            // OA is no longer needed — restore original destination and optionally set next
            if (_oa_state != oa_retstate) {
                // object avoidance has become inactive so reset target to original destination
                if (!set_wp_destination_NED_m(_destination_oabak_ned_m, _is_terrain_alt_oabak)) {
                    // trigger terrain failsafe
                    return false;
                }

                // if path from destination to next_destination is clear
                if (dest_to_next_dest_clear && (oa_ptr->get_options() & AP_OAPathPlanner::OA_OPTION_FAST_WAYPOINTS)) {
                    // set next destination if non-zero
                    if (!_next_destination_oabak_ned_m.is_zero()) {
                        set_wp_destination_next_NED_m(_next_destination_oabak_ned_m);
                    }
                }
                _oa_state = oa_retstate;
            }

            // Prevent transitioning past this waypoint if path to next is unclear
            // Note that this check is run on every iteration even if the path planner is not active
            if (!dest_to_next_dest_clear) {
                force_stop_at_next_wp();
            }
            break;

        case AP_OAPathPlanner::OA_PROCESSING:
            // Allow continued movement while OA path is processing if fast-waypointing is enabled
            if (oa_ptr->get_options() & AP_OAPathPlanner::OA_OPTION_FAST_WAYPOINTS) {
                // if fast waypoint option is set, proceed during processing
                break;
            }
            FALLTHROUGH;

        case AP_OAPathPlanner::OA_ERROR:
            // OA temporarily failing — stop vehicle at current position
            if ((_oa_state != AP_OAPathPlanner::OA_PROCESSING) && (_oa_state != AP_OAPathPlanner::OA_ERROR)) {
                // calculate stopping point
                Vector3p stopping_point_ned_m;
                get_wp_stopping_point_NED_m(stopping_point_ned_m);
                _oa_destination = Location::from_ekf_offset_NED_m(stopping_point_ned_m, Location::AltFrame::ABOVE_ORIGIN);
                _oa_next_destination.zero();
                if (set_wp_destination_NED_m(stopping_point_ned_m, false)) {
                    _oa_state = oa_retstate;
                }
            }
            break;

        case AP_OAPathPlanner::OA_SUCCESS:

            // Handle result differently depending on which OA planner was used
            switch (path_planner_used) {

            case AP_OAPathPlanner::OAPathPlannerUsed::None:
                // this should never happen.  this means the path planner has returned success but has failed to set the path planner used
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return false;

            case AP_OAPathPlanner::OAPathPlannerUsed::Dijkstras:
                // Dijkstra's.  Action is only needed if path planner has just became active or the target destination's lat or lon has changed
                // Interpolate altitude and set new target if different or first OA success
                if ((_oa_state != AP_OAPathPlanner::OA_SUCCESS) || !oa_destination_new.same_latlon_as(_oa_destination)) {
                    Location origin_oabak_loc = Location::from_ekf_offset_NED_m(_origin_oabak_ned_m, _is_terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
                    Location destination_oabak_loc = Location::from_ekf_offset_NED_m(_destination_oabak_ned_m, _is_terrain_alt_oabak ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
                    oa_destination_new.linearly_interpolate_alt(origin_oabak_loc, destination_oabak_loc);

                    // set new OA adjusted destination
                    if (!set_wp_destination_loc(oa_destination_new)) {
                        // trigger terrain failsafe
                        return false;
                    }
                    // if new target set successfully, update oa state and destination
                    _oa_state = oa_retstate;
                    _oa_destination = oa_destination_new;

                    // Set next destination if provided
                    if ((oa_ptr->get_options() & AP_OAPathPlanner::OA_OPTION_FAST_WAYPOINTS) && !oa_next_destination_new.is_zero()) {
                        // calculate oa_next_destination_new's altitude using linear interpolation between original origin and destination
                        // this "next destination" is still an intermediate point between the origin and destination
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

                // Adjust altitude based on current progress along the path
                Location target_alt_loc = current_loc;
                target_alt_loc.linearly_interpolate_alt(origin_loc, destination_loc);

                // Get terrain offset if needed
                float terrain_d_m = 0;
                if (_is_terrain_alt_oabak && !get_terrain_D_m(terrain_d_m)) {
                    // trigger terrain failsafe
                    return false;
                }

                // Convert global destination to NEU vector and pass directly to position controller
                Vector2f destination_ne_m;
                if (!_oa_destination.get_vector_xy_from_origin_NE_m(destination_ne_m)) {
                    // this should never happen because we can only get here if we have an EKF origin
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                    return false;
                }
                float target_alt_loc_alt_m = 0;
                UNUSED_RESULT(target_alt_loc.get_alt_m(target_alt_loc.get_alt_frame(), target_alt_loc_alt_m));
                Vector3p destination_ned_m{destination_ne_m.x, destination_ne_m.y, target_alt_loc_alt_m};

                // pass the desired position directly to the position controller
                _pos_control.input_pos_NED_m(destination_ned_m, terrain_d_m, 10.0);

                // update horizontal position controller (vertical is updated in vehicle code)
                _pos_control.NE_update_controller();

                // return success without calling parent AC_WPNav
                return true;
            }

            case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerVertical: {
                _oa_state = oa_retstate;
                _oa_destination = oa_destination_new;

                // Convert final destination to NEU offset and push to position controller
                Vector3p destination_ned_m;
                if (!_oa_destination.get_vector_from_origin_NED_m(destination_ned_m)) {
                    // this should never happen because we can only get here if we have an EKF origin
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                    return false;
                }

                // pass the desired position directly to the position controller as an offset from EKF origin in NEU
                _pos_control.input_pos_NED_m(destination_ned_m, 0, 10.0);

                // update horizontal position controller (vertical is updated in vehicle code)
                _pos_control.NE_update_controller();

                // return success without calling parent AC_WPNav
                return true;
            }

            }
        }
    }

    // Run standard waypoint update if OA was not active or handled above
    return AC_WPNav::update_wpnav();
}

#endif  // Ac_WPNAV_OA_ENABLED
