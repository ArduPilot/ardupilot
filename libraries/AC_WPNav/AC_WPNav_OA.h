#pragma once

#include "AC_WPNav_config.h"

#if AC_WPNAV_OA_ENABLED

#include <AC_WPNav/AC_WPNav.h>
#include <AC_Avoidance/AP_OAPathPlanner.h>
#include <AC_Avoidance/AP_OABendyRuler.h>

class AC_WPNav_OA : public AC_WPNav
{

public:
    /// Constructor
    AC_WPNav_OA(const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    // Returns the object-avoidance-adjusted waypoint location (in global coordinates).
    // Falls back to original destination if OA is not active.
    bool get_oa_wp_destination(Location& destination) const override;

    // Sets the waypoint destination using NEU coordinates in centimeters.
    // See set_wp_destination_NED_m() for full details.
    bool set_wp_destination_NEU_cm(const Vector3f& destination_neu_cm, bool is_terrain_alt = false) override;

    // Sets the waypoint destination using NED coordinates in meters.
    // - destination_ned_m: NED offset from EKF origin in meters.
    // - is_terrain_alt: true if the destination_ned_m is relative to the terrain surface.
    // arc_rad specifies the signed arc angle in radians for an ARC_WAYPOINT segment (0 for straight path)
    // - Resets OA state on success.
    bool set_wp_destination_NED_m(const Vector3p& destination_ned_m, bool is_terrain_alt = false, float arc_rad = 0.0) override;

    // Returns the horizontal distance to the final destination in centimeters.
    // See get_wp_distance_to_destination_m() for full details.
    float get_wp_distance_to_destination_cm() const override;

    // Returns the horizontal distance to the final destination in meters.
    // Ignores OA-adjusted targets and always measures to the original final destination.
    float get_wp_distance_to_destination_m() const override;

    // Returns the bearing to the final destination in centidegrees.
    // See get_wp_bearing_to_destination_rad() for full details.
    int32_t get_wp_bearing_to_destination_cd() const override;

    // Returns the bearing to the final destination in radians.
    // Ignores OA-adjusted targets and always calculates from original final destination.
    virtual float get_wp_bearing_to_destination_rad() const override;

    // Returns true if the vehicle has reached the final destination within radius threshold.
    // Ignores OA-adjusted intermediate destinations.
    bool reached_wp_destination() const override;

    // Runs the waypoint navigation update loop, including OA path planning logic.
    // Delegates to parent class if OA is not active or not required.
    bool update_wpnav() override;

protected:

    // oa path planning variables
    AP_OAPathPlanner::OA_RetState _oa_state;    // state of object avoidance, if OA_SUCCESS we use _oa_destination to avoid obstacles
    Vector3p    _origin_oabak_ned_m;            // backup of _origin_ned_m so it can be restored when oa completes
    Vector3p    _destination_oabak_ned_m;       // backup of _destination_ned_m so it can be restored when oa completes
    Vector3p    _next_destination_oabak_ned_m;  // backup of _next_destination_ned_m so it can be restored when oa completes
    bool        _is_terrain_alt_oabak;  // true if backup origin and destination z-axis are terrain altitudes
    Location    _oa_destination;        // intermediate destination during avoidance
    Location    _oa_next_destination;   // intermediate next destination during avoidance
};

#endif  // AC_WPNAV_OA_ENABLED
