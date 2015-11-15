// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
void Rover::navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	if (!have_position) {
		return;
	}

	if ((next_WP.lat == 0)||(home_is_set==false)){
		return;
	}

	// waypoint distance from rover
	// ----------------------------
	wp_distance = get_distance(current_loc, next_WP);

	if (wp_distance < 0){
		gcs_send_text_P(SEVERITY_HIGH,PSTR("<navigate> WP error - distance < 0"));
		return;
	}

    calc_home_distance_and_bearing();

	// control mode specific updates to nav_bearing
	// --------------------------------------------
	update_navigation();
}

// calc_home_distance_and_bearing - calculate distance and bearing to home for reporting and autopilot decisions
void Rover::calc_home_distance_and_bearing()
{
    // calculate home distance and bearing
    Vector3f home_v = pv_location_to_vector(ahrs.get_home());
    Vector3f curr_v = inertial_nav.get_position();
    int32_t home_distance = pv_get_horizontal_distance_cm(curr_v, home_v);
    int32_t home_bearing = pv_get_bearing_cd(curr_v,home_v);

    // give frsky library our current distance from home
    frsky_telemetry.set_home_distance(home_distance);
    // give frsky library our current bearing from home
    frsky_telemetry.set_home_bearing(home_bearing);
}

// pv_location_to_vector - convert lat/lon coordinates to a position vector
Vector3f Rover::pv_location_to_vector(const Location& loc)
{
    const struct Location &origin = inertial_nav.get_origin();
    float alt_above_origin = pv_alt_above_origin(loc.alt);  // convert alt-relative-to-home to alt-relative-to-origin
	// update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
    float scaleLongDown = longitude_scale(loc);
    return Vector3f((loc.lat-origin.lat) * LATLON_TO_CM, (loc.lng-origin.lng) * LATLON_TO_CM * scaleLongDown, alt_above_origin);
}

// pv_alt_above_origin - convert altitude above home to altitude above EKF origin
float Rover::pv_alt_above_origin(float alt_above_home_cm)
{
    const struct Location &origin = inertial_nav.get_origin();
    return alt_above_home_cm + (ahrs.get_home().alt - origin.alt);
}

// pv_get_horizontal_distance_cm - return distance between two positions in cm
float Rover::pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination)
{
    return pythagorous2(destination.x-origin.x,destination.y-origin.y);
}

// pv_get_bearing_cd - return bearing in centi-degrees between two positions
float Rover::pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination)
{
    float bearing = atan2f(destination.y-origin.y, destination.x-origin.x) * DEGX100;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}
