#include "Copter.h"

// Position vectors related utility functions

// position vectors are Vector3f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_location_to_vector - convert lat/lon coordinates to a position vector
Vector3f Copter::pv_location_to_vector(const Location& loc)
{
    const struct Location &origin = inertial_nav.get_origin();
    float alt_above_origin = pv_alt_above_origin(loc.alt);  // convert alt-relative-to-home to alt-relative-to-origin
    return Vector3f((loc.lat-origin.lat) * LATLON_TO_CM, (loc.lng-origin.lng) * LATLON_TO_CM * scaleLongDown, alt_above_origin);
}

// pv_alt_above_origin - convert altitude above home to altitude above EKF origin
float Copter::pv_alt_above_origin(float alt_above_home_cm)
{
    const struct Location &origin = inertial_nav.get_origin();
    return alt_above_home_cm + (ahrs.get_home().alt - origin.alt);
}

// pv_alt_above_home - convert altitude above EKF origin to altitude above home
float Copter::pv_alt_above_home(float alt_above_origin_cm)
{
    const struct Location &origin = inertial_nav.get_origin();
    return alt_above_origin_cm + (origin.alt - ahrs.get_home().alt);
}

// returns distance between a destination and home in cm
float Copter::pv_distance_to_home_cm(const Vector3f &destination)
{
    Vector3f home = pv_location_to_vector(ahrs.get_home());
    return get_horizontal_distance_cm(home, destination);
}
