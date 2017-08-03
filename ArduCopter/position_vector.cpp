#include "Copter.h"

// Position vectors related utility functions

// position vectors are Vector3f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_location_to_vector - convert lat/lon coordinates to a position vector
Vector3f Copter::pv_location_to_vector(const Location& loc)
{
    float alt_above_origin = pv_alt_above_origin(loc.alt);  // convert alt-relative-to-home to alt-relative-to-origin
    return Vector3f((loc.lat-ekf_origin.lat) * LATLON_TO_CM, (loc.lng-ekf_origin.lng) * LATLON_TO_CM * scaleLongDown, alt_above_origin);
}

// pv_alt_above_origin - convert altitude above home to altitude above EKF origin
float Copter::pv_alt_above_origin(float alt_above_home_cm)
{
    return alt_above_home_cm + (ahrs.get_home().alt - ekf_origin.alt);
}

// pv_alt_above_home - convert altitude above EKF origin to altitude above home
float Copter::pv_alt_above_home(float alt_above_origin_cm)
{
    return alt_above_origin_cm + (ekf_origin.alt - ahrs.get_home().alt);
}

// pv_get_bearing_cd - return bearing in centi-degrees between two positions
float Copter::pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination)
{
    float bearing = atan2f(destination.y-origin.y, destination.x-origin.x) * DEGX100;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

// pv_get_horizontal_distance_cm - return distance between two positions in cm
float Copter::pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination)
{
    return norm(destination.x-origin.x,destination.y-origin.y);
}

// returns distance between a destination and home in cm
float Copter::pv_distance_to_home_cm(const Vector3f &destination)
{
    Vector3f home = pv_location_to_vector(ahrs.get_home());
    return pv_get_horizontal_distance_cm(home, destination);
}
