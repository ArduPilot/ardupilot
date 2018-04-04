#include "Copter.h"

// Position vectors related utility functions

// position vectors are Vector3f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_location_to_vector - convert lat/lon coordinates to a position vector
bool Copter::pv_location_to_vector(const Location& loc, Vector3f &vec)
{
    struct Location origin;
    if (!ahrs.get_origin(origin)) {
        return false;
    }
    // convert alt-relative-to-home to alt-relative-to-origin
    float alt_above_origin;
    if (!pv_alt_above_origin(loc.alt, alt_above_origin)) {
        return false;
    }
    vec = Vector3f((loc.lat-origin.lat) * LATLON_TO_CM, (loc.lng-origin.lng) * LATLON_TO_CM * scaleLongDown, alt_above_origin);
    return true;
}

// pv_alt_above_origin - convert altitude above home to altitude above EKF origin
bool Copter::pv_alt_above_origin(float alt_above_home_cm, float &ret)
{
    struct Location origin;
    if (!ahrs.get_origin(origin)) {
        return false;
    }
    if (!ahrs.home_is_set()) {
        return false;
    }
    ret = alt_above_home_cm + (ahrs.get_home().alt - origin.alt);
    return true;
}

// pv_alt_above_home - convert altitude above EKF origin to altitude above home
bool Copter::pv_alt_above_home(float alt_above_origin_cm, float &ret)
{
    struct Location origin;
    if (!ahrs.get_origin(origin)) {
        return false;
    }
    if (!ahrs.home_is_set()) {
        return false;
    }
    ret = alt_above_origin_cm + (origin.alt - ahrs.get_home().alt);
    return true;
}

// returns distance between a destination and home in cm
bool Copter::pv_distance_to_home_cm(const Vector3f &destination, float &ret)
{
    if (!ahrs.home_is_set()) {
        return false;
    }
    Vector3f home;
    if (!pv_location_to_vector(ahrs.get_home(), home)) {
        return false;
    }
    ret = get_horizontal_distance_cm(home, destination);
    return true;
}
