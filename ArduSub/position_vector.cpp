#include "Sub.h"

// position_vector.pde related utility functions

// position vectors are Vector3f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_location_to_vector - convert lat/lon coordinates to a position vector
Vector3f Sub::pv_location_to_vector(const Location& loc)
{
    Location origin;
    if (!ahrs.get_origin(origin)) {
        origin.zero();
    }
    float alt_above_origin = pv_alt_above_origin(loc.alt);  // convert alt-relative-to-home to alt-relative-to-origin
    Vector3f vec = origin.get_distance_NED(loc);
    vec.xy() *= 100;
    vec.z = alt_above_origin;
    return vec;
}

// pv_alt_above_origin - convert altitude above home to altitude above EKF origin
float Sub::pv_alt_above_origin(float alt_above_home_cm)
{
    Location origin;
    if (!ahrs.get_origin(origin)) {
        origin.zero();
    }
    return alt_above_home_cm + (ahrs.get_home().alt - origin.alt);
}

