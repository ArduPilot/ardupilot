#include "Sub.h"

// position_vector.pde related utility functions

// position vectors are Vector3f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_location_to_vector - convert lat/lon coordinates to a position vector
Vector3f Sub::pv_location_to_vector(const Location& loc)
{
    float alt_above_origin = pv_alt_above_origin(loc.alt);  // convert alt-relative-to-home to alt-relative-to-origin
    return Vector3f((loc.lat-ekf_origin.lat) * LATLON_TO_CM, (loc.lng-ekf_origin.lng) * LATLON_TO_CM * scaleLongDown, alt_above_origin);
}

// pv_alt_above_origin - convert altitude above home to altitude above EKF origin
float Sub::pv_alt_above_origin(float alt_above_home_cm)
{
    return alt_above_home_cm + (ahrs.get_home().alt - ekf_origin.alt);
}

