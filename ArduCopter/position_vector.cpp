#include "Copter.h"

// Position vectors related utility functions

// position vectors are Vector3f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm


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
