#include "Sub.h"

// checks if we should update ahrs/RTL home position from the EKF
void Sub::update_home_from_EKF()
{
    // exit immediately if home already set
    if (ahrs.home_is_set()) {
        return;
    }
    if (!set_home_to_current_location(false)) {
        // ignore this failure
    }
}

// set_home_to_current_location - set home to current GPS location
bool Sub::set_home_to_current_location(bool lock)
{
    // get current location from EKF
    if (!ahrs.has_origin()) {
        // EKF3 will return GPS position and "true" if there is no origin
        return false;
    }
    Location temp_loc;
    if (ahrs.get_location(temp_loc)) {

        // Make home always at the water's surface.
        // This allows disarming and arming again at depth.
        // This also ensures that mission items with relative altitude frame, are always
        // relative to the water's surface, whether in a high elevation lake, or at sea level.
        temp_loc.offset_up_m(-barometer.get_altitude());
        return set_home(temp_loc, lock);
    }
    return false;
}

// set_home - sets ahrs home (used for RTL) to specified location
//  returns true if home location set successfully
bool Sub::set_home(const Location& loc, bool lock)
{
    return ahrs.set_home(loc, lock);
}
