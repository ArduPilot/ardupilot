#include "Rover.h"

// set ahrs home to current location from inertial-nav location
bool Rover::set_home_to_current_location(bool lock)
{
    Location temp_loc;
    if (ahrs.have_inertial_nav() && ahrs.get_location(temp_loc)) {
        if (!set_home(temp_loc, lock)) {
            return false;
        }
        // we have successfully set AHRS home, set it for SmartRTL
        g2.smart_rtl.set_home(true);
        return true;
    }
    return false;
}

// called periodically while disarmed to update our home position to
// our current location
void Rover::update_home()
{
    if (ahrs.home_is_locked()) {
        // we've been explicitly told our home location
        return;
    }

    Location loc{};
    if (!ahrs.get_location(loc)) {
        return;
    }

    barometer.update_calibration();

    if (ahrs.home_is_set() &&
        loc.get_distance(ahrs.get_home()) < DISTANCE_HOME_MINCHANGE) {
        // insufficiently moved from current home - don't change it
        return;
    }

    IGNORE_RETURN(ahrs.set_home(loc));
}
