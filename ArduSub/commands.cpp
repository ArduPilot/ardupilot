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
    // check if EKF origin has been set
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    Location new_home_loc = loc;

    // If the caller's requested altitude is exactly 0 in the absolute
    // (AMSL) frame, treat it as a request to lock home to the current
    // water surface instead. A deliberate home altitude of exactly AMSL 0
    // is never a meaningful choice for a submersible vehicle; this only
    // catches the ambiguous case from #33827, where a GCS lets a user set
    // home to "0m" meaning AMSL rather than the water surface. Any other
    // explicit altitude supplied by the caller (e.g. this class's own
    // set_home_to_current_location(), or a test explicitly setting home
    // to a specific altitude) is respected unmodified.
    int32_t requested_alt_cm;
    if (new_home_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, requested_alt_cm) && requested_alt_cm == 0) {
        Location surface_ref_loc;
        if (ahrs.get_location(surface_ref_loc)) {
            surface_ref_loc.offset_up_m(-barometer.get_altitude());
            int32_t surface_alt_cm;
            if (surface_ref_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, surface_alt_cm)) {
                new_home_loc.set_alt_cm(surface_alt_cm, Location::AltFrame::ABSOLUTE);
            }
        }
    }

    // set ahrs home (used for RTL)
    if (!ahrs.set_home(new_home_loc)) {
        return false;
    }

    // lock home position
    if (lock) {
        ahrs.lock_home();
    }

    // return success
    return true;
}
