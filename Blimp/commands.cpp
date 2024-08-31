#include "Blimp.h"

// checks if we should update ahrs/RTL home position from the EKF
void Blimp::update_home_from_EKF()
{
    // exit immediately if home already set
    if (ahrs.home_is_set()) {
        return;
    }

    // special logic if home is set in-flight
    if (motors->armed()) {
        set_home_to_current_location_inflight();
    } else {
        // move home to current ekf location (this will set home_state to HOME_SET)
        if (!set_home_to_current_location(false)) {
            // ignore failure
        }
    }
}

// set_home_to_current_location_inflight - set home to current GPS location (horizontally) and EKF origin vertically
void Blimp::set_home_to_current_location_inflight()
{
    // get current location from EKF
    Location temp_loc;
    Location ekf_origin;
    if (ahrs.get_location(temp_loc) && ahrs.get_origin(ekf_origin)) {
        temp_loc.alt = ekf_origin.alt;
        if (!set_home(temp_loc, false)) {
            return;
        }
    }
}
