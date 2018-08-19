#include "Sub.h"

// checks if we should update ahrs/RTL home position from the EKF
void Sub::update_home_from_EKF()
{
    // exit immediately if home already set
    if (ahrs.home_is_set()) {
        return;
    }

    // special logic if home is set in-flight
    if (motors.armed()) {
        set_home_to_current_location_inflight();
    } else {
        // move home to current ekf location (this will set home_state to HOME_SET)
        set_home_to_current_location(false);
    }
}

// set_home_to_current_location_inflight - set home to current GPS location (horizontally) and EKF origin vertically
void Sub::set_home_to_current_location_inflight()
{
    // get current location from EKF
    Location temp_loc;
    if (inertial_nav.get_location(temp_loc)) {
        const struct Location &ekf_origin = inertial_nav.get_origin();
        temp_loc.alt = ekf_origin.alt;
        set_home(temp_loc, false);
    }
}

// set_home_to_current_location - set home to current GPS location
bool Sub::set_home_to_current_location(bool lock)
{
    // get current location from EKF
    Location temp_loc;
    if (inertial_nav.get_location(temp_loc)) {

        // Make home always at the water's surface.
        // This allows disarming and arming again at depth.
        // This also ensures that mission items with relative altitude frame, are always
        // relative to the water's surface, whether in a high elevation lake, or at sea level.
        temp_loc.alt -= barometer.get_altitude() * 100.0f;
        return set_home(temp_loc, lock);
    }
    return false;
}

// set_home - sets ahrs home (used for RTL) to specified location
//  initialises inertial nav and compass on first call
//  returns true if home location set successfully
bool Sub::set_home(const Location& loc, bool lock)
{
    // check location is valid
    if (loc.lat == 0 && loc.lng == 0) {
        return false;
    }

    // check if EKF origin has been set
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    const bool home_was_set = ahrs.home_is_set();

    // set ahrs home (used for RTL)
    ahrs.set_home(loc);

    // init inav and compass declination
    if (!home_was_set) {
        // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
        scaleLongDown = longitude_scale(loc);
        // record home is set
        Log_Write_Event(DATA_SET_HOME);

        // log new home position which mission library will pull from ahrs
        if (should_log(MASK_LOG_CMD)) {
            AP_Mission::Mission_Command temp_cmd;
            if (mission.read_cmd_from_storage(0, temp_cmd)) {
                DataFlash.Log_Write_Mission_Cmd(mission, temp_cmd);
            }
        }
    }

    // lock home position
    if (lock) {
        ahrs.lock_home();
    }

    // log ahrs home and ekf origin dataflash
    ahrs.Log_Write_Home_And_Origin();

    // send new home and ekf origin to GCS
    gcs().send_home();
    gcs().send_ekf_origin();

    // return success
    return true;
}

// far_from_EKF_origin - checks if a location is too far from the EKF origin
//  returns true if too far
bool Sub::far_from_EKF_origin(const Location& loc)
{
    // check distance to EKF origin
    const struct Location &ekf_origin = inertial_nav.get_origin();
    return (get_distance(ekf_origin, loc) > EKF_ORIGIN_MAX_DIST_M);
}
