// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * the home_state has a number of possible values (see enum HomeState in defines.h's)
 *   HOME_UNSET             = home is not set, no GPS positions yet received
 *   HOME_SET_NOT_LOCKED    = home is set to EKF origin or armed location (can be moved)
 *   HOME_SET_AND_LOCKED    = home has been set by user, cannot be moved except by user initiated do-set-home command
 */

// checks if we should update ahrs/RTL home position from the EKF
void Copter::update_home_from_EKF()
{
    // exit immediately if home already set
    if (ap.home_state != HOME_UNSET) {
        return;
    }

    // special logic if home is set in-flight
    if (motors.armed()) {
        set_home_to_current_location_inflight();
    } else {
        // move home to current ekf location (this will set home_state to HOME_SET)
        set_home_to_current_location();
    }
}

// set_home_to_current_location_inflight - set home to current GPS location (horizontally) and EKF origin vertically
void Copter::set_home_to_current_location_inflight() {
    // get current location from EKF
    Location temp_loc;
    if (inertial_nav.get_location(temp_loc)) {
        const struct Location &ekf_origin = inertial_nav.get_origin();
        temp_loc.alt = ekf_origin.alt;
        set_home(temp_loc);
    }
}

// set_home_to_current_location - set home to current GPS location
bool Copter::set_home_to_current_location() {
    // get current location from EKF
    Location temp_loc;
    if (inertial_nav.get_location(temp_loc)) {
        return set_home(temp_loc);
    }
    return false;
}

// set_home_to_current_location_and_lock - set home to current location and lock so it cannot be moved
bool Copter::set_home_to_current_location_and_lock()
{
    if (set_home_to_current_location()) {
        set_home_state(HOME_SET_AND_LOCKED);
        return true;
    }
    return false;
}

// set_home_and_lock - sets ahrs home (used for RTL) to specified location and locks so it cannot be moved
//  unless this function is called again
bool Copter::set_home_and_lock(const Location& loc)
{
    if (set_home(loc)) {
        set_home_state(HOME_SET_AND_LOCKED);
        return true;
    }
    return false;
}

// set_home - sets ahrs home (used for RTL) to specified location
//  initialises inertial nav and compass on first call
//  returns true if home location set successfully
bool Copter::set_home(const Location& loc)
{
    // check location is valid
    if (loc.lat == 0 && loc.lng == 0) {
        return false;
    }

    // set ahrs home (used for RTL)
    ahrs.set_home(loc);

    // init inav and compass declination
    if (ap.home_state == HOME_UNSET) {
        // Set compass declination automatically
        if (g.compass_enabled) {
            compass.set_initial_location(gps.location().lat, gps.location().lng);
        }
        // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
        scaleLongDown = longitude_scale(loc);
        // record home is set
        set_home_state(HOME_SET_NOT_LOCKED);

        // log new home position which mission library will pull from ahrs
        if (should_log(MASK_LOG_CMD)) {
            AP_Mission::Mission_Command temp_cmd;
            if (mission.read_cmd_from_storage(0, temp_cmd)) {
                DataFlash.Log_Write_Mission_Cmd(mission, temp_cmd);
            }
        }
    }

    // log ahrs home and ekf origin dataflash
    Log_Write_Home_And_Origin();

    // send new home location to GCS
    GCS_MAVLINK::send_home_all(loc);

    // return success
    return true;
}

// far_from_EKF_origin - checks if a location is too far from the EKF origin
//  returns true if too far
bool Copter::far_from_EKF_origin(const Location& loc)
{
    // check distance to EKF origin
    const struct Location &ekf_origin = inertial_nav.get_origin();
    if (get_distance(ekf_origin, loc) > EKF_ORIGIN_MAX_DIST_M) {
        return true;
    }

    // close enough to origin
    return false;
}

// checks if we should update ahrs/RTL home position from GPS
void Copter::set_system_time_from_GPS()
{
    // exit immediately if system time already set
    if (ap.system_time_set) {
        return;
    }

    // if we have a 3d lock and valid location
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        // set system clock for log timestamps
        uint64_t gps_timestamp = gps.time_epoch_usec();
                
        hal.util->set_system_clock(gps_timestamp);
                
        // update signing timestamp
        GCS_MAVLINK::update_signing_timestamp(gps_timestamp);

        ap.system_time_set = true;
        Log_Write_Event(DATA_SYSTEM_TIME_SET);
    }
}
