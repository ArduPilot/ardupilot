#include "Copter.h"

// checks if we should update ahrs/RTL home position from the EKF
void Copter::update_home_from_EKF()
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
        set_home_to_current_location(false);
    }
}

// set_home_to_current_location_inflight - set home to current GPS location (horizontally) and EKF origin vertically
void Copter::set_home_to_current_location_inflight() {
    // get current location from EKF
    Location temp_loc;
    if (inertial_nav.get_location(temp_loc)) {
        const struct Location &ekf_origin = inertial_nav.get_origin();
        temp_loc.alt = ekf_origin.alt;
        if (!set_home(temp_loc, false)) {
            return;
        }
        // we have successfully set AHRS home, set it for SmartRTL
#if MODE_SMARTRTL_ENABLED == ENABLED
        g2.smart_rtl.set_home(true);
#endif
    }
}

// set_home_to_current_location - set home to current GPS location
bool Copter::set_home_to_current_location(bool lock) {
    // get current location from EKF
    Location temp_loc;
    if (inertial_nav.get_location(temp_loc)) {
        if (!set_home(temp_loc, lock)) {
            return false;
        }
        // we have successfully set AHRS home, set it for SmartRTL
#if MODE_SMARTRTL_ENABLED == ENABLED
        g2.smart_rtl.set_home(true);
#endif
        return true;
    }
    return false;
}

// set_home - sets ahrs home (used for RTL) to specified location
//  initialises inertial nav and compass on first call
//  returns true if home location set successfully
bool Copter::set_home(const Location& loc, bool lock)
{
    // check location is valid
    if (loc.lat == 0 && loc.lng == 0) {
        return false;
    }

    // check EKF origin has been set
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    // check home is close to EKF origin
    if (far_from_EKF_origin(loc)) {
        return false;
    }

    // set ahrs home (used for RTL)
    ahrs.set_home(loc);

    // init inav and compass declination
    if (!ahrs.home_is_set()) {
        // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
        scaleLongDown = longitude_scale(loc);
        // record home is set
        ahrs.set_home_status(HOME_SET_NOT_LOCKED);
        Log_Write_Event(DATA_SET_HOME);

#if MODE_AUTO_ENABLED == ENABLED
        // log new home position which mission library will pull from ahrs
        if (should_log(MASK_LOG_CMD)) {
            AP_Mission::Mission_Command temp_cmd;
            if (mission.read_cmd_from_storage(0, temp_cmd)) {
                DataFlash.Log_Write_Mission_Cmd(mission, temp_cmd);
            }
        }
#endif
    }

    // lock home position
    if (lock) {
        ahrs.set_home_status(HOME_SET_AND_LOCKED);
    }

    // log ahrs home and ekf origin dataflash
    Log_Write_Home_And_Origin();

    // send new home and ekf origin to GCS
    gcs().send_home(loc);
    gcs().send_ekf_origin(ekf_origin);

    // return success
    return true;
}

// sets ekf_origin if it has not been set.
//  should only be used when there is no GPS to provide an absolute position
void Copter::set_ekf_origin(const Location& loc)
{
    // check location is valid
    if (!check_latlng(loc)) {
        return;
    }

    // check EKF origin has already been set
    Location ekf_origin;
    if (ahrs.get_origin(ekf_origin)) {
        return;
    }

    if (!ahrs.set_origin(loc)) {
        return;
    }

    // log ahrs home and ekf origin dataflash
    Log_Write_Home_And_Origin();

    // send ekf origin to GCS
    gcs().send_ekf_origin(loc);
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
