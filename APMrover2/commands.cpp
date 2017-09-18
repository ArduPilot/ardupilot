#include "Rover.h"

// checks if we should update ahrs home position from the EKF's position
void Rover::update_home_from_EKF()
{
    // exit immediately if home already set
    if (home_is_set != HOME_UNSET) {
        return;
    }

    // move home to current ekf location (this will set home_state to HOME_SET)
    set_home_to_current_location(false);
}

// set ahrs home to current location from EKF reported location or GPS
bool Rover::set_home_to_current_location(bool lock)
{
    // use position from EKF if available otherwise use GPS
    Location temp_loc;
    if (ahrs.have_inertial_nav() && ahrs.get_position(temp_loc)) {
        return set_home(temp_loc, lock);
    }
    return false;
}

// sets ahrs home to specified location
//  returns true if home location set successfully
bool Rover::set_home(const Location& loc, bool lock)
{
    // check location is valid
    if (loc.lat == 0 && loc.lng == 0 && loc.alt == 0) {
        return false;
    }
    if (!check_latlng(loc)) {
        return false;
    }

    // check if EKF origin has been set
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    // set ahrs home
    ahrs.set_home(loc);

    // init compass declination
    if (home_is_set == HOME_UNSET) {
        // record home is set
        home_is_set = HOME_SET_NOT_LOCKED;

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
        home_is_set = HOME_SET_AND_LOCKED;
    }

    // Save Home to EEPROM
    mission.write_home_to_storage();

    // log ahrs home and ekf origin dataflash
    Log_Write_Home_And_Origin();

    // send new home and ekf origin to GCS
    gcs().send_home(loc);
    gcs().send_ekf_origin(loc);

    // send text of home position to ground stations
    gcs().send_text(MAV_SEVERITY_INFO, "Set HOME to %.6f %.6f at %.2fm",
            static_cast<double>(loc.lat * 1.0e-7f),
            static_cast<double>(loc.lng * 1.0e-7f),
            static_cast<double>(loc.alt * 0.01f));

    // return success
    return true;
}

// sets ekf_origin if it has not been set.
//  should only be used when there is no GPS to provide an absolute position
void Rover::set_ekf_origin(const Location& loc)
{
    // check location is valid
    if (!check_latlng(loc)) {
        return;
    }

    // check if EKF origin has already been set
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

// checks if we should update ahrs/RTL home position from GPS
void Rover::set_system_time_from_GPS()
{
    // exit immediately if system time already set
    if (system_time_set) {
        return;
    }

    // if we have a 3d lock and valid location
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        // set system clock for log timestamps
        const uint64_t gps_timestamp = gps.time_epoch_usec();

        hal.util->set_system_clock(gps_timestamp);

        // update signing timestamp
        GCS_MAVLINK::update_signing_timestamp(gps_timestamp);

        system_time_set = true;
    }
}

/*
  update home location from GPS
  this is called as long as we have 3D lock and the arming switch is
  not pushed
*/
void Rover::update_home()
{
    if (home_is_set == HOME_SET_NOT_LOCKED) {
        Location loc;
        if (ahrs.get_position(loc)) {
            if (get_distance(loc, ahrs.get_home()) > DISTANCE_HOME_MAX) {
                ahrs.set_home(loc);
                Log_Write_Home_And_Origin();
                gcs().send_home(gps.location());
            }
        }
    }
    barometer.update_calibration();
}
