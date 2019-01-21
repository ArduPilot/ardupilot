#include "Rover.h"

// checks if we should update ahrs home position from the EKF's position
void Rover::update_home_from_EKF()
{
    // exit immediately if home already set
    if (ahrs.home_is_set()) {
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
        if (!set_home(temp_loc, lock)) {
            return false;
        }
        // we have successfully set AHRS home, set it for SmartRTL
        g2.smart_rtl.set_home(true);
        return true;
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

    const bool home_was_set = ahrs.home_is_set();

    // set ahrs home
    ahrs.set_home(loc);

    if (!home_was_set) {
        // log new home position which mission library will pull from ahrs
        if (should_log(MASK_LOG_CMD)) {
            AP_Mission::Mission_Command temp_cmd;
            if (mode_auto.mission.read_cmd_from_storage(0, temp_cmd)) {
                logger.Write_Mission_Cmd(mode_auto.mission, temp_cmd);
            }
        }
    }

    // lock home position
    if (lock) {
        ahrs.lock_home();
    }

    // Save Home to EEPROM
    mode_auto.mission.write_home_to_storage();

    // send text of home position to ground stations
    gcs().send_text(MAV_SEVERITY_INFO, "Set HOME to %.6f %.6f at %.2fm",
            static_cast<double>(loc.lat * 1.0e-7f),
            static_cast<double>(loc.lng * 1.0e-7f),
            static_cast<double>(loc.alt * 0.01f));

    // return success
    return true;
}

/*
  update home location from GPS
  this is called as long as we have 3D lock and the arming switch is
  not pushed
*/
void Rover::update_home()
{
    if (!ahrs.home_is_locked()) {
        Location loc;
        if (ahrs.get_position(loc)) {
            if (get_distance(loc, ahrs.get_home()) > DISTANCE_HOME_MAX) {
                ahrs.set_home(loc);
            }
        }
    }
    barometer.update_calibration();
}
