#include "Rover.h"

// set ahrs home to current location from inertial-nav location
bool Rover::set_home_to_current_location(bool lock)
{
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
    const bool home_was_set = ahrs.home_is_set();

    // set ahrs home
    if (!ahrs.set_home(loc)) {
        return false;
    }

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

// called periodically while disarmed to update our home position to
// our current location
void Rover::update_home()
{
    if (ahrs.home_is_locked()) {
        // we've been explicitly told our home location
        return;
    }

    Location loc{};
    if (!ahrs.get_position(loc)) {
        return;
    }

    barometer.update_calibration();

    if (ahrs.home_is_set() &&
        loc.get_distance(ahrs.get_home()) < DISTANCE_HOME_MINCHANGE) {
        // insufficiently moved from current home - don't change it
        return;
    }

    if (!ahrs.set_home(loc)) {
        // silently ignored...
    }
}
