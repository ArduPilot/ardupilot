#include "Rover.h"
/*
 *  set_auto_WP - sets the target location the vehicle should drive to in Auto mode
 */
void Rover::set_auto_WP(const struct Location& loc)
{
    // copy the current WP into the OldWP slot
    // ---------------------------------------
    prev_WP = next_WP;

    // Load the next_WP slot
    // ---------------------
    next_WP = loc;

    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Resetting previous WP");
        prev_WP = current_loc;
    }

    // this is handy for the groundstation
    wp_totalDistance = get_distance(current_loc, next_WP);
    wp_distance      = wp_totalDistance;
}

void Rover::set_guided_WP(const struct Location& loc)
{
    guided_mode = Guided_WP;
    // copy the current location into the OldWP slot
    // ---------------------------------------
    prev_WP = current_loc;

    // Load the next_WP slot
    // ---------------------
    next_WP = loc;
    rover.guided_control.target_speed = g.speed_cruise;
    // this is handy for the groundstation
    wp_totalDistance = get_distance(current_loc, next_WP);
    wp_distance      = wp_totalDistance;

    rover.rtl_complete = false;
}

void Rover::set_guided_velocity(float target_steer_speed, float target_speed)
{
    guided_mode = Guided_Velocity;
    rover.guided_control.target_steer_speed = target_steer_speed;
    rover.guided_control.target_speed = target_speed;

    next_WP = current_loc;
    lateral_acceleration = 0.0f;
    // this is handy for the groundstation
    wp_totalDistance = 0;
    wp_distance      = 0.0f;

    rover.rtl_complete = false;
}

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
    if (ahrs.get_position(temp_loc)) {
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

    // set EKF origin to home if it hasn't been set yet
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        ahrs.set_origin(loc);
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

        // initialise navigation to home
        next_WP = prev_WP = home;

        // Load home for a default guided_WP
        set_guided_WP(home);
    }

    // lock home position
    if (lock) {
        home_is_set = HOME_SET_AND_LOCKED;
    }

    // Save Home to EEPROM
    mission.write_home_to_storage();

    // log ahrs home and ekf origin dataflash
    Log_Write_Home_And_Origin();

    // send new home location to GCS
    gcs().send_home(loc);

    // send text of home position to ground stations
    gcs().send_text(MAV_SEVERITY_INFO, "Set HOME to %.6f %.6f at %.2fm",
            static_cast<double>(loc.lat * 1.0e-7f),
            static_cast<double>(loc.lng * 1.0e-7f),
            static_cast<double>(loc.alt * 0.01f));

    // return success
    return true;
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

void Rover::restart_nav()
{
    g.pidSpeedThrottle.reset_I();
    prev_WP = current_loc;
    mission.start_or_resume();
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
