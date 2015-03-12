// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * the home_state has a number of possible values (see enum HomeState in defines.h's)
 *   HOME_UNSET             = home is not set, no GPS positions yet received
 *   HOME_SET_NOT_LOCKED    = home is set to EKF origin or armed location (can be moved)
 *   HOME_SET_AND_LOCKED    = home has been set by user, cannot be moved except by user initiated do-set-home command
 */

static uint8_t ground_start_count = 10;     // counter used to grab at least 10 reads before accepting a GPS location as home location

// checks if we should update ahrs/RTL home position from GPS
static void update_home_from_GPS()
{
    // exit immediately if counter has run down and home already set
    if (ground_start_count == 0 && ap.home_state != HOME_UNSET) {
        return;
    }

    // if counter has not run down
    if (ground_start_count > 0) {

        // reset counter if we do not have GPS lock
        if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
            ground_start_count = 10;

        // count down for 10 consecutive locks
        } else {
           ground_start_count--;
        }

        return;
    }

    // move home to current gps location (this will set home_state to HOME_SET)
    set_home(gps.location());
}

// set_home_to_current_location - set home to current GPS location
static bool set_home_to_current_location() {
    // exit with failure if we haven't had 10 good GPS position
    if (ground_start_count > 0) {
        return false;
    }

    // set home to latest gps location
    return set_home(gps.location());
}

// set_home_to_current_location_and_lock - set home to current location and lock so it cannot be moved
static bool set_home_to_current_location_and_lock()
{
    if (set_home_to_current_location()) {
        set_home_state(HOME_SET_AND_LOCKED);
        return true;
    }
    return false;
}

// set_home_and_lock - sets ahrs home (used for RTL) to specified location and locks so it cannot be moved
//  unless this function is called again
static bool set_home_and_lock(const Location& loc)
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
static bool set_home(const Location& loc)
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
        scaleLongUp   = 1.0f/scaleLongDown;
        // record home is set
        set_home_state(HOME_SET_NOT_LOCKED);
    }

    // To-Do: doing the stuff below constantly while armed could lead to lots of logging or performance hit?

    // log new home position which mission library will pull from ahrs
    if (should_log(MASK_LOG_CMD)) {
        AP_Mission::Mission_Command temp_cmd;
        if (mission.read_cmd_from_storage(0, temp_cmd)) {
            Log_Write_Cmd(temp_cmd);
        }
    }

    // return success
    return true;
}

// far_from_EKF_origin - checks if a location is too far from the EKF origin
//  returns true if too far
static bool far_from_EKF_origin(const Location& loc)
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
static void set_system_time_from_GPS()
{
    // exit immediately if system time already set
    if (ap.system_time_set) {
        return;
    }

    // if we have a 3d lock and valid location
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        // set system clock for log timestamps
        hal.util->set_system_clock(gps.time_epoch_usec());
        ap.system_time_set = true;
        Log_Write_Event(DATA_SYSTEM_TIME_SET);
    }
}

// check_gps_base_pos - sets gps base position (used for RTK only)
static void check_gps_base_pos()
{
    if (!ap.gps_base_pos_set && !motors.armed() && home_is_set()) {
        // if we're ready to enter RTK mode, then capture current state as home,
        // and enter RTK fixes
        if (gps.can_calculate_base_pos()) {
            gps.calculate_base_pos();
        }
        ap.gps_base_pos_set = true;
    }
}
