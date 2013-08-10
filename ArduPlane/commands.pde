// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  logic for dealing with the current command in the mission and home location
 */

static void init_commands()
{
    mission.init_commands();
    non_nav_command_ID      = NO_COMMAND;
}

static void update_auto()
{
    mission.resume();
    if (mission.get_mission_status()) {
        process_waypoint();
    } else {
        handle_no_commands();
        process_waypoint();
    }
}

static int32_t read_alt_to_hold()
{
    if (g.RTL_altitude_cm < 0) {
        return current_loc.alt;
    }
    return g.RTL_altitude_cm + mission.get_home_alt();
}


/*
 *  This function stores waypoint commands
 *  It looks to see what the next command type is and finds the last command.
 */
static void setup_path()
{
    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (location_passed_point(current_loc, mission.prev_wp(), mission.current_wp())) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Resetting prev_WP"));
        mission.override_prev_wp(current_loc);
    }

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    target_altitude_cm = current_loc.alt;

    // zero out our loiter vals to watch for missed waypoints
    loiter_angle_reset();

    setup_glide_slope();

    loiter_angle_reset();
}

static void set_guided_WP(void)
{
    if (g.loiter_radius < 0) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    // Load the next_WP slot
    // ---------------------
    mission.goto_location(guided_WP);

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    target_altitude_cm = current_loc.alt;

    setup_glide_slope();

    loiter_angle_reset();
}

// run this at setup on the ground
// -------------------------------
static void init_home()
{
    struct Location tmp;
    
    gcs_send_text_P(SEVERITY_LOW, PSTR("init home"));

    // block until we get a good fix
    // -----------------------------
    while (!g_gps->new_data || !g_gps->fix) {
        g_gps->update();
#if HIL_MODE != HIL_MODE_DISABLED
        // update hil gps so we have new_data
        gcs_update();
#endif
    }

    tmp.id         = MAV_CMD_NAV_WAYPOINT;
    tmp.lng        = g_gps->longitude;                                 // Lon * 10**7
    tmp.lat        = g_gps->latitude;                                  // Lat * 10**7
    tmp.alt        = max(g_gps->altitude_cm, 0);

    gcs_send_text_fmt(PSTR("gps alt: %lu"), (unsigned long)mission.get_home_alt());

    // Load home for a default guided_WP
    // -------------
    guided_WP = tmp;
    guided_WP.alt += g.RTL_altitude_cm;
    
    // Save Home to Mission
    // -------------------
    mission.set_home(tmp);

}

/*
  update home location from GPS
  this is called as long as we have 3D lock and the arming switch is
  not pushed
*/
static void update_home()
{
    struct Location tmp;

    tmp.lng        = g_gps->longitude;                                 // Lon * 10**7
    tmp.lat        = g_gps->latitude;                                  // Lat * 10**7
    tmp.alt        = max(g_gps->altitude_cm, 0);
    barometer.update_calibration();
    mission.set_home(tmp);
}
