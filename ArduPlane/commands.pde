// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  logic for dealing with the current command in the mission and home location
 */

static void init_commands()
{
    g.command_index.set_and_save(0);
    nav_command_ID  = NO_COMMAND;
    non_nav_command_ID      = NO_COMMAND;
    next_nav_command.id     = CMD_BLANK;
}

static void update_auto()
{
    if (g.command_index >= g.command_total) {
        handle_no_commands();
        if(g.command_total == 0) {
            next_WP.lat             = home.lat + 1000;                  // so we don't have bad calcs
            next_WP.lng             = home.lng + 1000;                  // so we don't have bad calcs
        }
    } else {
        if(g.command_index != 0) {
            g.command_index = nav_command_index;
            nav_command_index--;
        }
        nav_command_ID  = NO_COMMAND;
        non_nav_command_ID      = NO_COMMAND;
        next_nav_command.id     = CMD_BLANK;
        process_next_command();
    }
}

// this is only used by an air-start
static void reload_commands_airstart()
{
    init_commands();
    decrement_cmd_index();
}

/*
  fetch a mission item from EEPROM
*/
static struct Location get_cmd_with_index_raw(int16_t i)
{
    struct Location temp;
    uint16_t mem;

    // Find out proper location in memory by using the start_byte position + the index
    // --------------------------------------------------------------------------------
    if (i > g.command_total) {
        memset(&temp, 0, sizeof(temp));
        temp.id = CMD_BLANK;
    }else{
        // read WP position
        mem = (WP_START_BYTE) + (i * WP_SIZE);
        temp.id = eeprom_read_byte((uint8_t*)(uintptr_t)mem);

        mem++;
        temp.options = eeprom_read_byte((uint8_t*)(uintptr_t)mem);

        mem++;
        temp.p1 = eeprom_read_byte((uint8_t*)(uintptr_t)mem);

        mem++;
        temp.alt = (long)eeprom_read_dword((uint32_t*)(uintptr_t)mem);

        mem += 4;
        temp.lat = (long)eeprom_read_dword((uint32_t*)(uintptr_t)mem);

        mem += 4;
        temp.lng = (long)eeprom_read_dword((uint32_t*)(uintptr_t)mem);
    }

    return temp;
}

/*
  fetch a mission item from EEPROM. Adjust altitude to be absolute
*/
static struct Location get_cmd_with_index(int16_t i)
{
    struct Location temp;

    temp = get_cmd_with_index_raw(i);

    // Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
    if ((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) &&
        (temp.options & MASK_OPTIONS_RELATIVE_ALT) &&
        (temp.lat != 0 || temp.lng != 0)) {
        temp.alt += home.alt;
    }

    return temp;
}

// Setters
// -------
static void set_cmd_with_index(struct Location temp, int16_t i)
{
    i = constrain(i, 0, g.command_total.get());
    intptr_t mem = WP_START_BYTE + (i * WP_SIZE);

    // Set altitude options bitmask
    // XXX What is this trying to do?
    if ((temp.options & MASK_OPTIONS_RELATIVE_ALT) && i != 0) {
        temp.options = MASK_OPTIONS_RELATIVE_ALT;
    } else {
        temp.options = 0;
    }

    eeprom_write_byte((uint8_t *)   mem, temp.id);

    mem++;
    eeprom_write_byte((uint8_t *)   mem, temp.options);

    mem++;
    eeprom_write_byte((uint8_t *)   mem, temp.p1);

    mem++;
    eeprom_write_dword((uint32_t *) mem, temp.alt);

    mem += 4;
    eeprom_write_dword((uint32_t *) mem, temp.lat);

    mem += 4;
    eeprom_write_dword((uint32_t *) mem, temp.lng);
}

static void decrement_cmd_index()
{
    if (g.command_index > 0) {
        g.command_index.set_and_save(g.command_index - 1);
    }
}

static int32_t read_alt_to_hold()
{
    if (g.RTL_altitude_cm < 0) {
        return current_loc.alt;
    }
    return g.RTL_altitude_cm + home.alt;
}


/*
 *  This function stores waypoint commands
 *  It looks to see what the next command type is and finds the last command.
 */
static void set_next_WP(struct Location *wp)
{
    // copy the current WP into the OldWP slot
    // ---------------------------------------
    prev_WP = next_WP;

    // Load the next_WP slot
    // ---------------------
    next_WP = *wp;

    // if lat and lon is zero, then use current lat/lon
    // this allows a mission to contain a "loiter on the spot"
    // command
    if (next_WP.lat == 0 && next_WP.lng == 0) {
        next_WP.lat = current_loc.lat;
        next_WP.lng = current_loc.lng;
        // additionally treat zero altitude as current altitude
        if (next_WP.alt == 0) {
            next_WP.alt = current_loc.alt;
        }
    }


    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Resetting prev_WP"));
        prev_WP = current_loc;
    }

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    target_altitude_cm = current_loc.alt;

    if(prev_WP.id != MAV_CMD_NAV_TAKEOFF && prev_WP.alt != home.alt && (next_WP.id == MAV_CMD_NAV_WAYPOINT || next_WP.id == MAV_CMD_NAV_LAND))
        offset_altitude_cm = next_WP.alt - prev_WP.alt;
    else
        offset_altitude_cm = 0;

    // zero out our loiter vals to watch for missed waypoints
    loiter_delta            = 0;
    loiter_sum                      = 0;
    loiter_total            = 0;

    // this is handy for the groundstation
    wp_totalDistance        = get_distance(&current_loc, &next_WP);
    wp_distance             = wp_totalDistance;
    target_bearing_cd       = get_bearing_cd(&current_loc, &next_WP);
    nav_bearing_cd          = target_bearing_cd;

    // to check if we have missed the WP
    // ----------------------------
    old_target_bearing_cd   = target_bearing_cd;

    // set a new crosstrack bearing
    // ----------------------------
    reset_crosstrack();
}

static void set_guided_WP(void)
{
    // copy the current location into the OldWP slot
    // ---------------------------------------
    prev_WP = current_loc;

    // Load the next_WP slot
    // ---------------------
    next_WP = guided_WP;

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    target_altitude_cm = current_loc.alt;
    offset_altitude_cm = next_WP.alt - prev_WP.alt;

    // this is handy for the groundstation
    wp_totalDistance        = get_distance(&current_loc, &next_WP);
    wp_distance             = wp_totalDistance;
    target_bearing_cd       = get_bearing_cd(&current_loc, &next_WP);

    // to check if we have missed the WP
    // ----------------------------
    old_target_bearing_cd = target_bearing_cd;

    // set a new crosstrack bearing
    // ----------------------------
    reset_crosstrack();
}

// run this at setup on the ground
// -------------------------------
void init_home()
{
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

    home.id         = MAV_CMD_NAV_WAYPOINT;
    home.lng        = g_gps->longitude;                                 // Lon * 10**7
    home.lat        = g_gps->latitude;                                  // Lat * 10**7
    home.alt        = max(g_gps->altitude, 0);
    home_is_set = true;

    gcs_send_text_fmt(PSTR("gps alt: %lu"), (unsigned long)home.alt);

    // Save Home to EEPROM - Command 0
    // -------------------
    set_cmd_with_index(home, 0);

    // Save prev loc
    // -------------
    next_WP = prev_WP = home;

    // Load home for a default guided_WP
    // -------------
    guided_WP = home;
    guided_WP.alt += g.RTL_altitude_cm;

}



