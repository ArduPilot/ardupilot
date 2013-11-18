// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* Functions in this file:
	void init_commands()
	struct Location get_cmd_with_index(int i)
	void set_cmd_with_index(struct Location temp, int i)
	void increment_cmd_index()
	void decrement_cmd_index()
	long read_alt_to_hold()
	void set_next_WP(struct Location *wp)
	void set_guided_WP(void)
	void init_home()
	void restart_nav()
************************************************************ 
*/

static void init_commands()
{
    g.command_index.set_and_save(0);
	nav_command_ID	= NO_COMMAND;
	non_nav_command_ID	= NO_COMMAND;
	next_nav_command.id 	= CMD_BLANK;
}

// Getters
// -------
static struct Location get_cmd_with_index(int i)
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
		temp.id = hal.storage->read_byte(mem);

		mem++;
		temp.options = hal.storage->read_byte(mem);

		mem++;
		temp.p1 = hal.storage->read_byte(mem);

		mem++;
		temp.alt = (long)hal.storage->read_dword(mem);

		mem += 4;
		temp.lat = (long)hal.storage->read_dword(mem);

		mem += 4;
		temp.lng = (long)hal.storage->read_dword(mem);
	}

	// Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
	if((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) && temp.options & MASK_OPTIONS_RELATIVE_ALT){
		temp.alt += home.alt;
	}

	return temp;
}

// Setters
// -------
static void set_cmd_with_index(struct Location temp, int i)
{
	i = constrain_int16(i, 0, g.command_total.get());
	uint16_t mem = WP_START_BYTE + (i * WP_SIZE);

	// Set altitude options bitmask
	// XXX What is this trying to do?
	if ((temp.options & MASK_OPTIONS_RELATIVE_ALT) && i != 0){
		temp.options = MASK_OPTIONS_RELATIVE_ALT;
	} else {
		temp.options = 0;
	}

	hal.storage->write_byte(mem, temp.id);

    mem++;
	hal.storage->write_byte(mem, temp.options);

	mem++;
	hal.storage->write_byte(mem, temp.p1);

	mem++;
	hal.storage->write_dword(mem, temp.alt);

	mem += 4;
	hal.storage->write_dword(mem, temp.lat);

	mem += 4;
	hal.storage->write_dword(mem, temp.lng);
}

/*
This function stores waypoint commands
It looks to see what the next command type is and finds the last command.
*/
static void set_next_WP(const struct Location *wp)
{
	// copy the current WP into the OldWP slot
	// ---------------------------------------
	prev_WP = next_WP;

	// Load the next_WP slot
	// ---------------------
	next_WP = *wp;

    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Resetting prev_WP"));
        prev_WP = current_loc;
    }

	// this is handy for the groundstation
	wp_totalDistance 	= get_distance(current_loc, next_WP);
	wp_distance 		= wp_totalDistance;
}

static void set_guided_WP(void)
{
	// copy the current location into the OldWP slot
	// ---------------------------------------
	prev_WP = current_loc;

	// Load the next_WP slot
	// ---------------------
	next_WP = guided_WP;

	// this is handy for the groundstation
	wp_totalDistance 	= get_distance(current_loc, next_WP);
	wp_distance 		= wp_totalDistance;
}

// run this at setup on the ground
// -------------------------------
void init_home()
{
    if (!have_position) {
        // we need position information
        return;
    }

	gcs_send_text_P(SEVERITY_LOW, PSTR("init home"));

	home.id 	= MAV_CMD_NAV_WAYPOINT;

	home.lng 	= g_gps->longitude;				// Lon * 10**7
	home.lat 	= g_gps->latitude;				// Lat * 10**7
    gps_base_alt    = max(g_gps->altitude_cm, 0);
    home.alt        = g_gps->altitude_cm;
	home_is_set = true;

	// Save Home to EEPROM - Command 0
	// -------------------
	set_cmd_with_index(home, 0);

	// Save prev loc
	// -------------
	next_WP = prev_WP = home;

	// Load home for a default guided_WP
	// -------------
	guided_WP = home;
}

static void restart_nav()
{  
    g.pidSpeedThrottle.reset_I();
    prev_WP = current_loc;
    nav_command_ID = NO_COMMAND;
    nav_command_index = 0;
    process_next_command();
}
