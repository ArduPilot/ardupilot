// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_commands()
{
	// Zero is home, the curren command
    g.command_index = 0;

    // This are registers for the current may and must commands
    // setting to zero will allow them to be written to by new commands
	command_must_index	= NO_COMMAND;
	command_may_index	= NO_COMMAND;

	// clear the command queue
	clear_command_queue();
}

// forces the loading of a new command
// queue is emptied after a new command is processed
static void clear_command_queue(){
	next_command.id 	= NO_COMMAND;
}

// Getters
// -------
static struct Location get_cmd_with_index(int i)
{
	struct Location temp;

	// Find out proper location in memory by using the start_byte position + the index
	// --------------------------------------------------------------------------------
	if (i >= g.command_total) {
		// we do not have a valid command to load
		// return a WP with a "Blank" id
		temp.id = CMD_BLANK;

		// no reason to carry on
		return temp;

	}else{
		// we can load a command, we don't process it yet
		// read WP position
		int32_t mem = (WP_START_BYTE) + (i * WP_SIZE);

		temp.id = eeprom_read_byte((uint8_t*)mem);

		mem++;
		temp.options = eeprom_read_byte((uint8_t*)mem);

		mem++;
		temp.p1 = eeprom_read_byte((uint8_t*)mem);

		mem++;
		temp.alt = eeprom_read_dword((uint32_t*)mem);	// alt is stored in CM! Alt is stored relative!

		mem += 4;
		temp.lat = eeprom_read_dword((uint32_t*)mem); // lat is stored in decimal * 10,000,000

		mem += 4;
		temp.lng = eeprom_read_dword((uint32_t*)mem); // lon is stored in decimal * 10,000,000
	}

	// Add on home altitude if we are a nav command (or other command with altitude) and stored alt is relative
	//if((temp.id < MAV_CMD_NAV_LAST || temp.id == MAV_CMD_CONDITION_CHANGE_ALT) && temp.options & WP_OPTION_ALT_RELATIVE){
		//temp.alt += home.alt;
	//}

	if(temp.options & WP_OPTION_RELATIVE){
		// If were relative, just offset from home
		temp.lat	+=	home.lat;
		temp.lng	+=	home.lng;
	}

	return temp;
}

// Setters
// -------
static void set_command_with_index(struct Location temp, int i)
{
	i = constrain(i, 0, g.command_total.get());
	//Serial.printf("set_command: %d with id: %d\n", i, temp.id);

	// store home as 0 altitude!!!
	// Home is always a MAV_CMD_NAV_WAYPOINT (16)
	if (i == 0){
		temp.alt = 0;
		temp.id = MAV_CMD_NAV_WAYPOINT;
	}

	uint32_t mem = WP_START_BYTE + (i * WP_SIZE);

	eeprom_write_byte((uint8_t *)	mem, temp.id);

	mem++;
	eeprom_write_byte((uint8_t *)	mem, temp.options);

	mem++;
	eeprom_write_byte((uint8_t *)	mem, temp.p1);

	mem++;
	eeprom_write_dword((uint32_t *)	mem, temp.alt);	// Alt is stored in CM!

	mem += 4;
	eeprom_write_dword((uint32_t *)	mem, temp.lat);	// Lat is stored in decimal degrees * 10^7

	mem += 4;
	eeprom_write_dword((uint32_t *)	mem, temp.lng); // Long is stored in decimal degrees * 10^7

	// Make sure our WP_total
	if(g.command_total <= i)
		g.command_total.set_and_save(i+1);
}

static void increment_WP_index()
{
    if (g.command_index < (g.command_total-1)) {
        g.command_index++;
	}

    SendDebugln(g.command_index,DEC);
}

/*
static void decrement_WP_index()
{
    if (g.command_index > 0) {
        g.command_index.set_and_save(g.command_index - 1);
    }
}*/

static int32_t read_alt_to_hold()
{
	if(g.RTL_altitude < 0)
		return current_loc.alt;
	else
		return g.RTL_altitude;// + home.alt;
}


//********************************************************************************
// This function sets the waypoint and modes for Return to Launch
// It's not currently used
//********************************************************************************

/*
This function sets the next waypoint command
It precalculates all the necessary stuff.
*/

static void set_next_WP(struct Location *wp)
{
	//SendDebug("MSG <set_next_wp> wp_index: ");
	//SendDebugln(g.command_index, DEC);

	// copy the current WP into the OldWP slot
	// ---------------------------------------
	prev_WP = next_WP;

	// Load the next_WP slot
	// ---------------------
	next_WP = *wp;

	// used to control and limit the rate of climb - not used right now!
	// -----------------------------------------------------------------
	target_altitude = current_loc.alt;

	// this is used to offset the shrinking longitude as we go towards the poles
	float rads 			= (abs(next_WP.lat)/t7) * 0.0174532925;
	scaleLongDown 		= cos(rads);
	scaleLongUp 		= 1.0f/cos(rads);

	// this is handy for the groundstation
	// -----------------------------------
	wp_totalDistance 	= get_distance(&current_loc, &next_WP);
	wp_distance 		= wp_totalDistance;
	target_bearing 		= get_bearing(&current_loc, &next_WP);

	// to check if we have missed the WP
	// ---------------------------------
	original_target_bearing = target_bearing;

	// reset speed governer
	// --------------------
	waypoint_speed_gov = 0;
}


// run this at setup on the ground
// -------------------------------
static void init_home()
{
	home_is_set = true;

	// block until we get a good fix
	// -----------------------------
	while (!g_gps->new_data || !g_gps->fix) {
		g_gps->update();
        // we need GCS update while waiting for GPS, to ensure
        // we react to HIL mavlink
        gcs_update();
	}

	home.id 	= MAV_CMD_NAV_WAYPOINT;
	home.lng 	= g_gps->longitude;				// Lon * 10**7
	home.lat 	= g_gps->latitude;				// Lat * 10**7
	//home.alt 	= max(g_gps->altitude, 0);		// we sometimes get negatives from GPS, not valid
	home.alt 	= 0;							// Home is always 0

	// to point yaw towards home until we set it with Mavlink
	target_WP 	= home;

	// Save Home to EEPROM
	// -------------------
	// no need to save this to EPROM
	set_command_with_index(home, 0);
	print_wp(&home, 0);

	// Save prev loc this makes the calcs look better before commands are loaded
	prev_WP = home;

	// this is dangerous since we can get GPS lock at any time.
	//next_WP = home;

	// Load home for a default guided_WP
	// -------------
	guided_WP = home;
	guided_WP.alt += g.RTL_altitude;
}



