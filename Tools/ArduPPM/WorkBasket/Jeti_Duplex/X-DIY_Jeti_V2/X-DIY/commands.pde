// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

void init_commands()
{
	read_EEPROM_waypoint_info();
	wp_index 			= 0;
	command_must_index	= 0;
	command_may_index	= 0;
	next_command.id 	= CMD_BLANK;
}

void update_auto()
{
	if (wp_index == wp_total){
		return_to_launch();
		//wp_index 			= 0;
	}
}

void reload_commands()
{
	init_commands();
	read_command_index();			// Get wp_index = command_must_index from EEPROM
	if(wp_index > 0){
		decrement_WP_index;
	}
}

// Getters
// -------
struct Location get_wp_with_index(int i)
{
	struct Location temp;
	long mem;

	
	// Find out proper location in memory by using the start_byte position + the index
	// --------------------------------------------------------------------------------
	if (i > wp_total) {
		temp.id = CMD_BLANK;
	}else{
		// read WP position 
		mem = (WP_START_BYTE) + (i * WP_SIZE);
		temp.id = eeprom_read_byte((uint8_t*)mem);
		mem++;
		temp.p1 = eeprom_read_byte((uint8_t*)mem);
		mem++;
		temp.alt = (long)eeprom_read_dword((uint32_t*)mem);
		mem += 4;
		temp.lat = (long)eeprom_read_dword((uint32_t*)mem);
		mem += 4;
		temp.lng = (long)eeprom_read_dword((uint32_t*)mem);
	}
	return temp;
}

// Setters
// -------
void set_wp_with_index(struct Location temp, int i)
{

	i = constrain(i,0,wp_total);
	uint32_t mem = WP_START_BYTE + (i * WP_SIZE);

	eeprom_write_byte((uint8_t *)	mem, temp.id);

	mem++;
	eeprom_write_byte((uint8_t *)	mem, temp.p1);

	mem++;
	eeprom_write_dword((uint32_t *)	mem, temp.alt);

	mem += 4;
	eeprom_write_dword((uint32_t *)	mem, temp.lat);

	mem += 4;
	eeprom_write_dword((uint32_t *)	mem, temp.lng);
}

void increment_WP_index()
{
	if(wp_index < wp_total){
		wp_index++;
		SendDebug("MSG <increment_WP_index> WP index is incremented to ");
		SendDebugln(wp_index,DEC);
	}else{
		SendDebug("MSG <increment_WP_index> Failed to increment WP index of ");
		SendDebugln(wp_index,DEC);
	}
}
void decrement_WP_index()
{
	if(wp_index > 0){
		wp_index--;
	}
}

void loiter_at_location()
{
	next_WP = current_loc;
}

// add a new command at end of command set to RTL.
void return_to_launch(void)
{
	// home is WP 0
	// ------------
	wp_index = 0;
	
	// Loads WP from Memory
	// --------------------
	set_next_WP(&home);

	// Altitude to hold over home
	// Set by configuration tool
	// -------------------------
	next_WP.alt = read_alt_to_hold();
}

struct Location get_LOITER_home_wp()
{
	// read home position 
	struct Location temp = get_wp_with_index(0);	
	temp.id = CMD_LOITER;
	
	temp.alt = read_alt_to_hold();
	return temp;
}

/*
This function stores waypoint commands
It looks to see what the next command type is and finds the last command.
*/
void set_next_WP(struct Location *wp)
{
	//send_message(SEVERITY_LOW,"load WP");
	SendDebug("MSG <set_next_wp> wp_index: ");
	SendDebugln(wp_index,DEC);
	send_message(MSG_COMMAND, wp_index);

	// copy the current WP into the OldWP slot
	// ---------------------------------------
	prev_WP = current_loc;
	// Load the next_WP slot
	// ---------------------
	next_WP = *wp;
	// offset the altitude relative to home position
	// ---------------------------------------------
	next_WP.alt += home.alt;

	// used to control FBW and limit the rate of climb
	// -----------------------------------------------
	target_altitude = current_loc.alt;
	offset_altitude = next_WP.alt - prev_WP.alt;
	
	// zero out our loiter vals to watch for missed waypoints
	loiter_delta 	= 0;
	loiter_sum 		= 0;
	loiter_total 	= 0;

	float rads = (abs(next_WP.lat)/t7) * 0.0174532925;
	//377,173,810 / 10,000,000 = 37.717381 * 0.0174532925 = 0.658292482926943		
	scaleLongDown = cos(rads);
	scaleLongUp = 1.0f/cos(rads);

	// this is handy for the groundstation
	wp_totalDistance 	= getDistance(&current_loc, &next_WP);
	wp_distance 		= wp_totalDistance;
	
	print_current_waypoints();	
	
	target_bearing 		= get_bearing(&current_loc, &next_WP);
	old_target_bearing 	= target_bearing;
	// this is used to offset the shrinking longitude as we go towards the poles	
	
	// set a new crosstrack bearing
	// ----------------------------
	reset_crosstrack();
}


// run this at setup on the ground
// -------------------------------
void init_home()
{
	SendDebugln("MSG: <init_home> init home");

	// Extra read just in case
	// -----------------------
	//GPS.Read();

	// block until we get a good fix
	// -----------------------------
	while (!GPS.new_data || !GPS.fix) {
		GPS.update();
	}
	home.id = CMD_WAYPOINT;
	home.lng = GPS.longitude;				// Lon * 10**7
	home.lat = GPS.latitude;				// Lat * 10**7
	home.alt = GPS.altitude;
	home_is_set = TRUE;

	// ground altitude in centimeters for pressure alt calculations
	// ------------------------------------------------------------
	ground_alt 	= GPS.altitude;
	press_alt 	= GPS.altitude;  // Set initial value for filter
	save_pressure_data();

	// Save Home to EEPROM
	// -------------------
	set_wp_with_index(home, 0);

	// Save prev loc
	// -------------
	prev_WP = home;
	
	// Signal ready to fly
	// Make the servos wiggle - 3 times 
	// -----------------------
	demo_servos(3);
	send_message(SEVERITY_LOW,"\n\n Ready to FLY.");
	//           12345678901234567890123456789012
	jeti_status("     X-DIY        Ready to FLY");
	jeti_update();
}



