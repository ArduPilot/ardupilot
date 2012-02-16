/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
static void
handle_process_nav_cmd()
{
	// reset navigation integrators
	// -------------------------
	reset_I();

		gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nav_command.id);
	switch(next_nav_command.id){

		case MAV_CMD_NAV_TAKEOFF:
			do_takeoff();
			break;

		case MAV_CMD_NAV_WAYPOINT:	// Navigate to Waypoint
			do_nav_wp();
			break;

		case MAV_CMD_NAV_LAND:	// LAND to Waypoint
			do_land();
			break;

		case MAV_CMD_NAV_LOITER_UNLIM:	// Loiter indefinitely
			do_loiter_unlimited();
			break;

		case MAV_CMD_NAV_LOITER_TURNS:	// Loiter N Times
			do_loiter_turns();
			break;

		case MAV_CMD_NAV_LOITER_TIME:
			do_loiter_time();
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			do_RTL();
			break;

		default:
			break;
	}
}

static void
handle_process_condition_command()
{
	gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nonnav_command.id);
	switch(next_nonnav_command.id){

		case MAV_CMD_CONDITION_DELAY:
			do_wait_delay();
			break;

		case MAV_CMD_CONDITION_DISTANCE:
			do_within_distance();
			break;

		case MAV_CMD_CONDITION_CHANGE_ALT:
			do_change_alt();
			break;

		default:
			break;
	}
}

static void handle_process_do_command()
{
	gcs_send_text_fmt(PSTR("Executing command ID #%i"),next_nonnav_command.id);
	switch(next_nonnav_command.id){

		case MAV_CMD_DO_JUMP:
			do_jump();
			break;

		case MAV_CMD_DO_CHANGE_SPEED:
			do_change_speed();
			break;

		case MAV_CMD_DO_SET_HOME:
			do_set_home();
			break;

		case MAV_CMD_DO_SET_SERVO:
			do_set_servo();
			break;

		case MAV_CMD_DO_SET_RELAY:
			do_set_relay();
			break;

		case MAV_CMD_DO_REPEAT_SERVO:
			do_repeat_servo();
			break;

		case MAV_CMD_DO_REPEAT_RELAY:
			do_repeat_relay();
			break;

#if MOUNT == ENABLED
		// Sets the region of interest (ROI) for a sensor set or the
		// vehicle itself. This can then be used by the vehicles control
		// system to control the vehicle attitude and the attitude of various
		// devices such as cameras.
		//    |Region of interest mode. (see MAV_ROI enum)| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple cameras etc.)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
		case MAV_CMD_DO_SET_ROI:
			camera_mount.set_roi_cmd();
			break;

		case MAV_CMD_DO_MOUNT_CONFIGURE:	// Mission command to configure a camera mount |Mount operation mode (see MAV_CONFIGURE_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
			camera_mount.configure_cmd();
			break;

		case MAV_CMD_DO_MOUNT_CONTROL:		// Mission command to control a camera mount |pitch(deg*100) or lat, depending on mount mode.| roll(deg*100) or lon depending on mount mode| yaw(deg*100) or alt (in cm) depending on mount mode| Empty| Empty| Empty| Empty|
			camera_mount.control_cmd();
			break;
#endif
	}
}

static void handle_no_commands()
{
	gcs_send_text_fmt(PSTR("Returning to Home"));
	next_nav_command = home;
	next_nav_command.alt = read_alt_to_hold();
	next_nav_command.id = MAV_CMD_NAV_LOITER_UNLIM;
	nav_command_ID = MAV_CMD_NAV_LOITER_UNLIM;
	non_nav_command_ID = WAIT_COMMAND;
	handle_process_nav_cmd();
	
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

static bool verify_nav_command()	// Returns true if command complete
{
	switch(nav_command_ID) {

		case MAV_CMD_NAV_TAKEOFF:
			return verify_takeoff();
			break;

		case MAV_CMD_NAV_LAND:
			return verify_land();
			break;

		case MAV_CMD_NAV_WAYPOINT:
			return verify_nav_wp();
			break;

		case MAV_CMD_NAV_LOITER_UNLIM:
			return verify_loiter_unlim();
			break;

		case MAV_CMD_NAV_LOITER_TURNS:
			return verify_loiter_turns();
			break;

		case MAV_CMD_NAV_LOITER_TIME:
			return verify_loiter_time();
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			return verify_RTL();
			break;

		default:
			gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_nav: Invalid or no current Nav cmd"));
			return false;
			break;
	}
}

static bool verify_condition_command()		// Returns true if command complete
{
	switch(non_nav_command_ID) {
    case NO_COMMAND:
        break;

    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return verify_change_alt();
        break;
        
    case WAIT_COMMAND:
        return 0;
        break;
        

    default:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_conditon: Invalid or no current Condition cmd"));
        break;
	}
    return false;
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

static void do_RTL(void)
{
	control_mode 	= RTL;
	crash_timer 	= 0;
	next_WP 		= home;

	// Altitude to hold over home
	// Set by configuration tool
	// -------------------------
	next_WP.alt = read_alt_to_hold();

	if (g.log_bitmask & MASK_LOG_MODE)
		Log_Write_Mode(control_mode);
}

static void do_takeoff()
{
	set_next_WP(&next_nav_command);
	// pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
	takeoff_pitch 	 	= (int)next_nav_command.p1 * 100;
			//Serial.printf_P(PSTR("TO pitch:"));	Serial.println(takeoff_pitch);
			//Serial.printf_P(PSTR("home.alt:"));	Serial.println(home.alt);
	takeoff_altitude 	= next_nav_command.alt;
			//Serial.printf_P(PSTR("takeoff_altitude:"));	Serial.println(takeoff_altitude);
	next_WP.lat 		= home.lat + 1000;	// so we don't have bad calcs
	next_WP.lng 		= home.lng + 1000;	// so we don't have bad calcs
	takeoff_complete 	= false;			// set flag to use gps ground course during TO.  IMU will be doing yaw drift correction
											// Flag also used to override "on the ground" throttle disable
}

static void do_nav_wp()
{
	set_next_WP(&next_nav_command);
}

static void do_land()
{
	set_next_WP(&next_nav_command);
}

static void do_loiter_unlimited()
{
	set_next_WP(&next_nav_command);
}

static void do_loiter_turns()
{
	set_next_WP(&next_nav_command);
	loiter_total = next_nav_command.p1 * 360;
}

static void do_loiter_time()
{
	set_next_WP(&next_nav_command);
	loiter_time = millis();
	loiter_time_max = next_nav_command.p1; // units are (seconds * 10)
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
static bool verify_takeoff()
{
	if (g_gps->ground_speed > 300){
		if (hold_course == -1) {
			// save our current course to take off
			if(g.compass_enabled) {
				hold_course = dcm.yaw_sensor;
			} else {
				hold_course = g_gps->ground_course;
			}
		}
	}

	if (hold_course != -1) {
		// recalc bearing error with hold_course;
		nav_bearing = hold_course;
		// recalc bearing error
		calc_bearing_error();
	}

	if (current_loc.alt > takeoff_altitude)  {
		hold_course = -1;
		takeoff_complete = true;
		return true;
	} else {
		return false;
	}
}

// we are executing a landing
static bool verify_land()
{
	// we don't 'verify' landing in the sense that it never completes,
	// so we don't verify command completion. Instead we use this to
	// adjust final landing parameters

    // Set land_complete if we are within 2 seconds distance or within
    // 3 meters altitude of the landing point
	if (((wp_distance > 0) && (wp_distance <= (2*g_gps->ground_speed/100)))
		|| (current_loc.alt <= next_WP.alt + 300)){

		land_complete = 1;

		if(hold_course == -1) {
            // we have just reached the threshold of 2 seconds or 3
			// meters to landing. We now don't want to do any radical
			// turns, as rolling could put the wings into the runway.
			// To prevent further turns we set hold_course to the
			// current heading. Previously we set this to
			// crosstrack_bearing, but the xtrack bearing can easily
			// be quite large at this point, and that could induce a
			// sudden large roll correction which is very nasty at
			// this point in the landing.
			hold_course = dcm.yaw_sensor;
		}
	}

	if (hold_course != -1){
		// recalc bearing error with hold_course;
		nav_bearing = hold_course;
		// recalc bearing error
		calc_bearing_error();
	}

	update_crosstrack();
	return false;
}

static bool verify_nav_wp()
{
	hold_course = -1;
	update_crosstrack();
	if ((wp_distance > 0) && (wp_distance <= g.waypoint_radius)) {
		gcs_send_text_fmt(PSTR("Reached Waypoint #%i"),nav_command_index);
		return true;
	}
	// add in a more complex case
	// Doug to do
	if(loiter_sum > 300){
		gcs_send_text_P(SEVERITY_MEDIUM,PSTR("Missed WP"));
		return true;
	}
	return false;
}

static bool verify_loiter_unlim()
{
	update_loiter();
	calc_bearing_error();
	return false;
}

static bool verify_loiter_time()
{
	update_loiter();
	calc_bearing_error();
	if ((millis() - loiter_time) > (unsigned long)loiter_time_max * 10000l) {		// scale loiter_time_max from (sec*10) to milliseconds
		gcs_send_text_P(SEVERITY_LOW,PSTR("verify_nav: LOITER time complete"));
		return true;
	}
	return false;
}

static bool verify_loiter_turns()
{
	update_loiter();
	calc_bearing_error();
	if(loiter_sum > loiter_total) {
		loiter_total = 0;
		gcs_send_text_P(SEVERITY_LOW,PSTR("verify_nav: LOITER orbits complete"));
		// clear the command queue;
		return true;
	}
	return false;
}

static bool verify_RTL()
{
	if (wp_distance <= g.waypoint_radius) {
		gcs_send_text_P(SEVERITY_LOW,PSTR("Reached home"));
		return true;
	}else{
		return false;
	}
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

static void do_wait_delay()
{
	condition_start = millis();
	condition_value  = next_nonnav_command.lat * 1000;	// convert to milliseconds
}

static void do_change_alt()
{
	condition_rate		= abs((int)next_nonnav_command.lat);
	condition_value 	= next_nonnav_command.alt;
	if(condition_value < current_loc.alt) condition_rate = -condition_rate;
	target_altitude		= current_loc.alt + (condition_rate / 10);		// Divide by ten for 10Hz update
	next_WP.alt 		= condition_value;								// For future nav calculations
	offset_altitude 	= 0;											// For future nav calculations
}

static void do_within_distance()
{
	condition_value  = next_nonnav_command.lat;
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
	if ((unsigned)(millis() - condition_start) > condition_value){
		condition_value 	= 0;
		return true;
	}
	return false;
}

static bool verify_change_alt()
{
	if( (condition_rate>=0 && current_loc.alt >= condition_value) || (condition_rate<=0 && current_loc.alt <= condition_value)) {
		condition_value = 0;
		return true;
	}
	target_altitude += condition_rate / 10;
	return false;
}

static bool verify_within_distance()
{
	if (wp_distance < condition_value){
		condition_value = 0;
		return true;
	}
	return false;
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

static void do_loiter_at_location()
{
	next_WP = current_loc;
}

static void do_jump()
{
	struct Location temp;
	gcs_send_text_fmt(PSTR("In jump.  Jumps left: %i"),next_nonnav_command.lat);
	if(next_nonnav_command.lat > 0) {

		nav_command_ID		= NO_COMMAND;
		next_nav_command.id = NO_COMMAND;
		non_nav_command_ID 	= NO_COMMAND;
		
		temp 				= get_cmd_with_index(g.command_index);
		temp.lat 			= next_nonnav_command.lat - 1;					// Decrement repeat counter

		set_cmd_with_index(temp, g.command_index);
	gcs_send_text_fmt(PSTR("setting command index: %i"),next_nonnav_command.p1 - 1);
		g.command_index.set_and_save(next_nonnav_command.p1 - 1);
		nav_command_index 	= next_nonnav_command.p1 - 1;
		next_WP = prev_WP;		// Need to back "next_WP" up as it was set to the next waypoint following the jump
		process_next_command();
	} else if (next_nonnav_command.lat == -1) {								// A repeat count of -1 = repeat forever
		nav_command_ID 	= NO_COMMAND;
		non_nav_command_ID 	= NO_COMMAND;
	gcs_send_text_fmt(PSTR("setting command index: %i"),next_nonnav_command.p1 - 1);
	    g.command_index.set_and_save(next_nonnav_command.p1 - 1);
		nav_command_index 	= next_nonnav_command.p1 - 1;
		next_WP = prev_WP;		// Need to back "next_WP" up as it was set to the next waypoint following the jump
		process_next_command();
	}
}

static void do_change_speed()
{
	switch (next_nonnav_command.p1)
	{
		case 0: // Airspeed
			if(next_nonnav_command.alt > 0)
				g.airspeed_cruise.set(next_nonnav_command.alt * 100);
			break;
		case 1: // Ground speed
			g.min_gndspeed.set(next_nonnav_command.alt * 100);
			break;
	}

	if(next_nonnav_command.lat > 0)
		g.throttle_cruise.set(next_nonnav_command.lat);
}

static void do_set_home()
{
	if(next_nonnav_command.p1 == 1 && GPS_enabled) {
		init_home();
	} else {
		home.id 	= MAV_CMD_NAV_WAYPOINT;
		home.lng 	= next_nonnav_command.lng;				// Lon * 10**7
		home.lat 	= next_nonnav_command.lat;				// Lat * 10**7
		home.alt 	= max(next_nonnav_command.alt, 0);
		home_is_set = true;
	}
}

static void do_set_servo()
{
	APM_RC.OutputCh(next_nonnav_command.p1 - 1, next_nonnav_command.alt);
}

static void do_set_relay()
{
	if (next_nonnav_command.p1 == 1) {
		relay.on();
	} else if (next_nonnav_command.p1 == 0) {
		relay.off();
	}else{
		relay.toggle();
	}
}

static void do_repeat_servo()
{
	event_id = next_nonnav_command.p1 - 1;

	if(next_nonnav_command.p1 >= CH_5 + 1 && next_nonnav_command.p1 <= CH_8 + 1) {

		event_timer 	= 0;
		event_delay 	= next_nonnav_command.lng * 500.0;	// /2 (half cycle time) * 1000 (convert to milliseconds)
		event_repeat 	= next_nonnav_command.lat * 2;
		event_value 	= next_nonnav_command.alt;

		switch(next_nonnav_command.p1) {
			case CH_5:
				event_undo_value = g.rc_5.radio_trim;
				break;
			case CH_6:
				event_undo_value = g.rc_6.radio_trim;
				break;
			case CH_7:
				event_undo_value = g.rc_7.radio_trim;
				break;
			case CH_8:
				event_undo_value = g.rc_8.radio_trim;
				break;
		}
		update_events();
	}
}

static void do_repeat_relay()
{
	event_id 		= RELAY_TOGGLE;
	event_timer 	= 0;
	event_delay 	= next_nonnav_command.lat * 500.0;	// /2 (half cycle time) * 1000 (convert to milliseconds)
	event_repeat	= next_nonnav_command.alt * 2;
	update_events();
}
