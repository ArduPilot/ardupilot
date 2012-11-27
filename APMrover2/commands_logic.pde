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

		case MAV_CMD_NAV_WAYPOINT:
			return verify_nav_wp();

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			return verify_RTL();

		default:
			gcs_send_text_P(SEVERITY_HIGH,PSTR("verify_nav: Invalid or no current Nav cmd"));
			return false;
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
        prev_WP 		= current_loc;
	control_mode 	= RTL;
	crash_timer 	= 0;
	next_WP 		= home;
}

static void do_takeoff()
{
	set_next_WP(&next_nav_command);
}

static void do_nav_wp()
{
	set_next_WP(&next_nav_command);
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
static bool verify_takeoff()
{  return true;
}

static void calc_turn_radius(void)    // JLN update - adjut automaticaly the wp_radius Vs the speed and the turn angle
{
  wp_radius = ground_speed * 150 / g.roll_limit.get();
  //cliSerial->println(wp_radius, DEC);
}


static bool verify_nav_wp()
{
    update_crosstrack();

    if ((wp_distance > 0) && (wp_distance <= g.waypoint_radius)) {
        gcs_send_text_fmt(PSTR("Reached Waypoint #%i dist %um"),
                          (unsigned)nav_command_index,
                          (unsigned)get_distance(&current_loc, &next_WP));
        return true;
    }

    if(g.auto_wp_radius) { 
        calc_turn_radius();  // JLN update - auto-adap the wp_radius Vs the gspeed and max roll angle

        if ((wp_distance > 0) && (wp_distance <= wp_radius)) {
            gcs_send_text_fmt(PSTR("Reached Waypoint #%i"),nav_command_index);
            return true;
        }
    }

    // have we gone past the waypoint?
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text_fmt(PSTR("Passed Waypoint #%i dist %um"),
                          (unsigned)nav_command_index,
                          (unsigned)get_distance(&current_loc, &next_WP));
        return true;
    }

    return false;
}

static bool verify_RTL()
{
	if (wp_distance <= g.waypoint_radius) {
		gcs_send_text_P(SEVERITY_LOW,PSTR("Reached home"));
                rtl_complete = true;
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
	next_WP.alt 		= condition_value;								// For future nav calculations
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
	if ((uint32_t)(millis() - condition_start) > (uint32_t)condition_value){
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
	if(next_nonnav_command.p1 == 1 && g_gps->status() == GPS::GPS_OK) {
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

