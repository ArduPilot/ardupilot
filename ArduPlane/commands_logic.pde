/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
void
handle_process_must()
{
	// reset navigation integrators
	// -------------------------
	reset_I();

	switch(next_command.id){

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

void
handle_process_may()
{
	switch(next_command.id){

		case MAV_CMD_CONDITION_DELAY:
			do_wait_delay();
			break;

		case MAV_CMD_CONDITION_DISTANCE:
			do_within_distance();
			break;

		case MAV_CMD_CONDITION_CHANGE_ALT:
			do_change_alt();
			break;

	/*	case MAV_CMD_NAV_LAND_OPTIONS:	//    TODO - Add the command or equiv to MAVLink (repair in verify_may() also)
			gcs.send_text_P(SEVERITY_LOW,PSTR("Landing options set"));

			// pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
			landing_pitch 		= next_command.lng * 100;
			g.airspeed_cruise   =  next_command.alt * 100;
			g.throttle_cruise   = next_command.lat;
			landing_distance 	= next_command.p1;
			//landing_roll 	= command.lng;

			SendDebug_P("MSG: throttle_cruise = ");
			SendDebugln(g.throttle_cruise,DEC);
			break;
	*/

		default:
			break;
	}
}

void handle_process_now()
{
	switch(next_command.id){

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
	}
}

void handle_no_commands()
{
	if (command_must_ID)
		return;

	switch (control_mode){
		case LAND:
			// don't get a new command
			break;

		default:
			set_mode(RTL);
			break;
	}
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

bool verify_must()
{
	switch(command_must_ID) {

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
			gcs.send_text_P(SEVERITY_HIGH,PSTR("verify_must: Invalid or no current Nav cmd"));
			return false;
			break;
	}
}

bool verify_may()
{
	switch(command_may_ID) {
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

    default:
        gcs.send_text_P(SEVERITY_HIGH,PSTR("verify_may: Invalid or no current Condition cmd"));
        return false;
        break;
	}
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

void do_RTL(void)
{
	control_mode 	= LOITER;
	crash_timer 	= 0;
	next_WP 		= home;

	// Altitude to hold over home
	// Set by configuration tool
	// -------------------------
	next_WP.alt = read_alt_to_hold();

	// output control mode to the ground station
	gcs.send_message(MSG_HEARTBEAT);

	if (g.log_bitmask & MASK_LOG_MODE)
		Log_Write_Mode(control_mode);
}

void do_takeoff()
{
	set_next_WP(&next_command);
	// pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
	takeoff_pitch 	 	= (int)next_command.p1 * 100;
			//Serial.printf_P(PSTR("TO pitch:"));	Serial.println(takeoff_pitch);
			//Serial.printf_P(PSTR("home.alt:"));	Serial.println(home.alt);
	takeoff_altitude 	= next_command.alt;
			//Serial.printf_P(PSTR("takeoff_altitude:"));	Serial.println(takeoff_altitude);
	next_WP.lat 		= home.lat + 1000;	// so we don't have bad calcs
	next_WP.lng 		= home.lng + 1000;	// so we don't have bad calcs
	takeoff_complete 	= false;			// set flag to use gps ground course during TO.  IMU will be doing yaw drift correction
}

void do_nav_wp()
{
	set_next_WP(&next_command);
}

void do_land()
{
	set_next_WP(&next_command);
}

void do_loiter_unlimited()
{
	set_next_WP(&next_command);
}

void do_loiter_turns()
{
	set_next_WP(&next_command);
	loiter_total = next_command.p1 * 360;
}

void do_loiter_time()
{
	set_next_WP(&next_command);
	loiter_time = millis();
	loiter_time_max = next_command.p1; // units are (seconds * 10)
}

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/
bool verify_takeoff()
{
	if (g_gps->ground_speed > 300){
		if(hold_course == -1){
			// save our current course to take off
			if(g.compass_enabled) {
				hold_course = dcm.yaw_sensor;
			} else {
				hold_course = g_gps->ground_course;
			}
		}
	}

	if(hold_course > -1){
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

bool verify_land()
{
	// we don't verify landing - we never go to a new Must command after Land
	if (((wp_distance > 0) && (wp_distance <= (2*g_gps->ground_speed/100)))
		|| (current_loc.alt <= next_WP.alt + 300)){

		land_complete = 1;		//Set land_complete if we are within 2 seconds distance or within 3 meters altitude

		if(hold_course == -1){
			// save our current course to land
			//hold_course = yaw_sensor;
			// save the course line of the runway to land
			hold_course = crosstrack_bearing;
		}
	}

	if(hold_course > -1){
		// recalc bearing error with hold_course;
		nav_bearing = hold_course;
		// recalc bearing error
		calc_bearing_error();
	}

	update_crosstrack();
	return false;
}

bool verify_nav_wp()
{
	hold_course = -1;
	update_crosstrack();
	if ((wp_distance > 0) && (wp_distance <= g.waypoint_radius)) {
		//SendDebug_P("MSG <verify_must: MAV_CMD_NAV_WAYPOINT> REACHED_WAYPOINT #");
		//SendDebugln(command_must_index,DEC);
		char message[30];
		sprintf(message,"Reached Waypoint #%i",command_must_index);
		gcs.send_text(SEVERITY_LOW,message);
		return true;
	}
	// add in a more complex case
	// Doug to do
	if(loiter_sum > 300){
		gcs.send_text_P(SEVERITY_MEDIUM,PSTR("<verify_must: MAV_CMD_NAV_WAYPOINT> Missed WP"));
		return true;
	}
	return false;
}

bool verify_loiter_unlim()
{
	update_loiter();
	calc_bearing_error();
	return false;
}

bool verify_loiter_time()
{
	update_loiter();
	calc_bearing_error();
	if ((millis() - loiter_time) > (long)loiter_time_max * 10000l) {		// scale loiter_time_max from (sec*10) to milliseconds
		gcs.send_text_P(SEVERITY_LOW,PSTR("verify_must: LOITER time complete"));
		return true;
	}
	return false;
}

bool verify_loiter_turns()
{
	update_loiter();
	calc_bearing_error();
	if(loiter_sum > loiter_total) {
		loiter_total = 0;
		gcs.send_text_P(SEVERITY_LOW,PSTR("verify_must: LOITER orbits complete"));
		// clear the command queue;
		return true;
	}
	return false;
}

bool verify_RTL()
{
	if (wp_distance <= g.waypoint_radius) {
		gcs.send_text_P(SEVERITY_LOW,PSTR("Reached home"));
		return true;
	}else{
		return false;
	}
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

void do_wait_delay()
{
	condition_start = millis();
	condition_value  = next_command.lat * 1000;	// convert to milliseconds
}

void do_change_alt()
{
	condition_rate		= next_command.p1;
	condition_value 	= next_command.alt;
	target_altitude		= current_loc.alt + (condition_rate / 10);		// Divide by ten for 10Hz update
	next_WP.alt 		= condition_value;								// For future nav calculations
	offset_altitude 	= 0;											// For future nav calculations
}

void do_within_distance()
{
	condition_value  = next_command.lat;
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

bool verify_wait_delay()
{
	if ((millis() - condition_start) > condition_value){
		condition_value 	= 0;
		return true;
	}
	return false;
}

bool verify_change_alt()
{
	//XXX this doesn't look right. How do you descend?
	if(current_loc.alt >= condition_value) {
		//Serial.printf_P(PSTR("alt, top:"));
		//Serial.print(current_loc.alt, DEC);
		//Serial.printf_P(PSTR("\t"));
		//Serial.println(condition_value, DEC);
		condition_value = 0;
		return true;
	}
	target_altitude += condition_rate / 10;
	return false;
}

bool verify_within_distance()
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

void do_loiter_at_location()
{
	next_WP = current_loc;
}

void do_jump()
{
	struct Location temp;
	if(next_command.lat > 0) {

		command_must_index 	= 0;
		command_may_index 	= 0;
		temp 				= get_wp_with_index(g.waypoint_index);
		temp.lat 			= next_command.lat - 1;					// Decrement repeat counter

		set_wp_with_index(temp, g.waypoint_index);
		g.waypoint_index.set_and_save(next_command.p1 - 1);
	}
}

void do_change_speed()
{
	// Note: we have no implementation for commanded ground speed, only air speed and throttle
	if(next_command.alt > 0)
		g.airspeed_cruise.set_and_save(next_command.alt * 100);

	if(next_command.lat > 0)
		g.throttle_cruise.set_and_save(next_command.lat * 100);
}

void do_set_home()
{
	if(next_command.p1 == 1 && GPS_enabled) {
		init_home();
	} else {
		home.id 	= MAV_CMD_NAV_WAYPOINT;
		home.lng 	= next_command.lng;				// Lon * 10**7
		home.lat 	= next_command.lat;				// Lat * 10**7
		home.alt 	= max(next_command.alt, 0);
		home_is_set = true;
	}
}

void do_set_servo()
{
	APM_RC.OutputCh(next_command.p1 - 1, next_command.alt);
}

void do_set_relay()
{
	if (next_command.p1 == 1) {
		relay_on();
	} else if (next_command.p1 == 0) {
		relay_off();
	}else{
		relay_toggle();
	}
}

void do_repeat_servo()
{
	event_id = next_command.p1 - 1;

	if(next_command.p1 >= CH_5 + 1 && next_command.p1 <= CH_8 + 1) {

		event_timer 	= 0;
		event_delay 	= next_command.lng * 500.0;	// /2 (half cycle time) * 1000 (convert to milliseconds)
		event_repeat 	= next_command.lat * 2;
		event_value 	= next_command.alt;

		switch(next_command.p1) {
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

void do_repeat_relay()
{
	event_id 		= RELAY_TOGGLE;
	event_timer 	= 0;
	event_delay 	= next_command.lat * 500.0;	// /2 (half cycle time) * 1000 (convert to milliseconds)
	event_repeat	= next_command.alt * 2;
	update_events();
}
