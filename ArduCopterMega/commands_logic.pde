/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
static void handle_process_must()
{
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
			//do_loiter_turns();
			break;

		case MAV_CMD_NAV_LOITER_TIME:  // 19
			do_loiter_time();
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			do_RTL();
			break;

		default:
			break;
	}

}

static void handle_process_may()
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

		case MAV_CMD_CONDITION_YAW:
			do_yaw();
			break;

		default:
			break;
	}
}

static void handle_process_now()
{
	switch(next_command.id){

		case MAV_CMD_DO_JUMP:
			do_jump();
			break;

		case MAV_CMD_DO_CHANGE_SPEED:
			//do_change_speed();
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

		case MAV_CMD_DO_SET_ROI:
			do_target_yaw();
	}
}

static void handle_no_commands()
{
	// we don't want to RTL yet. Maybe this will change in the future. RTL is kinda dangerous.
	// use landing commands
	/*
	switch (control_mode){
		default:
			//set_mode(RTL);
			break;
	}
	return;
	*/
	Serial.println("Handle No CMDs");
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

static bool verify_must()
{
	//Serial.printf("vmust: %d\n", command_must_ID);

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
			return false;
			break;

		case MAV_CMD_NAV_LOITER_TURNS:
			return true;
			break;

		case MAV_CMD_NAV_LOITER_TIME:
			return verify_loiter_time();
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			return verify_RTL();
			break;

		default:
			//gcs.send_text_P(SEVERITY_HIGH,PSTR("<verify_must: default> No current Must commands"));
			return false;
			break;
	}
}

static bool verify_may()
{
	switch(command_may_ID) {

		case MAV_CMD_CONDITION_DELAY:
			return verify_wait_delay();
			break;

		case MAV_CMD_CONDITION_DISTANCE:
			return verify_within_distance();
			break;

		case MAV_CMD_CONDITION_CHANGE_ALT:
			return verify_change_alt();
			break;

		case MAV_CMD_CONDITION_YAW:
			return verify_yaw();
			break;

		default:
			//gcs.send_text_P(SEVERITY_HIGH,PSTR("<verify_must: default> No current May commands"));
			return false;
			break;
	}
}

/********************************************************************************/
//
/********************************************************************************/

static void do_RTL(void)
{
	Location temp	= home;
	temp.alt		= read_alt_to_hold();

	//so we know where we are navigating from
	// --------------------------------------
	next_WP = current_loc;

	// Loads WP from Memory
	// --------------------
	set_next_WP(&temp);

	// output control mode to the ground station
	// -----------------------------------------
	gcs.send_message(MSG_HEARTBEAT);
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

static void do_takeoff()
{
	wp_control = LOITER_MODE;

	// Start with current location
	Location temp = current_loc;

	// next_command.alt is a relative altitude!!!
	if (next_command.options & WP_OPTION_ALT_RELATIVE) {
		temp.alt = next_command.alt + home.alt;
		//Serial.printf("rel alt: %ld",temp.alt);
	} else {
		temp.alt = next_command.alt;
		//Serial.printf("abs alt: %ld",temp.alt);
	}

	takeoff_complete = false;
	// set flag to use g_gps ground course during TO.  IMU will be doing yaw drift correction

	// Set our waypoint
	set_next_WP(&temp);
}

static void do_nav_wp()
{
	wp_control = WP_MODE;

	// next_command.alt is a relative altitude!!!
	if (next_command.options & WP_OPTION_ALT_RELATIVE) {
		next_command.alt	+= home.alt;
	}
	set_next_WP(&next_command);

	// this is our bitmask to verify we have met all conditions to move on
	wp_verify_byte 	= 0;

	// this will be used to remember the time in millis after we reach or pass the WP.
	loiter_time 	= 0;

	// this is the delay, stored in seconds and expanded to millis
	loiter_time_max = next_command.p1 * 1000;

	// if we don't require an altitude minimum, we save this flag as passed (1)
	if((next_WP.options & WP_OPTION_ALT_REQUIRED) == 0){
		// we don't need to worry about it
		wp_verify_byte |= NAV_ALTITUDE;
	}
}

static void do_land()
{
	wp_control = LOITER_MODE;

	//Serial.println("dlnd ");

	// not really used right now, might be good for debugging
	land_complete		= false;

	// A value that drives to 0 when the altitude doesn't change
	velocity_land		= 2000;

	// used to limit decent rate
	land_start 			= millis();

	// used to limit decent rate
	original_alt		= current_loc.alt;

	// hold at our current location
	set_next_WP(&current_loc);
}

static void do_loiter_unlimited()
{
	wp_control = LOITER_MODE;

	//Serial.println("dloi ");
	if(next_command.lat == 0)
		set_next_WP(&current_loc);
	else
		set_next_WP(&next_command);
}

static void do_loiter_turns()
{
/*
	wp_control = LOITER_MODE;

	if(next_command.lat == 0)
		set_next_WP(&current_loc);
	else
		set_next_WP(&next_command);

	loiter_total = next_command.p1 * 360;
*/
}

static void do_loiter_time()
{
	///*
	wp_control = LOITER_MODE;
	set_next_WP(&current_loc);
	loiter_time 	= millis();
	loiter_time_max = next_command.p1 * 1000; // units are (seconds)
	//Serial.printf("dlt %ld, max %ld\n",loiter_time, loiter_time_max);
	//*/
}

/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

static bool verify_takeoff()
{

	// wait until we are ready!
	if(g.rc_3.control_in == 0){
		return false;
	}

	if (current_loc.alt > next_WP.alt){
		//Serial.println("Y");
		takeoff_complete = true;
		return true;

	}else{

		//Serial.println("N");
		return false;
	}
}

static bool verify_land()
{
	// land at 1 meter per second
	next_WP.alt  = original_alt - ((millis() - land_start) / 20);			// condition_value = our initial

	velocity_land  = ((old_alt - current_loc.alt) *.2) + (velocity_land * .8);
	old_alt = current_loc.alt;

	if(g.sonar_enabled){
		// decide which sensor we're using
		if(sonar_alt < 40){
			land_complete = true;
			//Serial.println("Y");
			//return true;
		}
	}

	if(velocity_land <= 0){
		land_complete = true;
		//return true;
	}
	//Serial.printf("N, %d\n", velocity_land);
	//Serial.printf("N_alt, %ld\n", next_WP.alt);

	return false;
}

static bool verify_nav_wp()
{
	// Altitude checking
	if(next_WP.options & WP_OPTION_ALT_REQUIRED){
		// we desire a certain minimum altitude
		if (current_loc.alt > next_WP.alt){
			// we have reached that altitude
			wp_verify_byte |= NAV_ALTITUDE;
		}
	}

	// Did we pass the WP?	// Distance checking
	if((wp_distance <= g.waypoint_radius) || check_missed_wp()){

		// if we have a distance calc error, wp_distance may be less than 0
		if(wp_distance > 0){
			wp_verify_byte |= NAV_LOCATION;

			if(loiter_time == 0){
				loiter_time = millis();
			}
		}
	}

	// Hold at Waypoint checking, we cant move on until this is OK
	if(wp_verify_byte & NAV_LOCATION){
		// we have reached our goal

		// loiter at the WP
		wp_control 	= LOITER_MODE;

		if ((millis() - loiter_time) > loiter_time_max) {
			wp_verify_byte |= NAV_DELAY;
			//gcs.send_text_P(SEVERITY_LOW,PSTR("verify_must: LOITER time complete"));
			//Serial.println("vlt done");
		}
	}

	if(wp_verify_byte >= 7){
	//if(wp_verify_byte & NAV_LOCATION){
		char message[30];
		sprintf(message,"Reached Command #%i",command_must_index);
		gcs.send_text(SEVERITY_LOW,message);
		wp_verify_byte = 0;
		return true;
	}else{
		return false;
	}
}

static bool verify_loiter_unlim()
{
	return false;
}

static bool verify_loiter_time()
{
	//Serial.printf("vlt %ld\n",(millis() - loiter_time));

	if ((millis() - loiter_time) > loiter_time_max) {		// scale loiter_time_max from (sec*10) to milliseconds
		//gcs.send_text_P(SEVERITY_LOW,PSTR("verify_must: LOITER time complete"));
		//Serial.println("vlt done");
		return true;
	}
	return false;
}

static bool verify_RTL()
{
	if (wp_distance <= g.waypoint_radius) {
		gcs.send_text_P(SEVERITY_LOW,PSTR("Reached home"));
		return true;
	}else{
		return false;
	}
}

/********************************************************************************/
//	Condition (May) commands
/********************************************************************************/

static void do_wait_delay()
{
	//Serial.print("dwd ");
	condition_start = millis();
	condition_value	 = next_command.lat * 1000; // convert to milliseconds
	Serial.println(condition_value,DEC);
}

static void do_change_alt()
{
	Location temp	= next_WP;
	condition_start = current_loc.alt;
	if (next_command.options & WP_OPTION_ALT_RELATIVE) {
		condition_value		= next_command.alt + home.alt;
	} else {
		condition_value		= next_command.alt;
	}
	temp.alt		= condition_value;
	set_next_WP(&temp);
}

static void do_within_distance()
{
	condition_value	 = next_command.lat;
}

static void do_yaw()
{
	//Serial.println("dyaw ");
	yaw_tracking = MAV_ROI_NONE;

	// target angle in degrees
	command_yaw_start		= nav_yaw; // current position
	command_yaw_start_time	= millis();

	command_yaw_dir			= next_command.p1;		// 1 = clockwise,	 0 = counterclockwise
	command_yaw_relative	= next_command.lng;		// 1 = Relative,	 0 = Absolute

	command_yaw_speed		= next_command.lat * 100; // ms * 100


	// if unspecified go 30° a second
	if(command_yaw_speed == 0)
		command_yaw_speed = 3000;

	// if unspecified go counterclockwise
	if(command_yaw_dir == 0)
		command_yaw_dir = -1;

	if (command_yaw_relative){
		// relative
		//command_yaw_dir	  = (command_yaw_end > 0) ? 1 : -1;
		//command_yaw_end	  += nav_yaw;
		//command_yaw_end	  = wrap_360(command_yaw_end);
		command_yaw_delta	= next_command.alt * 100;
	}else{
		// absolute
		command_yaw_end		= next_command.alt * 100;

		// calculate the delta travel in deg * 100
		if(command_yaw_dir == 1){
			if(command_yaw_start >= command_yaw_end){
				command_yaw_delta = 36000 - (command_yaw_start - command_yaw_end);
			}else{
				command_yaw_delta = command_yaw_end - command_yaw_start;
			}
		}else{
			if(command_yaw_start > command_yaw_end){
				command_yaw_delta = command_yaw_start - command_yaw_end;
			}else{
				command_yaw_delta = 36000 + (command_yaw_start - command_yaw_end);
			}
		}
		command_yaw_delta = wrap_360(command_yaw_delta);
	}


	// rate to turn deg per second - default is ten
	command_yaw_time	= (command_yaw_delta / command_yaw_speed) * 1000;
}


/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
	//Serial.print("vwd");
	if ((unsigned)(millis() - condition_start) > condition_value){
		//Serial.println("y");
		condition_value		= 0;
		return true;
	}
	//Serial.println("n");
	return false;
}

static bool verify_change_alt()
{
	if (condition_start < next_WP.alt){
		// we are going higer
		if(current_loc.alt > next_WP.alt){
			condition_value = 0;
			return true;
		}
	}else{
		// we are going lower
		if(current_loc.alt < next_WP.alt){
			condition_value = 0;
			return true;
		}
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

static bool verify_yaw()
{
	//Serial.print("vyaw ");

	if((millis() - command_yaw_start_time) > command_yaw_time){
		// time out
		// make sure we hold at the final desired yaw angle
		nav_yaw = command_yaw_end;
		//Serial.println("Y");
		return true;

	}else{
		// else we need to be at a certain place
		// power is a ratio of the time : .5 = half done
		float power = (float)(millis() - command_yaw_start_time) / (float)command_yaw_time;

		nav_yaw		= command_yaw_start + ((float)command_yaw_delta * power * command_yaw_dir);
		nav_yaw		= wrap_360(nav_yaw);
		//Serial.printf("ny %ld\n",nav_yaw);
		return false;
	}
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/

static void do_target_yaw()
{
	yaw_tracking = next_command.p1;

	if(yaw_tracking == MAV_ROI_LOCATION){
		target_WP = next_command;
	}
}

static void do_loiter_at_location()
{
	next_WP = current_loc;
}

static void do_jump()
{
	struct Location temp;
	if(next_command.lat > 0) {

		command_must_index	= NO_COMMAND;
		command_may_index	= NO_COMMAND;
		temp				= get_command_with_index(g.waypoint_index);
		temp.lat			= next_command.lat - 1;					// Decrement repeat counter

		set_command_with_index(temp, g.waypoint_index);
		g.waypoint_index.set_and_save(next_command.p1 - 1);
	}
}

static void do_set_home()
{
	if(next_command.p1 == 1) {
		init_home();
	} else {
		home.id		= MAV_CMD_NAV_WAYPOINT;
		home.lng	= next_command.lng;				// Lon * 10**7
		home.lat	= next_command.lat;				// Lat * 10**7
		home.alt	= max(next_command.alt, 0);
		home_is_set = true;
	}
}

static void do_set_servo()
{
	APM_RC.OutputCh(next_command.p1 - 1, next_command.alt);
}

static void do_set_relay()
{
	if (next_command.p1 == 1) {
		relay_on();
	} else if (next_command.p1 == 0) {
		relay_off();
	}else{
		relay_toggle();
	}
}

static void do_repeat_servo()
{
	event_id = next_command.p1 - 1;

	if(next_command.p1 >= CH_5 + 1 && next_command.p1 <= CH_8 + 1) {

		event_timer		= 0;
		event_delay		= next_command.lng * 500.0; // /2 (half cycle time) * 1000 (convert to milliseconds)
		event_repeat	= next_command.lat * 2;
		event_value		= next_command.alt;

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

static void do_repeat_relay()
{
	event_id		= RELAY_TOGGLE;
	event_timer		= 0;
	event_delay		= next_command.lat * 500.0; // /2 (half cycle time) * 1000 (convert to milliseconds)
	event_repeat	= next_command.alt * 2;
	update_events();
}
