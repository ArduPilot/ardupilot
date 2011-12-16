/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/********************************************************************************/
// Command Event Handlers
/********************************************************************************/
static void process_nav_command()
{
	switch(command_nav_queue.id){

		case MAV_CMD_NAV_TAKEOFF:	// 22
			do_takeoff();
			break;

		case MAV_CMD_NAV_WAYPOINT:	// 16  Navigate to Waypoint
			do_nav_wp();
			break;

		case MAV_CMD_NAV_LAND:	// 21 LAND to Waypoint
			do_land();
			break;

		case MAV_CMD_NAV_LOITER_UNLIM:	// 17 Loiter indefinitely
			do_loiter_unlimited();
			break;

		case MAV_CMD_NAV_LOITER_TURNS:	//18 Loiter N Times
			do_loiter_turns();
			break;

		case MAV_CMD_NAV_LOITER_TIME:  // 19
			do_loiter_time();
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH: //20
			do_RTL();
			break;

		default:
			break;
	}

}

static void process_cond_command()
{
	switch(command_cond_queue.id){

		case MAV_CMD_CONDITION_DELAY: // 112
			do_wait_delay();
			break;

		case MAV_CMD_CONDITION_DISTANCE: // 114
			do_within_distance();
			break;

		case MAV_CMD_CONDITION_CHANGE_ALT: // 113
			do_change_alt();
			break;

		case MAV_CMD_CONDITION_YAW: // 115
			do_yaw();
			break;

		default:
			break;
	}
}

static void process_now_command()
{
	switch(command_cond_queue.id){

		case MAV_CMD_DO_JUMP:  // 177
			do_jump();
			break;

		case MAV_CMD_DO_CHANGE_SPEED: // 178
			do_change_speed();
			break;

		case MAV_CMD_DO_SET_HOME: // 179
			do_set_home();
			break;

		case MAV_CMD_DO_SET_SERVO: // 183
			do_set_servo();
			break;

		case MAV_CMD_DO_SET_RELAY: // 181
			do_set_relay();
			break;

		case MAV_CMD_DO_REPEAT_SERVO: // 184
			do_repeat_servo();
			break;

		case MAV_CMD_DO_REPEAT_RELAY: // 182
			do_repeat_relay();
			break;

		case MAV_CMD_DO_SET_ROI: // 201
			do_target_yaw();
	}
}

//static void handle_no_commands()
//{
	/*
	switch (control_mode){
		default:
			set_mode(RTL);
			break;
	}*/
	//return;
	//Serial.println("Handle No CMDs");
//}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

static bool verify_must()
{
	//Serial.printf("vmust: %d\n", command_nav_ID);

	switch(command_nav_queue.id) {

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
			return verify_loiter_turns();
			break;

		case MAV_CMD_NAV_LOITER_TIME:
			return verify_loiter_time();
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			return verify_RTL();
			break;

		default:
			//gcs_send_text_P(SEVERITY_HIGH,PSTR("<verify_must: default> No current Must commands"));
			return false;
			break;
	}
}

static bool verify_may()
{
	switch(command_cond_queue.id) {

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
			//gcs_send_text_P(SEVERITY_HIGH,PSTR("<verify_must: default> No current May commands"));
			return false;
			break;
	}
}

/********************************************************************************/
//
/********************************************************************************/

static void do_RTL(void)
{
	// TODO: Altitude option from mission planner
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
	gcs_send_message(MSG_HEARTBEAT);
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

static void do_takeoff()
{
	wp_control = LOITER_MODE;

	// Start with current location
	Location temp = current_loc;

	// command_nav_queue.alt is a relative altitude!!!
	if (command_nav_queue.options & MASK_OPTIONS_RELATIVE_ALT) {
		temp.alt = command_nav_queue.alt + home.alt;
		//Serial.printf("rel alt: %ld",temp.alt);
	} else {
		temp.alt = command_nav_queue.alt;
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

	// command_nav_queue.alt is a relative altitude!!!
	if (command_nav_queue.options & MASK_OPTIONS_RELATIVE_ALT) {
		command_nav_queue.alt	+= home.alt;
	}
	set_next_WP(&command_nav_queue);


	// this is our bitmask to verify we have met all conditions to move on
	wp_verify_byte 	= 0;

	// this will be used to remember the time in millis after we reach or pass the WP.
	loiter_time 	= 0;

	// this is the delay, stored in seconds and expanded to millis
	loiter_time_max = command_nav_queue.p1 * 1000;

	// if we don't require an altitude minimum, we save this flag as passed (1)
	if((next_WP.options & MASK_OPTIONS_RELATIVE_ALT) == 0){
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
	if(command_nav_queue.lat == 0)
		set_next_WP(&current_loc);
	else
		set_next_WP(&command_nav_queue);
}

static void do_loiter_turns()
{
	wp_control = CIRCLE_MODE;

	if(command_nav_queue.lat == 0){
		// allow user to specify just the altitude
		if(command_nav_queue.alt > 0){
			current_loc.alt = command_nav_queue.alt;
		}
		set_next_WP(&current_loc);
	}else{
		set_next_WP(&command_nav_queue);
	}

	loiter_total = command_nav_queue.p1 * 360;
	loiter_sum	 = 0;
	old_target_bearing = target_bearing;
}

static void do_loiter_time()
{
	if(command_nav_queue.lat == 0){
		wp_control 		= LOITER_MODE;
		loiter_time 	= millis();
		set_next_WP(&current_loc);
	}else{
		wp_control 		= WP_MODE;
		set_next_WP(&command_nav_queue);
	}

	loiter_time_max = command_nav_queue.p1 * 1000; // units are (seconds)
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
	// land at .62 meter per second
	next_WP.alt  = original_alt - ((millis() - land_start) / 16);			// condition_value = our initial

	velocity_land  = ((old_alt - current_loc.alt) *.2) + (velocity_land * .8);
	old_alt = current_loc.alt;

	if (current_loc.alt < 250){
		wp_control = NO_NAV_MODE;
		next_WP.alt = -200; // force us down
	}

	if(g.sonar_enabled){
		// decide which sensor we're using
		if(sonar_alt < 40){
			land_complete = true;
			//Serial.println("Y");
			return true;
		}
	}

	if(velocity_land <= 0){
		land_complete = true;
		// commented out to prevent tragedy
		//return true;
	}
	//Serial.printf("N, %d\n", velocity_land);
	//Serial.printf("N_alt, %ld\n", next_WP.alt);
	return false;
}

static bool verify_nav_wp()
{
	// Altitude checking
	if(next_WP.options & MASK_OPTIONS_RELATIVE_ALT){
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
			//gcs_send_text_P(SEVERITY_LOW,PSTR("verify_must: LOITER time complete"));
			//Serial.println("vlt done");
		}
	}

	if(wp_verify_byte >= 7){
	//if(wp_verify_byte & NAV_LOCATION){
		char message[30];
		sprintf(message,"Reached Command #%i",command_nav_index);
		gcs_send_text(SEVERITY_LOW,message);
		wp_verify_byte = 0;
		return true;
	}else{
		return false;
	}
}

//static bool verify_loiter_unlim()
//{
//	return false;
//}

static bool verify_loiter_time()
{
	if(wp_control == LOITER_MODE){
		if ((millis() - loiter_time) > loiter_time_max) {
			return true;
		}
	}
	if(wp_control == WP_MODE &&  wp_distance <= g.waypoint_radius){
		// reset our loiter time
		loiter_time = millis();
		// switch to position hold
		wp_control 	= LOITER_MODE;
	}
	return false;
}

static bool verify_loiter_turns()
{
	//Serial.printf("loiter_sum: %d \n", loiter_sum);
	// have we rotated around the center enough times?
	// -----------------------------------------------
	if(abs(loiter_sum) > loiter_total) {
		loiter_total 	= 0;
		loiter_sum		= 0;
		//gcs_send_text_P(SEVERITY_LOW,PSTR("verify_must: LOITER orbits complete"));
		// clear the command queue;
		return true;
	}
	return false;
}

static bool verify_RTL()
{
	// loiter at the WP
	wp_control 	= WP_MODE;

	// Did we pass the WP?	// Distance checking
	if((wp_distance <= g.waypoint_radius) || check_missed_wp()){
		wp_control 	= LOITER_MODE;

		//gcs_send_text_P(SEVERITY_LOW,PSTR("Reached home"));
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
	condition_value	= command_cond_queue.lat * 1000; // convert to milliseconds
	//Serial.println(condition_value,DEC);
}

static void do_change_alt()
{
	Location temp	= next_WP;
	condition_start = current_loc.alt;
	condition_value	= command_cond_queue.alt;
	temp.alt		= command_cond_queue.alt;
	set_next_WP(&temp);
}

static void do_within_distance()
{
	condition_value	 = command_cond_queue.lat;
}

static void do_yaw()
{
	//Serial.println("dyaw ");
	yaw_tracking = MAV_ROI_NONE;

	// target angle in degrees
	command_yaw_start		= nav_yaw; // current position
	command_yaw_start_time	= millis();

	command_yaw_dir			= command_cond_queue.p1;			// 1 = clockwise,	 0 = counterclockwise
	command_yaw_speed		= command_cond_queue.lat * 100; 	// ms * 100
	command_yaw_relative	= command_cond_queue.lng;			// 1 = Relative,	 0 = Absolute



	// if unspecified go 30° a second
	if(command_yaw_speed == 0)
		command_yaw_speed = 3000;

	// if unspecified go counterclockwise
	if(command_yaw_dir == 0)
		command_yaw_dir = -1;
	else
		command_yaw_dir = 1;

	if (command_yaw_relative == 1){
		// relative
		command_yaw_delta	= command_cond_queue.alt * 100;

	}else{
		// absolute
		command_yaw_end		= command_cond_queue.alt * 100;

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
	//Serial.printf("change_alt, ca:%d, na:%d\n", (int)current_loc.alt, (int)next_WP.alt);
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
	//Serial.printf("cond dist :%d\n", (int)condition_value);
	if (wp_distance < condition_value){
		condition_value = 0;
		return true;
	}
	return false;
}

static bool verify_yaw()
{
	//Serial.printf("vyaw %d\n", (int)(nav_yaw/100));

	if((millis() - command_yaw_start_time) > command_yaw_time){
		// time out
		// make sure we hold at the final desired yaw angle
		nav_yaw = command_yaw_end;
		auto_yaw 	= nav_yaw;

		//Serial.println("Y");
		return true;

	}else{
		// else we need to be at a certain place
		// power is a ratio of the time : .5 = half done
		float power = (float)(millis() - command_yaw_start_time) / (float)command_yaw_time;

		nav_yaw		= command_yaw_start + ((float)command_yaw_delta * power * command_yaw_dir);
		nav_yaw		= wrap_360(nav_yaw);
		auto_yaw 	= nav_yaw;
		//Serial.printf("ny %ld\n",nav_yaw);
		return false;
	}
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/

static void do_change_speed()
{
	g.waypoint_speed_max = command_cond_queue.p1 * 100;
}

static void do_target_yaw()
{
	yaw_tracking = command_cond_queue.p1;

	if(yaw_tracking == MAV_ROI_LOCATION){
		target_WP = command_cond_queue;
	}
}

static void do_loiter_at_location()
{
	next_WP = current_loc;
}

static void do_jump()
{
	//Serial.printf("do Jump: %d\n", jump);

	if(jump == -10){
		//Serial.printf("Fresh Jump\n");
		// we use a locally stored index for jump
		jump = command_cond_queue.lat;
	}
	//Serial.printf("Jumps left: %d\n",jump);

	if(jump > 0) {
		//Serial.printf("Do Jump to %d\n",command_cond_queue.p1);
		jump--;
		change_command(command_cond_queue.p1);

	} else if (jump == 0){
		//Serial.printf("Did last jump\n");
		// we're done, move along
		jump = -11;

	} else if (jump == -1) {
		//Serial.printf("jumpForever\n");
		// repeat forever
		change_command(command_cond_queue.p1);
	}
}

static void do_set_home()
{
	if(command_cond_queue.p1 == 1) {
		init_home();
	} else {
		home.id		= MAV_CMD_NAV_WAYPOINT;
		home.lng	= command_cond_queue.lng;				// Lon * 10**7
		home.lat	= command_cond_queue.lat;				// Lat * 10**7
		home.alt	= max(command_cond_queue.alt, 0);
		home_is_set = true;
	}
}

static void do_set_servo()
{
	APM_RC.OutputCh(command_cond_queue.p1 - 1, command_cond_queue.alt);
}

static void do_set_relay()
{
	if (command_cond_queue.p1 == 1) {
		relay.on();
	} else if (command_cond_queue.p1 == 0) {
		relay.off();
	}else{
		relay.toggle();
	}
}

static void do_repeat_servo()
{
	event_id = command_cond_queue.p1 - 1;

	if(command_cond_queue.p1 >= CH_5 + 1 && command_cond_queue.p1 <= CH_8 + 1) {

		event_timer		= 0;
		event_value		= command_cond_queue.alt;
		event_repeat	= command_cond_queue.lat * 2;
		event_delay		= command_cond_queue.lng * 500.0; // /2 (half cycle time) * 1000 (convert to milliseconds)

		switch(command_cond_queue.p1) {
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
	event_delay		= command_cond_queue.lat * 500.0; // /2 (half cycle time) * 1000 (convert to milliseconds)
	event_repeat	= command_cond_queue.alt * 2;
	update_events();
}
