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

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			do_RTL();
			break;
	}
}

void
handle_process_may()
{
	switch(next_command.id){

		case MAV_CMD_CONDITION_DELAY:
			do_delay();
			break;

		case MAV_CMD_CONDITION_CHANGE_ALT:
			do_change_alt();
			break;

		case MAV_CMD_CONDITION_ANGLE:
			do_yaw();
			break;

		default:
			break;
	}
}

void
handle_process_now()
{
	switch(next_command.id){
		case MAV_CMD_DO_SET_HOME:
			init_home();
			break;

		case MAV_CMD_DO_REPEAT_SERVO:
			new_event(&next_command);
			break;

		case MAV_CMD_DO_SET_SERVO:
			do_set_servo();
			break;

		case MAV_CMD_DO_SET_RELAY:
			do_set_relay();
			break;
	}
}

void
handle_no_commands()
{
	switch (control_mode){
		case LAND:
			// don't get a new command
			break;

		//case GCS_AUTO:
		//	set_mode(LOITER);

		default:
			set_mode(RTL);
			//next_command = get_LOITER_home_wp();
			//SendDebug("MSG <load_next_command> Preload RTL cmd id: ");
			//SendDebugln(next_command.id,DEC);
			break;
	}
}

bool verify_must()
{
	switch(command_must_ID) {

		case MAV_CMD_NAV_TAKEOFF:	// Takeoff!
			return verify_takeoff();
			break;

		case MAV_CMD_NAV_LAND:
			return verify_land();
			break;

		case MAV_CMD_NAV_WAYPOINT:	// reach a waypoint
			return verify_nav_wp();
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			return verify_RTL();
			break;

		default:
			//gcs.send_text(SEVERITY_HIGH,"<verify_must: default> No current Must commands");
			return false;
			break;
	}
}

bool verify_may()
{
	switch(command_may_ID) {

		case MAV_CMD_CONDITION_ANGLE:
			return verify_yaw();
			break;

		case MAV_CMD_CONDITION_DELAY:
			return verify_delay();
			break;

		case MAV_CMD_CONDITION_CHANGE_ALT:
			return verify_change_alt();
			break;

		default:
			//gcs.send_text(SEVERITY_HIGH,"<verify_must: default> No current May commands");
			return false;
			break;

	}
}

/********************************************************************************/
// Must command implementations
/********************************************************************************/

void
do_takeoff()
{
	Location temp 		= current_loc;
	temp.alt			= next_command.alt;
	takeoff_complete 	= false;			// set flag to use g_gps ground course during TO.  IMU will be doing yaw drift correction

	set_next_WP(&temp);
}

bool
verify_takeoff()
{
	if (current_loc.alt > next_WP.alt){
		takeoff_complete 	= true;
		return true;
	}else{
		return false;
	}
}

void
do_nav_wp()
{
	set_next_WP(&next_command);
}

bool
verify_nav_wp()
{
	update_crosstrack();
	if ((wp_distance > 0) && (wp_distance <= g.waypoint_radius)) {
		//SendDebug("MSG <verify_must: MAV_CMD_NAV_WAYPOINT> REACHED_WAYPOINT #");
		//SendDebugln(command_must_index,DEC);
		char message[30];
		sprintf(message,"Reached Waypoint #%i",command_must_index);
		gcs.send_text(SEVERITY_LOW,message);
		return true;
	}
	// add in a more complex case
	// Doug to do
	if(loiter_sum > 300){
		gcs.send_text(SEVERITY_MEDIUM,"Missed WP");
		return true;
	}
	return false;
}

void
do_land()
{
	land_complete 		= false;			// set flag to use g_gps ground course during TO.  IMU will be doing yaw drift correction
	velocity_land		= 1000;

	Location temp 		= current_loc;
	temp.alt 			= home.alt;

	set_next_WP(&temp);
}

bool
verify_land()
{
	update_crosstrack();

	velocity_land  = ((old_alt - current_loc.alt) *.05) + (velocity_land * .95);
	old_alt = current_loc.alt;

	if(velocity_land == 0){
		land_complete 		= true;
		return true;
	}

	return false;
}

// add a new command at end of command set to RTL.
void
do_RTL()
{
	Location temp 	= home;
	temp.alt 		= read_alt_to_hold();

	//so we know where we are navigating from
	next_WP = current_loc;

	// Loads WP from Memory
	// --------------------
	set_next_WP(&temp);
}

bool
verify_RTL()
{
	if (wp_distance <= g.waypoint_radius) {
		gcs.send_text(SEVERITY_LOW,"Reached home");
		return true;
	}else{
		return false;
	}
}

/********************************************************************************/
// May command implementations
/********************************************************************************/

void
do_yaw()
{
	// p1:		bearing
	// alt:		speed
	// lat:		direction (-1,1),
	// lng:		rel (1) abs (0)

	// target angle in degrees
	command_yaw_start		= nav_yaw; // current position
	command_yaw_start_time 	= millis();

	// which direction to turn
	// 1 = clockwise, -1 = counterclockwise
	command_yaw_dir		= next_command.lat;

	// 1 = Relative or 0 = Absolute
	if (next_command.lng == 1) {
		// relative
		command_yaw_dir  = (command_yaw_end > 0) ? 1 : -1;
		command_yaw_end += nav_yaw;
		command_yaw_end = wrap_360(command_yaw_end);
	}else{
		// absolute
		command_yaw_end 	= next_command.p1 * 100;
	}


	// if unspecified go 10° a second
	if(command_yaw_speed == 0)
		command_yaw_speed = 10;

	// if unspecified go clockwise
	if(command_yaw_dir == 0)
		command_yaw_dir = 1;

	// calculate the delta travel
	if(command_yaw_dir == 1){
		if(command_yaw_start > command_yaw_end){
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

	// rate to turn deg per second - default is ten
	command_yaw_speed 	= next_command.alt;
	command_yaw_time 	= command_yaw_delta / command_yaw_speed;
	//9000 turn in 10 seconds
	//command_yaw_time = 9000/ 10 = 900° per second
}

bool
verify_yaw()
{
	if((millis() - command_yaw_start_time) > command_yaw_time){
		nav_yaw = command_yaw_end;
		return true;
	}else{
		// else we need to be at a certain place
		// power is a ratio of the time : .5 = half done
		float power = (float)(millis() - command_yaw_start_time) / (float)command_yaw_time;
		nav_yaw 	= command_yaw_start + ((float)command_yaw_delta * power * command_yaw_dir);
		return false;
	}
}

void
do_delay()
{
	delay_start = millis();
	delay_timeout  = next_command.lat;
}

bool
verify_delay()
{
	if ((millis() - delay_start) > delay_timeout){
		delay_timeout = 0;
		return true;
	}else{
		return false;
	}
}

void
do_change_alt()
{
	Location temp 	= next_WP;
	temp.alt 		= next_command.alt + home.alt;
	set_next_WP(&temp);
}

bool
verify_change_alt()
{
	if(abs(current_loc.alt - next_WP.alt) < 100){
		return true;
	}else{
		return false;
	}
}

/********************************************************************************/
// Now command implementations
/********************************************************************************/

void do_hold_position()
{
	set_next_WP(&current_loc);
}

void do_set_servo()
{
	APM_RC.OutputCh(next_command.p1, next_command.alt);
}

void do_set_relay()
{
	if (next_command.p1 == 0) {
		relay_on();
	} else if (next_command.p1 == 1) {
		relay_off();
	}else{
		relay_toggle();
	}
}
