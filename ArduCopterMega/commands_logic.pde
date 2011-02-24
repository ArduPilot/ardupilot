void
handle_no_commands()
{
	switch (control_mode){
		case LAND:
			// don't get a new command
			break;

		default:
			next_command = get_LOITER_home_wp();
			//SendDebug("MSG <load_next_command> Preload RTL cmd id: ");
			//SendDebugln(next_command.id,DEC);
			break;
	}
}

void
handle_process_must(byte id)
{
	// reset navigation integrators
	// -------------------------
	reset_I();

	switch(id){
		case MAV_CMD_NAV_TAKEOFF:	// TAKEOFF!
			// pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0
			takeoff_altitude 	= (int)next_command.alt;
			next_WP.lat 		= current_loc.lat + 10;	// so we don't have bad calcs
			next_WP.lng 		= current_loc.lng + 10;	// so we don't have bad calcs
			next_WP.alt			= current_loc.alt + takeoff_altitude;
			takeoff_complete 	= false;			// set flag to use g_gps ground course during TO.  IMU will be doing yaw drift correction
			//set_next_WP(&next_WP);
			break;

		case MAV_CMD_NAV_WAYPOINT:	// Navigate to Waypoint
			break;

		//case MAV_CMD_NAV_R_WAYPOINT:	// Navigate to Waypoint
		//	next_command.lat 	+= home.lat;	// offset from home location
		//	next_command.lng 	+= home.lng;	// offset from home location

			// we've recalculated the WP so we need to set it again
		//	set_next_WP(&next_command);
		//	break;

		case MAV_CMD_NAV_LAND:	// LAND to Waypoint
			velocity_land		= 1000;
			next_WP.lat 		= current_loc.lat;
			next_WP.lng 		= current_loc.lng;
			next_WP.alt 		= home.alt;
			land_complete 		= false;			// set flag to use g_gps ground course during TO.  IMU will be doing yaw drift correction
			break;

		/*
		case MAV_CMD_ALTITUDE:	//
			next_WP.lat 		= current_loc.lat + 10;	// so we don't have bad calcs
			next_WP.lng 		= current_loc.lng + 10;	// so we don't have bad calcs
			break;
		*/

		case MAV_CMD_NAV_LOITER_UNLIM:	// Loiter indefinitely
			break;

		case MAV_CMD_NAV_LOITER_TURNS:	// Loiter N Times
			break;

		case MAV_CMD_NAV_LOITER_TIME:
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			return_to_launch();
			break;
	}
}

void
handle_process_may(byte id)
{
	switch(id){

		case MAV_CMD_CONDITION_DELAY:	// Navigate to Waypoint
			delay_start = millis();
			delay_timeout  = next_command.lat;
			break;

		//case MAV_CMD_NAV_LAND_OPTIONS:	// Land this puppy
		//	break;

		case MAV_CMD_CONDITION_YAW:
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

			break;

		default:
			break;
	}}

void
handle_process_now(byte id)
{
	switch(id){
		case MAV_CMD_DO_SET_HOME:
			init_home();
			break;

		case MAV_CMD_DO_REPEAT_SERVO:
			new_event(&next_command);
			break;

		case MAV_CMD_DO_SET_SERVO:
			//Serial.print("MAV_CMD_DO_SET_SERVO ");
			//Serial.print(next_command.p1,DEC);
			//Serial.print(" PWM: ");
			//Serial.println(next_command.alt,DEC);
			APM_RC.OutputCh(next_command.p1, next_command.alt);
			break;

		case MAV_CMD_DO_SET_RELAY:
			if (next_command.p1 == 0) {
				relay_on();
			} else if (next_command.p1 == 1) {
				relay_off();
			}else{
				relay_toggle();
			}
			break;
	}
}

// Verify commands
// ---------------
void verify_must()
{
	switch(command_must_ID) {

		/*case MAV_CMD_ALTITUDE:
				if (abs(next_WP.alt - current_loc.alt) < 100){
					command_must_index 	= 0;
				}
			break;
		*/

		case MAV_CMD_NAV_TAKEOFF:	// Takeoff!
			if (current_loc.alt > (next_WP.alt -100)){
				command_must_index 	= 0;
				takeoff_complete 	= true;
			}
			break;

		case MAV_CMD_NAV_LAND:
			// 10 - 9  = 1
			velocity_land  = ((old_alt - current_loc.alt) *.05) + (velocity_land * .95);
			old_alt = current_loc.alt;
			if(velocity_land == 0){
				land_complete 		= true;
				command_must_index 	= 0;
			}
			update_crosstrack();

			break;

		case MAV_CMD_NAV_WAYPOINT:	// reach a waypoint
			update_crosstrack();
			if ((wp_distance > 0) && (wp_distance <= g.waypoint_radius)) {
				//SendDebug("MSG <verify_must: MAV_CMD_NAV_WAYPOINT> REACHED_WAYPOINT #");
				//SendDebugln(command_must_index,DEC);
				char message[50];
				sprintf(message,"Reached Waypoint #%i",command_must_index);
				gcs.send_text(SEVERITY_LOW,message);

				// clear the command queue;
				command_must_index 	= 0;
			}
			// add in a more complex case
			// Doug to do
			if(loiter_sum > 300){
				send_message(SEVERITY_MEDIUM,"Missed WP");
				command_must_index 	= 0;
			}
			break;

		case MAV_CMD_NAV_LOITER_TURNS:	// LOITER N times
			break;

		case MAV_CMD_NAV_LOITER_TIME:	// loiter N milliseconds
			break;

		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
			if (wp_distance <= g.waypoint_radius) {
				gcs.send_text(SEVERITY_LOW,"Reached home");
				command_must_index 	= 0;
			}
			break;

		//case MAV_CMD_NAV_LOITER_UNLIM:	// Just plain LOITER
		//	break;


		default:
			gcs.send_text(SEVERITY_HIGH,"<verify_must: default> No current Must commands");
			break;
	}
}

void verify_may()
{
	float power;
	switch(command_may_ID) {

		case MAV_CMD_CONDITION_ANGLE:
			if((millis() - command_yaw_start_time) > command_yaw_time){
				command_must_index 	= 0;
				nav_yaw = command_yaw_end;
			}else{
				// else we need to be at a certain place
				// power is a ratio of the time : .5 = half done
				power = (float)(millis() - command_yaw_start_time) / (float)command_yaw_time;
				nav_yaw = command_yaw_start + ((float)command_yaw_delta * power * command_yaw_dir);
			}
			break;

		case MAV_CMD_CONDITION_DELAY:
			if ((millis() - delay_start) > delay_timeout){
				command_may_index = 0;
				delay_timeout = 0;
			}

		//case CMD_LAND_OPTIONS:
		//	break;
	}
}

