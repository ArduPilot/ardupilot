// called by 10 Hz loop
// --------------------
void update_commands(void)
{
	// This function loads commands into three buffers
	// when a new command is loaded, it is processed with process_XXX()
	// ----------------------------------------------------------------
	if((home_is_set == false) || (control_mode != AUTO)){
		return;	// don't do commands
	}
	
	if(control_mode == AUTO){
		load_next_command();
		process_next_command();
	}
	
	//verify_must();
	//verify_may();
}


void load_next_command()
{
	// fetch next command if it's empty
	// --------------------------------
	if(next_command.id == CMD_BLANK){
		next_command = get_wp_with_index(g.waypoint_index + 1);
		if(next_command.id != CMD_BLANK){
			//Serial.print("MSG fetch found new cmd from list at index: ");
			//Serial.println((g.waypoint_index + 1),DEC);
			//Serial.print("MSG cmd id: ");
			//Serial.println(next_command.id,DEC);	
		}
	}
	
	// If the preload failed, return or just Loiter
	// generate a dynamic command for RTL
	// --------------------------------------------
	if(next_command.id == CMD_BLANK){
		// we are out of commands!
		//send_message(SEVERITY_LOW,"out of commands!");
		//Serial.print("MSG out of commands, wp_index: ");
		//Serial.println(g.waypoint_index,DEC);

		
		switch (control_mode){
			case LAND:
				// don't get a new command
				break;
				
			default:
				next_command = get_LOITER_home_wp();
				//Serial.print("MSG Preload RTL cmd id: ");
				//Serial.println(next_command.id,DEC);
				break;
		}
	}
}

void process_next_command()
{
	// these are waypoint/Must commands
	// ---------------------------------
	if (command_must_index == 0){ // no current command loaded
		if (next_command.id >= 0x10 && next_command.id <= 0x1F ){
			increment_WP_index();
			save_command_index();	// to Recover from in air Restart
			command_must_index = g.waypoint_index;
			
			//Serial.print("MSG new command_must_id ");
			//Serial.print(next_command.id,DEC);
			//Serial.print(" index:");
			//Serial.println(command_must_index,DEC);
			if (g.log_bitmask & MASK_LOG_CMD)		
				Log_Write_Cmd(g.waypoint_index, &next_command);
			process_must();
		}
	}
	
	// these are May commands
	// ----------------------
	if (command_may_index == 0){
		if (next_command.id >= 0x20 && next_command.id <= 0x2F ){
			increment_WP_index();// this command is from the command list in EEPROM
			command_may_index = g.waypoint_index;
			//Serial.print("new command_may_index ");
			//Serial.println(command_may_index,DEC);
			if (g.log_bitmask & MASK_LOG_CMD)
				Log_Write_Cmd(g.waypoint_index, &next_command);
			process_may();
		}
	}
	
	// these are do it now commands
	// ---------------------------
	if (next_command.id >= 0x30){
		increment_WP_index();// this command is from the command list in EEPROM	
		if (g.log_bitmask & MASK_LOG_CMD)
			Log_Write_Cmd(g.waypoint_index, &next_command);				
		process_now();
	}

}

/*
These functions implement the waypoint commands.
*/
void process_must()
{
	//Serial.print("process must index: ");
	//Serial.println(command_must_index,DEC);

	send_message(SEVERITY_LOW,"New cmd: ");
	send_message(MSG_COMMAND, g.waypoint_index);
	
	// clear May indexes
	command_may_index	= 0;
	command_may_ID		= 0;

	command_must_ID = next_command.id;

	// invalidate command so a new one is loaded
	// -----------------------------------------
	next_command.id = 0;
	
	// reset navigation integrators
	// -------------------------
	reset_I();
	
	// loads the waypoint into Next_WP struct
	// --------------------------------------
	set_next_WP(&next_command);
		
	switch(command_must_ID){
	
		case CMD_TAKEOFF:	// TAKEOFF!
			takeoff_altitude 	= (int)next_command.alt;
			next_WP.lat 		= current_loc.lat + 10;	// so we don't have bad calcs
			next_WP.lng 		= current_loc.lng + 10;	// so we don't have bad calcs
			next_WP.alt			= current_loc.alt + takeoff_altitude;
			takeoff_complete 	= false;			// set flag to use gps ground course during TO.  IMU will be doing yaw drift correction 
			//set_next_WP(&next_WP);
			break;

		case CMD_WAYPOINT:	// Navigate to Waypoint
			break;
			
		case CMD_R_WAYPOINT:	// Navigate to Waypoint
			next_command.lat 	+= home.lat;	// offset from home location
			next_command.lng 	+= home.lng;	// offset from home location

			// we've recalculated the WP so we need to set it again
			set_next_WP(&next_command);
			break;

		case CMD_LAND:	// LAND to Waypoint
			velocity_land		= 1000;
			next_WP.lat 		= current_loc.lat;	
			next_WP.lng 		= current_loc.lng;	
			next_WP.alt 		= home.alt;			
			land_complete 		= false;			// set flag to use gps ground course during TO.  IMU will be doing yaw drift correction 
			break;
			
		case CMD_ALTITUDE:	// Loiter indefinitely
			next_WP.lat 		= current_loc.lat + 10;	// so we don't have bad calcs
			next_WP.lng 		= current_loc.lng + 10;	// so we don't have bad calcs
			break;
			
		case CMD_LOITER:	// Loiter indefinitely
			break;
		
		case CMD_LOITER_N_TURNS:	// Loiter N Times
			break;

		case CMD_LOITER_TIME:
			break;

		case CMD_RTL:
			return_to_launch();		
			break;

	}
}

void process_may()
{
	//Serial.print("process_may cmd# ");
	//Serial.println(g.waypoint_index,DEC);	
	command_may_ID = next_command.id;

	// invalidate command so a new one is loaded
	// -----------------------------------------
	next_command.id = 0;
	
	send_message(SEVERITY_LOW,"New cmd: ");
	send_message(MSG_COMMAND, g.waypoint_index);
	
	// do the command
	// --------------
	switch(command_may_ID){
	
		case CMD_DELAY:	// Navigate to Waypoint
			delay_start = millis();
			delay_timeout  = next_command.lat;
			break;

		case CMD_LAND_OPTIONS:	// Land this puppy
			break;

		case CMD_ANGLE:
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
			} else {
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
	}
}

void process_now()
{
	const 	float t5			= 100000.0;
	//Serial.print("process_now cmd# ");
	//Serial.println(g.waypoint_index,DEC);

	byte id = next_command.id;

	// invalidate command so a new one is loaded
	// -----------------------------------------
	next_command.id = 0;
	
	send_message(SEVERITY_LOW, "New cmd: ");
	send_message(MSG_COMMAND, g.waypoint_index);
		
	// do the command
	// --------------
	switch(id){
	
		case CMD_RESET_INDEX:	
			init_commands();
			break;

		case CMD_GETVAR_INDEX:
			//
			break;

		case CMD_SENDVAR_INDEX:
			//
			break;

		case CMD_TELEMETRY:	
			//
			break;

		//case CMD_AIRSPEED_CRUISE:	
			//airspeed_cruise = next_command.p1 * 100;
			// todo save to EEPROM
			//break;

		case CMD_THROTTLE_CRUISE:	
			g.throttle_cruise = next_command.p1;
			// todo save to EEPROM
			break;

		case CMD_RESET_HOME:	
			init_home();
			break;

		case CMD_KP_GAIN:
			//kp[next_command.p1] = next_command.alt/t5;
			// todo save to EEPROM
			break;
		case CMD_KI_GAIN:
			//ki[next_command.p1] = next_command.alt/t5;
			// todo save to EEPROM
			break;
		case CMD_KD_GAIN:
			//kd[next_command.p1] = next_command.alt/t5;
			// todo save to EEPROM
			break;

		case CMD_KI_MAX:	
			//integrator_max[next_command.p1] = next_command.alt/t5;
			// todo save to EEPROM
			break;

		//case CMD_KFF_GAIN:	
		//	kff[next_command.p1] = next_command.alt/t5;
			// todo save to EEPROM
		//	break;

		//case CMD_RADIO_TRIM:	
			//radio_trim[next_command.p1] = next_command.alt;
			//save_EEPROM_trims();
			//break;
			
		//case CMD_RADIO_MAX:	
		//	radio_max[next_command.p1] = next_command.alt;
		//	save_EEPROM_radio_minmax();
		//	break;
			
		//case CMD_RADIO_MIN:	
		//	radio_min[next_command.p1] = next_command.alt;
		//	save_EEPROM_radio_minmax();
		//	break;

		case CMD_REPEAT:	
			new_event(&next_command);
			break;

		case CMD_SERVO:
			APM_RC.OutputCh(next_command.p1, next_command.alt);
			break;
			
		case CMD_INDEX:
			command_must_index = 0;
			command_may_index = 0;
			g.waypoint_index = next_command.p1 - 1;
			break;

		case CMD_RELAY:
			if(next_command.p1 = 0){
				relay_A();
			}else{
				relay_B();
			}
			break;
			
	}
}

// Verify commands
// ---------------
void verify_must()
{
	float power;
	
	switch(command_must_ID) {
	
		case CMD_ALTITUDE:
				if (abs(next_WP.alt - current_loc.alt) < 100){
					command_must_index 	= 0;
				}
			break;
		
		case CMD_TAKEOFF:	// Takeoff!
			if (current_loc.alt > (next_WP.alt -100)){
				command_must_index 	= 0;
				takeoff_complete 	= true;
			}
			break;

		case CMD_LAND:
			// 10 - 9  = 1  
			velocity_land  = ((old_alt - current_loc.alt) *.05) + (velocity_land * .95);
			old_alt = current_loc.alt;
			if(velocity_land == 0){
				land_complete 		= true;
				command_must_index 	= 0;
			}
			break;
		
		case CMD_WAYPOINT:	// reach a waypoint
			if ((wp_distance > 0) && (wp_distance <= wp_radius)) {
				Serial.print("MSG REACHED_WAYPOINT #");
				Serial.println(command_must_index,DEC);
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

		case CMD_LOITER_N_TURNS:	// LOITER N times
			break;
			
		case CMD_LOITER_TIME:	// loiter N milliseconds
			break;
			
		case CMD_RTL:
			if (wp_distance <= wp_radius) {
				//Serial.println("REACHED_HOME");
				command_must_index 	= 0;
			}
			break;
			
		case CMD_LOITER:	// Just plain LOITER
			break;

		case CMD_ANGLE:
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

		default:
			send_message(SEVERITY_HIGH,"No current Must commands");
			break;
	}	
}

void verify_may()
{
	switch(command_may_ID) {
		case CMD_DELAY:
			if ((millis() - delay_start) > delay_timeout){
				command_may_index = 0;
				delay_timeout = 0;
			}
			
		case CMD_LAND_OPTIONS:
			break;
	}
}

