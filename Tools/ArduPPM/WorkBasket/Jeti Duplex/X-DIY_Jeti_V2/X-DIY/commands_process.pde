// called by 10 Hz loop
// --------------------
void update_commands(void)
{
	// This function loads commands into three buffers
	// when a new command is loaded, it is processed with process_XXX()
	// ----------------------------------------------------------------
	if((home_is_set == FALSE) || (control_mode != AUTO)){
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
		next_command = get_wp_with_index(wp_index+1);
		if(next_command.id != CMD_BLANK){
			//SendDebug("MSG <load_next_command> fetch found new cmd from list at index ");
			//SendDebug((wp_index+1),DEC);
			//SendDebug(" with cmd id ");
			//SendDebugln(next_command.id,DEC);	
		}
	}
	
	// If the preload failed, return or just Loiter
	// generate a dynamic command for RTL
	// --------------------------------------------
	if(next_command.id == CMD_BLANK){
		// we are out of commands!
		//send_message(SEVERITY_LOW,"out of commands!");
		//SendDebug("MSG <load_next_command> out of commands, wp_index: ");
		//SendDebugln(wp_index,DEC);

		
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
}

void process_next_command()
{
	// these are waypoint/Must commands
	// ---------------------------------
	if (command_must_index == 0){ // no current command loaded
		if (next_command.id >= 0x10 && next_command.id <= 0x1F ){
			increment_WP_index();
			save_command_index();	// to Recover from in air Restart
			command_must_index = wp_index;

			//SendDebug("MSG <process_next_command> new command_must_id ");
			//SendDebug(next_command.id,DEC);
			//SendDebug(" index:");
			//SendDebugln(command_must_index,DEC);
			if (log_bitmask & MASK_LOG_CMD)		
				Log_Write_Cmd(wp_index, &next_command);
			process_must();
		}
	}
	
	// these are May commands
	// ----------------------
	if (command_may_index == 0){
		if (next_command.id >= 0x20 && next_command.id <= 0x2F ){
			increment_WP_index();// this command is from the command list in EEPROM
			command_may_index = wp_index;
			//Serial.print("new command_may_index ");
			//Serial.println(command_may_index,DEC);
			if (log_bitmask & MASK_LOG_CMD)
				Log_Write_Cmd(wp_index, &next_command);
			process_may();
		}
	}
	
	// these are do it now commands
	// ---------------------------
	if (next_command.id >= 0x30){
		increment_WP_index();// this command is from the command list in EEPROM	
		if (log_bitmask & MASK_LOG_CMD)
			Log_Write_Cmd(wp_index, &next_command);				
		process_now();
	}

}

/*
These functions implement the waypoint commands.
*/
void process_must()
{
	//SendDebug("process must index: ");
	//SendDebugln(command_must_index,DEC);

	send_message(SEVERITY_LOW,"New cmd: <process_must>");
	send_message(MSG_COMMAND, wp_index);
	
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
			// pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0 
			takeoff_pitch 	 	= next_command.p1 * 100;
			takeoff_altitude 	= next_WP.alt;		// next_WP.alt is calculated by the set_next_WP command
			next_WP.lat 		= home.lat + 1000;	// so we don't have bad calcs
			next_WP.lng 		= home.lng + 1000;	// so we don't have bad calcs
			break;

		case CMD_WAYPOINT:	// Navigate to Waypoint
			break;

		case CMD_LAND:	// LAND to Waypoint
			break;
			
		case CMD_LOITER:	// Loiter indefinitely
			break;
		
		case CMD_LOITER_N_TURNS:	// Loiter N Times
			loiter_total = next_command.p1 * 360;
			break;

		case CMD_LOITER_TIME:
			loiter_time = millis();
			loiter_time_max = next_command.p1 * 10000; // seconds * 10 * 1000
			break;

		case CMD_RTL:
			break;
	}
}

void process_may()
{
	//Serial.print("process_may cmd# ");
	//Serial.println(wp_index,DEC);	
	command_may_ID = next_command.id;

	// invalidate command so a new one is loaded
	// -----------------------------------------
	next_command.id = 0;
	
	send_message(SEVERITY_LOW,"<process_may> New may command loaded:");
	send_message(MSG_COMMAND, wp_index);
	
	// do the command
	// --------------
	switch(command_may_ID){
	
		case CMD_DELAY:	// Navigate to Waypoint
			delay_start = millis();
			delay_timeout  = next_command.lat;
			break;

		case CMD_LAND_OPTIONS:	// Land this puppy
			//send_message(SEVERITY_LOW,"Landing");
			
			// pitch in deg, airspeed  m/s, throttle %, track WP 1 or 0 
			landing_pitch 		= next_command.lng * 100;
			airspeed_cruise 	= next_command.alt * 100;
			throttle_cruise 	= next_command.lat;
			landing_distance 	= next_command.p1;
			//landing_roll 	= command.lng;

			SendDebug("MSG: throttle_cruise = ");
			SendDebugln(throttle_cruise,DEC);
			break;
			
		default:
			break;
	}
}

void process_now()
{
	const 	float t5			= 100000.0;
	//Serial.print("process_now cmd# ");
	//Serial.println(wp_index,DEC);

	byte id = next_command.id;

	// invalidate command so a new one is loaded
	// -----------------------------------------
	next_command.id = 0;
	
	send_message(SEVERITY_LOW, "<process_now> New now command loaded: ");
	send_message(MSG_COMMAND, wp_index);
		
	// do the command
	// --------------
	switch(id){
	
		//case CMD_AP_MODE:	
			//next_command.p1 = constrain(next_command.p1, MANUAL, LAND);
			//set_mode(next_command.p1);
			//break;

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

		case CMD_AIRSPEED_CRUISE:	
			airspeed_cruise = next_command.p1 * 100;
			// todo save to EEPROM
			break;

		case CMD_THROTTLE_CRUISE:	
			throttle_cruise = next_command.p1;
			// todo save to EEPROM
			break;

		case CMD_RESET_HOME:	
			init_home();
			break;

		case CMD_KP_GAIN:
			kp[next_command.p1] = next_command.alt/t5;
			// todo save to EEPROM
			break;
		case CMD_KI_GAIN:
			ki[next_command.p1] = next_command.alt/t5;
			// todo save to EEPROM
			break;
		case CMD_KD_GAIN:
			kd[next_command.p1] = next_command.alt/t5;
			// todo save to EEPROM
			break;

		case CMD_KI_MAX:	
			integrator_max[next_command.p1] = next_command.alt/t5;
			// todo save to EEPROM
			break;

		case CMD_KFF_GAIN:	
			kff[next_command.p1] = next_command.alt/t5;
			// todo save to EEPROM
			break;

		case CMD_RADIO_TRIM:	
			radio_trim[next_command.p1] = next_command.alt;
			save_EEPROM_trims();
			break;
			
		case CMD_RADIO_MAX:	
			radio_max[next_command.p1] = next_command.alt;
			save_EEPROM_radio_minmax();
			break;
			
		case CMD_RADIO_MIN:	
			radio_min[next_command.p1] = next_command.alt;
			save_EEPROM_radio_minmax();
			break;

		case CMD_REPEAT:	
			new_event(&next_command);
			break;

		case CMD_SERVO:
			//Serial.print("CMD_SERVO ");
			//Serial.print(next_command.p1,DEC);
			//Serial.print(" PWM: ");
			//Serial.println(next_command.alt,DEC);
			APM_RC.OutputCh(next_command.p1, next_command.alt);
			break;
			
		case CMD_INDEX:
			command_must_index = 0;
			command_may_index = 0;
			wp_index = next_command.p1 - 1;
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
	
		case CMD_TAKEOFF:	// Takeoff!
			//Serial.print("verify_must cmd# ");
			//Serial.println(command_must_index,DEC);

			if (GPS.ground_speed > 3){
				if(hold_course == -1){
					// save our current course to land
					hold_course = yaw_sensor;
				}
			}
			if(hold_course > -1){
				// recalc bearing error with hold_course;
				nav_bearing = hold_course;
				// recalc bearing error
				calc_bearing_error();
			}
			if (current_loc.alt > (home.alt + takeoff_altitude))  {
				command_must_index 	= 0;
				hold_course = -1;
			}
			break;



		case CMD_LAND:
			// we don't verify landing - we never go to a new Must command after Land.
			if ( ((wp_distance > 0) && (wp_distance <= (2*GPS.ground_speed/100))) || (current_loc.alt <= next_WP.alt + 300) )
			{
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

			break;
		
		case CMD_WAYPOINT:	// reach a waypoint
			hold_course == -1;
			update_crosstrack();
			if ((wp_distance > 0) && (wp_distance <= wp_radius)) {
				SendDebug("MSG <verify_must: CMD_WAYPOINT> REACHED_WAYPOINT #");
				SendDebugln(command_must_index,DEC);
				// clear the command queue;
				command_must_index 	= 0;
			}
			// add in a more complex case
			// Doug to do
			if(loiter_sum > 300){
				send_message(SEVERITY_MEDIUM,"<verify_must: CMD_WAYPOINT> Missed WP");
				command_must_index 	= 0;
			}
			break;

		case CMD_LOITER_N_TURNS:	// LOITER N times
			if (wp_distance <= loiter_radius){
				nav_bearing -= 9000;
				
			}else if (wp_distance < (loiter_radius + LOITER_RANGE)){
				power = -((float)(wp_distance - loiter_radius - LOITER_RANGE) / LOITER_RANGE);
				power = constrain(power, 0, 1);
				nav_bearing -= power * 9000;
				
			}else{
				update_crosstrack();
			}
			if(loiter_sum > loiter_total) {
				loiter_total = 0;
				send_message(SEVERITY_LOW,"<verify_must: CMD_LOITER_N_TURNS> LOITER orbits complete ");
				// clear the command queue;
				command_must_index 	= 0;
			}
			// recalc bearing error
			nav_bearing = wrap_360(nav_bearing);
			calc_bearing_error();
			break;
			
		case CMD_LOITER_TIME:	// loiter N milliseconds

			if (wp_distance <= loiter_radius){
				nav_bearing -= 9000;
				
			}else if (wp_distance < (loiter_radius + LOITER_RANGE)){
				power = -((float)(wp_distance - loiter_radius - LOITER_RANGE) / LOITER_RANGE);
				power = constrain(power, 0, 1);
				nav_bearing -= power * 9000;
				
			}else{
				update_crosstrack();
			}

			if ((millis() - loiter_time) > loiter_time_max) {
				send_message(SEVERITY_LOW,"<verify_must: CMD_LOITER_TIME> LOITER time complete ");
				command_must_index 	= 0;
			}
			nav_bearing = wrap_360(nav_bearing);
			// recalc bearing error
			calc_bearing_error();
			break;
			
		case CMD_RTL:
			if (wp_distance <= 30) {
				//Serial.println("REACHED_HOME");
				command_must_index 	= 0;
			}
			break;
			
		case CMD_LOITER:	// Just plain LOITER
			break;			
			
		default:
			send_message(SEVERITY_HIGH,"<verify_must: default> No current Must commands");
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
			if ((wp_distance > 0) && (wp_distance <= landing_distance)) {
				//Serial.print("XXX REACHED_WAYPOINT #");
				//Serial.println(command_must_index,DEC);
				// clear the command queue;
				command_may_index 	= 0;
			}
			break;
	}
}

