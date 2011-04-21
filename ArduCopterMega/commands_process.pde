/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// called by 10 Hz Medium loop
// ---------------------------
void update_commands(void)
{
	// This function loads commands into three buffers
	// when a new command is loaded, it is processed with process_XXX()
	// ----------------------------------------------------------------
	if(home_is_set == false){
		return;	// don't do commands
	}

	if(control_mode == AUTO){
		load_next_command_from_EEPROM();
		process_next_command();
	}									// Other (eg GCS_Auto) modes may be implemented here
}

void verify_commands(void)
{
	if(verify_must()){
		command_must_index 	= NO_COMMAND;
	}

	if(verify_may()){
		command_may_index 	= NO_COMMAND;
		command_may_ID		= NO_COMMAND;
	}
}

void load_next_command_from_EEPROM()
{
	// fetch next command if the next command queue is empty
	// -----------------------------------------------------
	if(next_command.id == NO_COMMAND){
		next_command = get_wp_with_index(g.waypoint_index + 1);
		//Serial.println("AA");
		print_wp(&next_command, g.waypoint_index + 1);
	}

	// If the preload failed, return or just Loiter
	// --------------------------------------------
	if(next_command.id == NO_COMMAND){
		Serial.println("lnc_nc");
		// we are out of commands!
		if(handle_no_commands() == true){
			gcs.send_text_P(SEVERITY_LOW,PSTR("out of commands!"));
		}
	}
}

void process_next_command()
{
	// these are Navigation/Must commands
	// ---------------------------------
	if (command_must_index == 0){ // no current command loaded
		if (next_command.id < MAV_CMD_NAV_LAST ){
			increment_WP_index();
			//save_command_index();			// TO DO - fix - to Recover from in air Restart
			command_must_index = g.waypoint_index;

			SendDebug("MSG <pnc> new c_must_id ");
			SendDebug(next_command.id,DEC);
			SendDebug(" index:");

			SendDebugln(command_must_index,DEC);
			if (g.log_bitmask & MASK_LOG_CMD)
				Log_Write_Cmd(g.waypoint_index, &next_command);
			process_must();
		}
	}

	// these are Condition/May commands
	// ----------------------
	if (command_may_index == 0){
		if (next_command.id > MAV_CMD_NAV_LAST && next_command.id < MAV_CMD_CONDITION_LAST ){
			increment_WP_index();		// this command is from the command list in EEPROM
			command_may_index = g.waypoint_index;
				SendDebug("MSG <pnc> new may ");
				SendDebugln(next_command.id,DEC);
				//Serial.print("new command_may_index ");
				//Serial.println(command_may_index,DEC);
			if (g.log_bitmask & MASK_LOG_CMD)
				Log_Write_Cmd(g.waypoint_index, &next_command);
			process_may();
		}

		// these are Do/Now commands
		// ---------------------------
		if (next_command.id > MAV_CMD_CONDITION_LAST){
			increment_WP_index();		// this command is from the command list in EEPROM
				SendDebug("MSG <pnc> new now ");
				SendDebugln(next_command.id,DEC);

			if (g.log_bitmask & MASK_LOG_CMD)
				Log_Write_Cmd(g.waypoint_index, &next_command);
			process_now();
		}
	}
}

/**************************************************/
//  These functions implement the commands.
/**************************************************/
void process_must()
{
	//gcs.send_text_P(SEVERITY_LOW,PSTR("New cmd: <process_must>"));
	//gcs.send_message(MSG_COMMAND_LIST, g.waypoint_index);
	//Serial.printf("pmst %d\n", (int)next_command.id);
	print_wp(&next_command, g.waypoint_index);

	// clear May indexes
	command_may_index	= NO_COMMAND;
	command_may_ID		= NO_COMMAND;

	command_must_ID 	= next_command.id;
	handle_process_must();

	// invalidate command so a new one is loaded
	// -----------------------------------------
	next_command.id		= NO_COMMAND;
}

void process_may()
{
	//gcs.send_text_P(SEVERITY_LOW,PSTR("<process_may>"));
	//gcs.send_message(MSG_COMMAND_LIST, g.waypoint_index);
	Serial.print("pmay");

	command_may_ID = next_command.id;
	handle_process_may();

	// invalidate command so a new one is loaded
	// -----------------------------------------
	next_command.id = NO_COMMAND;
}

void process_now()
{
	Serial.print("pnow");
	handle_process_now();

	// invalidate command so a new one is loaded
	// -----------------------------------------
	next_command.id = NO_COMMAND;

	//gcs.send_text_P(SEVERITY_LOW, PSTR("<process_now>"));
	//gcs.send_message(MSG_COMMAND_LIST, g.waypoint_index);
}

