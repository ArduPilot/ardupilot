/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// called by 10 Hz Medium loop
// ---------------------------
void update_commands(void)
{
	// This function loads commands into three buffers
	// when a new command is loaded, it is processed with process_XXX()

	// If we have a command in the queue,
	// nothing to do.
	if(next_command.id != NO_COMMAND){
		return;
	}

	// fetch next command if the next command queue is empty
	// -----------------------------------------------------
	next_command = get_command_with_index(g.waypoint_index + 1);

	if(next_command.id == NO_COMMAND){
		// if no commands were available from EEPROM
		// --------------------------------------------

		handle_no_commands();
		gcs.send_text_P(SEVERITY_LOW,PSTR("out of commands!"));

	} else {
		// A command was loaded from EEPROM
		// --------------------------------------------

		// debug by outputing the Waypoint loaded
		print_wp(&next_command, g.waypoint_index + 1);

		// Set our current mission index + 1;
		increment_WP_index();

		// act on our new command
		process_next_command();

	}
}

// called with GPS navigation update - not constantly
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

void process_next_command()
{
	// these are Navigation/Must commands
	// ---------------------------------
	if (command_must_index == NO_COMMAND){ // no current command loaded
		if (next_command.id < MAV_CMD_NAV_LAST ){

			// we remember the index of our mission here
			command_must_index = g.waypoint_index;

			// dubugging output
			SendDebug("MSG <pnc> new c_must_id ");
			SendDebug(next_command.id,DEC);
			SendDebug(" index:");
			SendDebugln(command_must_index,DEC);

			// Save CMD to Log
			if (g.log_bitmask & MASK_LOG_CMD)
				Log_Write_Cmd(g.waypoint_index, &next_command);

			// Act on the new command
			process_must();
		}
	}

	// these are Condition/May commands
	// ----------------------
	if (command_may_index == 0){
		if (next_command.id > MAV_CMD_NAV_LAST && next_command.id < MAV_CMD_CONDITION_LAST ){

			// we remember the index of our mission here
			command_may_index = g.waypoint_index;
			SendDebug("MSG <pnc> new may ");
			SendDebugln(next_command.id,DEC);
			//Serial.print("new command_may_index ");
			//Serial.println(command_may_index,DEC);

			// Save CMD to Log
			if (g.log_bitmask & MASK_LOG_CMD)
				Log_Write_Cmd(g.waypoint_index, &next_command);

			process_may();
		}

		// these are Do/Now commands
		// ---------------------------
		if (next_command.id > MAV_CMD_CONDITION_LAST){
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

	// clear May indexes to force loading of more commands
	// existing May commands are tossed.
	command_may_index	= NO_COMMAND;
	command_may_ID		= NO_COMMAND;

	// remember our command ID
	command_must_ID 	= next_command.id;

	// implements the Flight Logic
	handle_process_must();

	// invalidate command queue so a new one is loaded
	// -----------------------------------------------
	clear_command_queue();
}

void process_may()
{
	//gcs.send_text_P(SEVERITY_LOW,PSTR("<process_may>"));
	//gcs.send_message(MSG_COMMAND_LIST, g.waypoint_index);
	Serial.print("pmay");

	command_may_ID = next_command.id;
	handle_process_may();

	// invalidate command queue so a new one is loaded
	// -----------------------------------------------
	clear_command_queue();
}

void process_now()
{
	Serial.print("pnow");
	handle_process_now();

	// invalidate command queue so a new one is loaded
	// -----------------------------------------------
	clear_command_queue();
}

