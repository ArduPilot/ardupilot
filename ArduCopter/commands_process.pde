/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// For changing active command mid-mission
//----------------------------------------
static void change_command(uint8_t cmd_index)
{
	struct Location temp = get_cmd_with_index(cmd_index);

	if (temp.id > MAV_CMD_NAV_LAST ){
		gcs_send_text_P(SEVERITY_LOW,PSTR("error: non-Nav cmd"));
	} else {
		//Serial.printf("APM:New cmd Index: %d\n", cmd_index);
		command_must_index 	= NO_COMMAND;
		next_command.id 	= NO_COMMAND;
		g.command_index.set_and_save(cmd_index);
		update_commands();
	}
}

// called by 10 Hz loop
// --------------------
static void update_commands(void)
{
	// A: if we do not have any commands there is nothing to do
	// B: We have completed the mission, don't redo the mission
	if (g.command_total <= 1 || g.command_index == 255)
		return;

	// fill command queue with a new command if available
	if(next_command.id == NO_COMMAND){

		// fetch next command if the next command queue is empty
		// -----------------------------------------------------

		/*
		range check - don't grab a command that isn't there
		0 : home command
		1 : first WP
		2 : seconds WP
		3 : third WP

		current g.command_index = 2,
		Now load index 3
		if(2 < (4-1))
		if (2 < 3)
		OK we can safely load the next index 3
		*/

		if (g.command_index < (g.command_total -1)) {
			// load next index
			next_command = get_cmd_with_index(g.command_index + 1);
		}
	}

	// Are we out of must commands and the queue is empty?
	if(next_command.id == NO_COMMAND && command_must_index == NO_COMMAND){
		// if no commands were available from EEPROM
		// And we have no nav commands
		// --------------------------------------------
		if (command_must_ID == NO_COMMAND){
			gcs_send_text_P(SEVERITY_LOW,PSTR("mission complete"));
			handle_no_commands();
			g.command_index = 255;
		}
	}

	// check to see if we need to act on our command queue
	if (process_next_command()){
		//Serial.printf("did PNC: %d\n", next_command.id);

		// We acted on the queue - let's debug that
		// ----------------------------------------
		print_wp(&next_command, g.command_index);

		// invalidate command queue so a new one is loaded
		// -----------------------------------------------
		clear_command_queue();

		// make sure we load the next command index
		// ----------------------------------------
		increment_WP_index();
	}
}

// called with GPS navigation update - not constantly
static void verify_commands(void)
{
	if(verify_must()){
		//Serial.printf("verified must cmd %d\n" , command_must_index);
		command_must_index 	= NO_COMMAND;
	}else{
		//Serial.printf("verified must false %d\n" , command_must_index);
	}

	if(verify_may()){
		//Serial.printf("verified may cmd %d\n" , command_may_index);
		command_may_index 	= NO_COMMAND;
		command_may_ID		= NO_COMMAND;
	}
}

static bool
process_next_command()
{
	// these are Navigation/Must commands
	// ---------------------------------
	if (command_must_index == NO_COMMAND){ // no current command loaded
		if (next_command.id < MAV_CMD_NAV_LAST ){

			// we remember the index of our mission here
			command_must_index = g.command_index + 1;

			// Save CMD to Log
			if (g.log_bitmask & MASK_LOG_CMD)
				Log_Write_Cmd(g.command_index + 1, &next_command);

			// Act on the new command
			process_must();
			return true;
		}
	}

	// these are Condition/May commands
	// ----------------------
	if (command_may_index == NO_COMMAND){
		if (next_command.id > MAV_CMD_NAV_LAST && next_command.id < MAV_CMD_CONDITION_LAST ){

			// we remember the index of our mission here
			command_may_index = g.command_index + 1;

			//SendDebug("MSG <pnc> new may ");
			//SendDebugln(next_command.id,DEC);
			//Serial.print("new command_may_index ");
			//Serial.println(command_may_index,DEC);

			// Save CMD to Log
			if (g.log_bitmask & MASK_LOG_CMD)
				Log_Write_Cmd(g.command_index + 1, &next_command);

			process_may();
			return true;
		}

		// these are Do/Now commands
		// ---------------------------
		if (next_command.id > MAV_CMD_CONDITION_LAST){
			//SendDebug("MSG <pnc> new now ");
			//SendDebugln(next_command.id,DEC);

			if (g.log_bitmask & MASK_LOG_CMD)
				Log_Write_Cmd(g.command_index + 1, &next_command);
			process_now();
			return true;
		}
	}
	// we did not need any new commands
	return false;
}

/**************************************************/
//  These functions implement the commands.
/**************************************************/
static void process_must()
{
	//gcs_send_text_P(SEVERITY_LOW,PSTR("New cmd: <process_must>"));
	//Serial.printf("pmst %d\n", (int)next_command.id);

	// clear May indexes to force loading of more commands
	// existing May commands are tossed.
	command_may_index	= NO_COMMAND;
	command_may_ID		= NO_COMMAND;

	// remember our command ID
	command_must_ID 	= next_command.id;

	// implements the Flight Logic
	handle_process_must();

}

static void process_may()
{
	//gcs_send_text_P(SEVERITY_LOW,PSTR("<process_may>"));
	//Serial.print("pmay");

	command_may_ID = next_command.id;
	handle_process_may();
}

static void process_now()
{
	//Serial.print("pnow");
	handle_process_now();
}
