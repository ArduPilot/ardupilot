/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// For changing active command mid-mission
//----------------------------------------
static void change_command(uint8_t cmd_index)
{
	//Serial.printf("change_command: %d\n",cmd_index );
	// limit range
	cmd_index = min(g.command_total-1, cmd_index);

	// load command
	struct Location temp = get_cmd_with_index(cmd_index);

	//Serial.printf("loading cmd: %d with id:%d\n", cmd_index, temp.id);

	// verify it's a nav command
	if (temp.id > MAV_CMD_NAV_LAST ){
		//gcs_send_text_P(SEVERITY_LOW,PSTR("error: non-Nav cmd"));

	} else {
		// clear out command queue
		init_commands();

		// copy command to the queue
		command_nav_queue		= temp;
		command_nav_index 		= cmd_index;
		execute_nav_command();
	}
}

// called by 10 Hz loop
// --------------------
static void update_commands()
{
	//Serial.printf("update_commands: %d\n",increment );
	// A: if we do not have any commands there is nothing to do
	// B: We have completed the mission, don't redo the mission
	if (g.command_total <= 1 || g.command_index == 255)
		return;

	if(command_nav_queue.id == NO_COMMAND){
		// Our queue is empty
		// fill command queue with a new command if available, or exit mission
		// -------------------------------------------------------------------
		if (command_nav_index < (g.command_total -1)) {

			command_nav_index++;
			command_nav_queue = get_cmd_with_index(command_nav_index);

			if (command_nav_queue.id <= MAV_CMD_NAV_LAST ){
				execute_nav_command();
			} else{
				// this is a conditional command so we skip it
				command_nav_queue.id = NO_COMMAND;
			}
		}else{
			command_nav_index = 255;
		}
	}

	if(command_cond_queue.id == NO_COMMAND){
		// Our queue is empty
		// fill command queue with a new command if available, or do nothing
		// -------------------------------------------------------------------

		// no nav commands completed yet
		if (prev_nav_index == NO_COMMAND)
			return;

		if (command_cond_index >= command_nav_index){
			// don't process the fututre
			//command_cond_index = NO_COMMAND;
			return;

		}else if (command_cond_index == NO_COMMAND){
			// start from scratch
			// look at command after the most recent completed nav
			command_cond_index = prev_nav_index + 1;

		}else{
			// we've completed 1 cond, look at next command for another
			command_cond_index++;
		}

		if(command_cond_index < (g.command_total -2)){
			// we're OK to load a new command (last command must be a nav command)
			command_cond_queue = get_cmd_with_index(command_cond_index);

			if (command_cond_queue.id > MAV_CMD_CONDITION_LAST){
				// this is a do now command
				process_now_command();

				// clear command queue
				command_cond_queue.id = NO_COMMAND;

			}else if (command_cond_queue.id > MAV_CMD_NAV_LAST ){
				// this is a conditional command
				process_cond_command();

			}else{
				// this is a nav command, don't process
				// clear the command conditional queue and index
				prev_nav_index			= NO_COMMAND;
				command_cond_index 		= NO_COMMAND;
				command_cond_queue.id 	= NO_COMMAND;
			}

		}
	}
}

static void execute_nav_command(void)
{
	// This is what we report to MAVLINK
	g.command_index  = command_nav_index;

	// Save CMD to Log
	if (g.log_bitmask & MASK_LOG_CMD)
		Log_Write_Cmd(g.command_index, &command_nav_queue);

	// Act on the new command
	process_nav_command();

	// clear navigation prameters
	reset_nav();

	// clear May indexes to force loading of more commands
	// existing May commands are tossed.
	command_cond_index	= NO_COMMAND;
}

// called with GPS navigation update - not constantly
static void verify_commands(void)
{
	if(verify_must()){
		//Serial.printf("verified must cmd %d\n" , command_nav_index);
		command_nav_queue.id = NO_COMMAND;

		// store our most recent executed nav command
		prev_nav_index = command_nav_index;

		// Wipe existing conditionals
		command_cond_index 		= NO_COMMAND;
		command_cond_queue.id 	= NO_COMMAND;

	}else{
		//Serial.printf("verified must false %d\n" , command_nav_index);
	}

	if(verify_may()){
		//Serial.printf("verified may cmd %d\n" , command_cond_index);
		command_cond_queue.id = NO_COMMAND;
	}
}
