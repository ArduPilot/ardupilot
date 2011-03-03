// called by 10 Hz loop
// --------------------
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
	}else if(control_mode == GCS_AUTO){
		/*if( there is a command recieved )
			process_next_command();
		*/
	}
}

void verify_commands(void)
{
	if(verify_must()){
		command_must_index 	= NO_COMMAND;
	}

	if(verify_may()){
		command_may_index 	= NO_COMMAND;
	}
}

void load_next_command_from_EEPROM()
{
	// fetch next command if the next command queue is empty
	// -----------------------------------------------------
	if(next_command.id == NO_COMMAND){

		next_command = get_wp_with_index(g.waypoint_index + 1);

		//if(next_command.id != NO_COMMAND){
			//SendDebug("MSG <load_next_command> fetch found new cmd from list at index ");
			//SendDebug((g.waypoint_index + 1),DEC);
			//SendDebug(" with cmd id ");
			//SendDebugln(next_command.id,DEC);
		//}
	}

	// If the preload failed, return or just Loiter
	// generate a dynamic command for RTL
	// --------------------------------------------
	if(next_command.id == NO_COMMAND){
		// we are out of commands!
		gcs.send_text(SEVERITY_LOW,"out of commands!");
		//SendDebug("MSG <load_next_command> out of commands, g.waypoint_index: ");
		//SendDebugln(g.waypoint_index,DEC);
		handle_no_commands();
	}
}

void process_next_command()
{
	// these are waypoint/Must commands
	// ---------------------------------
	if (command_must_index == 0){ // no current command loaded
		if (next_command.id < MAV_CMD_NAV_LAST ){
			increment_WP_index();
			//save_command_index();	// to Recover from in air Restart
			command_must_index = g.waypoint_index;

			//SendDebug("MSG <process_next_command> new command_must_id ");
			//SendDebug(next_command.id,DEC);
			//SendDebug(" index:");
			//SendDebugln(command_must_index,DEC);
			if (g.log_bitmask & MASK_LOG_CMD)
				Log_Write_Cmd(g.waypoint_index, &next_command);
			process_must();
		}
	}

	// these are May commands
	// ----------------------
	if (command_may_index == 0){
		if (next_command.id > MAV_CMD_NAV_LAST && next_command.id < MAV_CMD_CONDITION_LAST ){
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
	if (next_command.id > MAV_CMD_CONDITION_LAST){
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
	//SendDebug("process must index: ");
	//SendDebugln(command_must_index,DEC);

	gcs.send_text(SEVERITY_LOW,"New cmd: <process_must>");
	gcs.send_message(MSG_COMMAND_LIST, g.waypoint_index);

	// clear May indexes
	command_may_index	= 0;
	command_may_ID		= 0;

	command_must_ID = next_command.id;

	// loads the waypoint into Next_WP struct
	// --------------------------------------
	//set_next_WP(&next_command);

	// invalidate command so a new one is loaded
	// -----------------------------------------
	handle_process_must();
	next_command.id = NO_COMMAND;
}

void process_may()
{
	//Serial.print("process_may cmd# ");
	//Serial.println(g.waypoint_index,DEC);
	command_may_ID = next_command.id;

	gcs.send_text(SEVERITY_LOW,"<process_may> New may command loaded:");
	gcs.send_message(MSG_COMMAND_LIST, g.waypoint_index);

	handle_process_may();

	// invalidate command so a new one is loaded
	// -----------------------------------------
	next_command.id = NO_COMMAND;
}

void process_now()
{
	const 	float t5			= 100000.0;
	//Serial.print("process_now cmd# ");
	//Serial.println(g.waypoint_index,DEC);

	gcs.send_text(SEVERITY_LOW, "<process_now> New now command loaded: ");
	gcs.send_message(MSG_COMMAND_LIST, g.waypoint_index);
	handle_process_now();

	// invalidate command so a new one is loaded
	// -----------------------------------------
	next_command.id = NO_COMMAND;
}

