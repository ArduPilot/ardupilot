/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// For changing active command mid-mission
//----------------------------------------
void change_command(uint8_t cmd_index)
{
    struct Location temp;

    if (cmd_index == 0) {
        init_commands();
        gcs_send_text_fmt(PSTR("Received Request - reset mission"));
        return;
    }

    temp = get_cmd_with_index(cmd_index);

    if (temp.id > MAV_CMD_NAV_LAST ) {
        gcs_send_text_fmt(PSTR("Cannot change to non-Nav cmd %u"), (unsigned)cmd_index);
    } else {
        gcs_send_text_fmt(PSTR("Received Request - jump to command #%i"),cmd_index);

        nav_command_ID          = NO_COMMAND;
        next_nav_command.id = NO_COMMAND;
        non_nav_command_ID      = NO_COMMAND;

        // If appears that the nav command index is the index of the first nav command from one
        // before the master index?
        nav_command_index       = cmd_index - 1;
        g.command_index.set_and_save(cmd_index);
        update_commands();
    }
}

// called by 10 Hz loop
// --------------------
static void update_commands(void)
{
    if(control_mode == AUTO) {
        if(home_is_set == true && g.command_total > 1) {
            process_next_command();
        }
    }
}

/*
 * Called only from update_navigation() via navigate() in ArduPlane.pde medium loop.
 * At a different phase in medium loop, update_commands() is called.
 */
static void verify_commands(void)
{
    if(verify_nav_command()) {
    	// This is to indicate command-completed. process_next_command() will find the next one.
        nav_command_ID = NO_COMMAND;
    }

    if(verify_condition_command()) {
        non_nav_command_ID = NO_COMMAND;
    }
}

static void process_next_command()
{
    // This function makes sure that we always have a current navigation command
    // and loads conditional or immediate commands if applicable

    struct Location temp;
    uint8_t old_index = nav_command_index;

    // these are Navigation/Must commands
    // ---------------------------------
    if (nav_command_ID == NO_COMMAND) {    // no current navigation command loaded
        temp.id = MAV_CMD_NAV_LAST;
        // Apparently we scan here for a nav. command.
        // And apparently valid indexes start from 1 and end at (including) g.command_total.
        // The scan will always increment nav_command_index if there are more nav. commands
        // (if nav_command_index points to a nav command from the start, it is skipped)
        while(temp.id >= MAV_CMD_NAV_LAST && nav_command_index <= g.command_total) {
            nav_command_index++;
            temp = get_cmd_with_index(nav_command_index);
        }

        gcs_send_text_fmt(PSTR("Nav command index updated to #%i"),nav_command_index);

        if(nav_command_index > g.command_total) {
            // we are out of commands!
            gcs_send_text_P(SEVERITY_LOW,PSTR("out of commands!"));
            // Do the pseudo RTL. This does not actually change control mode. Since this is only 
            // called from AUTO mode we know it is still AUTO.
            handle_no_commands();
        } else {
            next_nav_command = temp;
            // WHY is this stored separately when it is also in the struct?
            nav_command_ID = next_nav_command.id;
            // OK we invalidate the next non nav command (why is the index set to an ID???)
            non_nav_command_index = 0;                                 // This will cause the next intervening non-nav command (if any) to be loaded
            non_nav_command_ID = NO_COMMAND;

            if (g.log_bitmask & MASK_LOG_CMD) {
                Log_Write_Cmd(g.command_index, &next_nav_command);
            }

            // This is the do_xxx dispatcher.
            // Problem: If we have a JUMP nonnav command (maybe others too) with a nav command after that,
            // the nav command fires before the nonnav has a chance (to jump away).
            handle_process_nav_cmd();
        }
    }

    // So one of 3 is now true:
    // 1) We have just completed a nav command and non_nav_command_index is the first command after the completed nav command. non_nav_command_ID is 
    //    the next nav command
    // or
    // 2) We have just completed a nav command and therea are no more commands.
    // 3) We have not completed prv. nav. command.

    // these are Condition/May and Do/Now commands
    // -------------------------------------------
    if (non_nav_command_index == 0) {                   // If the index is 0 then we have just loaded a nav command
        non_nav_command_index = old_index + 1;			// The index after the nav command.
        //gcs_send_text_fmt(PSTR("Non-Nav command index #%i"),non_nav_command_index);
    } else if (non_nav_command_ID == NO_COMMAND) {      
    	// If the ID is NO_COMMAND and the index is nonzero then we have just completed a non-nav command
        non_nav_command_index++;
    }

    // So triggering the search for a new nonnav command was: a) New nav command loaded or b) previous nonnav completed.
    // In both cases, non_nav_command_ID is set to NO_COMMAND. non_nav_command_index has already been set past previous nonnav.
    if (nav_command_index <= (int)g.command_total && non_nav_command_ID == NO_COMMAND) {
        temp = get_cmd_with_index(non_nav_command_index);
        if (temp.id <= MAV_CMD_NAV_LAST) {                       
            // The next command is a nav command.  No non-nav commands to do
        	// Why is this save done here? Apparently this has nothing do do with that.
            g.command_index.set_and_save(nav_command_index);
            
            // And why is the nonnav command index rewound to the previous nav command?
            non_nav_command_index = nav_command_index;
            
            non_nav_command_ID = WAIT_COMMAND;
            gcs_send_text_fmt(PSTR("Non-Nav command ID updated to #%i idx=%u"),
                              (unsigned)non_nav_command_ID, 
                              (unsigned)non_nav_command_index);

        } else {                                                                        
            // The next command is a non-nav command.  Prepare to execute it.
            g.command_index.set_and_save(non_nav_command_index);
            next_nonnav_command = temp;
            
            non_nav_command_ID = next_nonnav_command.id;
            
            gcs_send_text_fmt(PSTR("(2) Non-Nav command ID updated to #%i idx=%u"),
                              (unsigned)non_nav_command_ID, (unsigned)non_nav_command_index);

            if (g.log_bitmask & MASK_LOG_CMD) {
                Log_Write_Cmd(g.command_index, &next_nonnav_command);
            }

            process_non_nav_command();
        }
    }
}

static void process_non_nav_command()
{
    //gcs_send_text_P(SEVERITY_LOW,PSTR("new non-nav command loaded"));

    if(non_nav_command_ID < MAV_CMD_CONDITION_LAST) {
        handle_process_condition_command();
    } else {
        handle_process_do_command();
        // These are the non conditional commands - fire and forget them.
        // Clear command ID so a new one is loaded
        // -----------------------------------------
        if (non_nav_command_ID != WAIT_COMMAND) {
            non_nav_command_ID = NO_COMMAND;
        }
    }
}
