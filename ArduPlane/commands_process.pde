/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// For changing active command mid-mission
//----------------------------------------
void change_command(uint8_t cmd_index)
{
    struct Location temp = get_cmd_with_index(cmd_index);

    if (temp.id > MAV_CMD_NAV_LAST ) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Bad Request - cannot change to non-Nav cmd"));
    } else {
        gcs_send_text_fmt(PSTR("Received Request - jump to command #%i"),cmd_index);

        nav_command_ID          = NO_COMMAND;
        next_nav_command.id = NO_COMMAND;
        non_nav_command_ID      = NO_COMMAND;

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
    }                                                                           // Other (eg GCS_Auto) modes may be implemented here
}

static void verify_commands(void)
{
    if(verify_nav_command()) {
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
    byte old_index = 0;

    // these are Navigation/Must commands
    // ---------------------------------
    if (nav_command_ID == NO_COMMAND) {    // no current navigation command loaded
        old_index = nav_command_index;
        temp.id = MAV_CMD_NAV_LAST;
        while(temp.id >= MAV_CMD_NAV_LAST && nav_command_index <= g.command_total) {
            nav_command_index++;
            temp = get_cmd_with_index(nav_command_index);
        }

        gcs_send_text_fmt(PSTR("Nav command index updated to #%i"),nav_command_index);

        if(nav_command_index > g.command_total) {
            // we are out of commands!
            gcs_send_text_P(SEVERITY_LOW,PSTR("out of commands!"));
            handle_no_commands();
        } else {
            next_nav_command = temp;
            nav_command_ID = next_nav_command.id;
            non_nav_command_index = NO_COMMAND;                                 // This will cause the next intervening non-nav command (if any) to be loaded
            non_nav_command_ID = NO_COMMAND;

            if (g.log_bitmask & MASK_LOG_CMD) {
                Log_Write_Cmd(g.command_index, &next_nav_command);
            }
            process_nav_cmd();
        }
    }

    // these are Condition/May and Do/Now commands
    // -------------------------------------------
    if (non_nav_command_index == NO_COMMAND) {                  // If the index is NO_COMMAND then we have just loaded a nav command
        non_nav_command_index = old_index + 1;
        //gcs_send_text_fmt(PSTR("Non-Nav command index #%i"),non_nav_command_index);
    } else if (non_nav_command_ID == NO_COMMAND) {      // If the ID is NO_COMMAND then we have just completed a non-nav command
        non_nav_command_index++;
    }

    //gcs_send_text_fmt(PSTR("Nav command index #%i"),nav_command_index);
    //gcs_send_text_fmt(PSTR("Non-Nav command index #%i"),non_nav_command_index);
    //gcs_send_text_fmt(PSTR("Non-Nav command ID #%i"),non_nav_command_ID);
    if(nav_command_index <= (int)g.command_total && non_nav_command_ID == NO_COMMAND) {
        temp = get_cmd_with_index(non_nav_command_index);
        if(temp.id <= MAV_CMD_NAV_LAST) {                       // The next command is a nav command.  No non-nav commands to do
            g.command_index.set_and_save(nav_command_index);
            non_nav_command_index = nav_command_index;
            non_nav_command_ID = WAIT_COMMAND;
            gcs_send_text_fmt(PSTR("Non-Nav command ID updated to #%i"),non_nav_command_ID);

        } else {                                                                        // The next command is a non-nav command.  Prepare to execute it.
            g.command_index.set_and_save(non_nav_command_index);
            next_nonnav_command = temp;
            non_nav_command_ID = next_nonnav_command.id;
            gcs_send_text_fmt(PSTR("Non-Nav command ID updated to #%i"),non_nav_command_ID);

            if (g.log_bitmask & MASK_LOG_CMD) {
                Log_Write_Cmd(g.command_index, &next_nonnav_command);
            }

            process_non_nav_command();
        }

    }
}

/**************************************************/
//  These functions implement the commands.
/**************************************************/
static void process_nav_cmd()
{
    //gcs_send_text_P(SEVERITY_LOW,PSTR("New nav command loaded"));

    // clear non-nav command ID and index
    non_nav_command_index   = NO_COMMAND;               // Redundant - remove?
    non_nav_command_ID              = NO_COMMAND;               // Redundant - remove?

    handle_process_nav_cmd();

}

static void process_non_nav_command()
{
    //gcs_send_text_P(SEVERITY_LOW,PSTR("new non-nav command loaded"));

    if(non_nav_command_ID < MAV_CMD_CONDITION_LAST) {
        handle_process_condition_command();
    } else {
        handle_process_do_command();
        // flag command ID so a new one is loaded
        // -----------------------------------------
        non_nav_command_ID = NO_COMMAND;
    }
}
