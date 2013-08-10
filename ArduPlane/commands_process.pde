/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// For changing active command mid-mission
//----------------------------------------
void change_waypoint(uint8_t new_waypoint_index)
{
    if (mission.change_waypoint_index(new_waypoint_index)) {
        process_waypoint();
    } else {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Error Changing Command!"));
    }
}

static void verify_commands(void)
{
   if (verify_nav_command()) {
        if (control_mode == AUTO) {
            if (mission.increment_waypoint_index()) {
                process_waypoint();
            } else {
                handle_no_commands();
            }
        }
    }
    
    if (verify_condition_command() || non_nav_command_ID == NO_COMMAND) {
        if(mission.get_new_cmd(next_nonnav_command) && control_mode == AUTO) {
            process_non_nav_command();
        } else {
            non_nav_command_ID=NO_COMMAND;
        }
    }
}

static void process_waypoint(void) 
{
    gcs_send_text_fmt(PSTR("Nav command index updated to #%i"),mission.waypoint_index());
    
    if (g.log_bitmask & MASK_LOG_CMD) {
        Log_Write_Cmd(mission.waypoint_index(), &mission.current_wp());
    }
    if(control_mode == AUTO) {
        non_nav_command_ID=NO_COMMAND;
        handle_process_nav_cmd();
    }
}

static void process_non_nav_command()
{
    non_nav_command_ID=next_nonnav_command.id;
    gcs_send_text_fmt(PSTR("Non-Nav command ID updated to #%i idx=%u"),
                  (unsigned)non_nav_command_ID, 
                  (unsigned)mission.command_index());            
    
    if (g.log_bitmask & MASK_LOG_CMD) {
        Log_Write_Cmd(mission.command_index(), &next_nonnav_command);
    }                          
    
    if(non_nav_command_ID < MAV_CMD_CONDITION_LAST) {
        handle_process_condition_command();
    } else {
        handle_process_do_command();

        non_nav_command_ID=NO_COMMAND;
    }
}
