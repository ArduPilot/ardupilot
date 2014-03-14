/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// forward declaration of verify_command to keep compiler happy
static bool verify_command(const AP_Mission::Mission_Command& cmd);

// called by update navigation at 10Hz
// --------------------
static void update_commands(void)
{
    if(control_mode == AUTO) {
        if (home_is_set) {
            if(mission.state() == AP_Mission::MISSION_RUNNING) {
                mission.update();
            } else {
                // auto_rtl_command should have been set to MAV_CMD_NAV_LOITER_UNLIM by exit_mission
                verify_command(auto_rtl_command);
            }
        }
    }
}

