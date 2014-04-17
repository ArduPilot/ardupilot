// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// run this at setup on the ground
// -------------------------------
static void init_home()
{
    set_home_is_set(true);

    ahrs.set_home(gps.location());

    inertial_nav.setup_home_position();

    // log new home position which mission library will pull from ahrs
    if (g.log_bitmask & MASK_LOG_CMD) {
        AP_Mission::Mission_Command temp_cmd;
        if (mission.read_cmd_from_storage(0, temp_cmd)) {
            Log_Write_Cmd(temp_cmd);
        }
    }

    // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
    scaleLongDown = longitude_scale(home);
    scaleLongUp   = 1.0f/scaleLongDown;
}



