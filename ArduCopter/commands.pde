// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// run this at setup on the ground
// -------------------------------

static void init_home()
{
    Location loc;

    if(!ap.home_is_set) {
        ::printf("2\n");
        loc = gps.location();
        set_home_is_set(true);
        ahrs.set_home(loc);
        inertial_nav.setup_home_position();
    } else {
        ::printf("1\n");
        loc.lat = current_loc.lat;
        loc.lng = current_loc.lng;
        loc.alt = 0.0f;
        ahrs.set_home(loc);
    }

    // log new home position which mission library will pull from ahrs
    if (should_log(MASK_LOG_CMD)) {
        AP_Mission::Mission_Command temp_cmd;
        if (mission.read_cmd_from_storage(0, temp_cmd)) {
            Log_Write_Cmd(temp_cmd);
        }
    }

    // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
    scaleLongDown = longitude_scale(loc);
    scaleLongUp   = 1.0f/scaleLongDown;
}
