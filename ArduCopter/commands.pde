// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static int32_t get_RTL_alt()
{
    if(g.rtl_altitude <= 0) {
		return min(current_loc.alt, RTL_ALT_MAX);
    }else if (g.rtl_altitude < current_loc.alt) {
		return min(current_loc.alt, RTL_ALT_MAX);
    }else{
        return g.rtl_altitude;
    }
}

// run this at setup on the ground
// -------------------------------
static void init_home()
{
    set_home_is_set(true);
    ahrs.set_home(g_gps->latitude, g_gps->longitude, 0);

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



