// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// run this at setup on the ground
// -------------------------------
static void init_home()
{
    set_home_is_set(true);

    // copter uses 0 home altitude
    Location loc = gps.location();

    ahrs.set_home(loc);

    inertial_nav.setup_home_position();

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
