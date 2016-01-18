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

void set_home_is_set(bool b)
{
    // if no change, exit immediately
    if( ap.home_is_set == b )
        return;

    ap.home_is_set 	= b;
    if(b) {
        Log_Write_Event(DATA_SET_HOME);
    }
}

// ---------------------------------------------
void set_auto_armed(bool b)
{
    // if no change, exit immediately
    if( ap.auto_armed == b )
        return;

    ap.auto_armed = b;
    if(b){
        Log_Write_Event(DATA_AUTO_ARMED);
    }
}

// ---------------------------------------------
static void set_failsafe_radio(bool b)
{
    // only act on changes
    // -------------------
    if(failsafe.radio != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.radio = b;

        if (failsafe.radio == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_radio_off_event();
        }else{
            // We've lost radio contact
            // ------------------------
            failsafe_radio_on_event();
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}

// BEV added this one
static void set_failsafe_rc_override(bool b)
{
    // only act on changes
    // -------------------
    if(failsafe.rc_override_fs != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.rc_override_fs = b;

        if (failsafe.rc_override_fs == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_rc_override_off_event();
        }else{
            // We've lost radio contact
            // ------------------------
            failsafe_rc_override_on_event();
        }
    }
}


// ---------------------------------------------
void set_failsafe_battery(bool b)
{
    failsafe.battery = b;
    AP_Notify::flags.failsafe_battery = b;
}


// ---------------------------------------------
static void set_failsafe_gps(bool b)
{
    failsafe.gps = b;

    // update AP_Notify
    AP_Notify::flags.failsafe_gps = b;
}

// ---------------------------------------------
static void set_failsafe_gcs(bool b)
{
    failsafe.gcs = b;
}

// ---------------------------------------------
void set_land_complete(bool b)
{
    // if no change, exit immediately
    if( ap.land_complete == b )
        return;

    if(b){
        Log_Write_Event(DATA_LAND_COMPLETE);
    }else{
        Log_Write_Event(DATA_NOT_LANDED);
    }
    ap.land_complete = b;
}

// ---------------------------------------------

// set land complete maybe flag
void set_land_complete_maybe(bool b)
{
    // if no change, exit immediately
    if (ap.land_complete_maybe == b)
        return;

    if (b) {
        Log_Write_Event(DATA_LAND_COMPLETE_MAYBE);
    }
    ap.land_complete_maybe = b;
}

// ---------------------------------------------

void set_pre_arm_check(bool b)
{
    if(ap.pre_arm_check != b) {
        ap.pre_arm_check = b;
        AP_Notify::flags.pre_arm_check = b;
    }
}

void set_pre_arm_rc_check(bool b)
{
    if(ap.pre_arm_rc_check != b) {
        ap.pre_arm_rc_check = b;
    }
}

