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

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
static void failsafe_radio_on_event()
{
    // if motors are not armed there is nothing to do
    if( !motors.armed() ) {
        return;
    }

    // This is how to handle a failsafe.
    switch(control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero OR vehicle is landed disarm motors
            if (g.rc_3.control_in == 0 || ap.land_complete) {
                init_disarm_motors();

            // if failsafe_throttle is FS_THR_ENABLED_ALWAYS_LAND then land immediately
            }else if( (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) || (!ap.home_is_set) ) { //BEV enter land if home is not set yet
                set_mode_land_with_pause();

            // if far from home then RTL
            }else if(home_distance > wp_nav.get_wp_radius()) {
                if (!set_mode(RTL)) {
                    set_mode_land_with_pause();
                }

            // We have no GPS or are very close to home so we will land
            }else{
                set_mode_land_with_pause();
            }
            break;

        case AUTO:
            // if mission has not started AND vehicle is landed, disarm motors
            if (!ap.auto_armed && ap.land_complete) {
                init_disarm_motors();

            // if failsafe_throttle is FS_THR_ENABLED_ALWAYS_LAND then land immediately
            } else if(g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                set_mode_land_with_pause();

            // if failsafe_throttle is FS_THR_ENABLED_ALWAYS_RTL do RTL
            } else if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_RTL) {
                if(home_distance > wp_nav.get_wp_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode_land_with_pause();
                    }
                }else{
                    // We are very close to home so we will land
                    set_mode_land_with_pause();
                }
            }
            // failsafe_throttle must be FS_THR_ENABLED_CONTINUE_MISSION so no need to do anything
            break;

        case LAND:
            // continue to land if battery failsafe is also active otherwise fall through to default handling
            if (g.failsafe_battery_enabled == FS_BATT_LAND && failsafe.battery) {
                break;
            }
            // no break
        default:
            // used for AltHold, Guided, Loiter, RTL, Circle, OF_Loiter, Drift, Sport, Flip, Autotune, PosHold
            // if landed disarm
            if (ap.land_complete) {
                init_disarm_motors();

            // if failsafe_throttle is FS_THR_ENABLED_ALWAYS_LAND then land immediately
            } else if(g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                set_mode_land_with_pause();

            // if far from home then RTL
            }else if(home_distance > wp_nav.get_wp_radius()) {
                if (!set_mode(RTL)){
                    // if RTL fails because of no GPS, then LAND
                    set_mode_land_with_pause();
                }
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode_land_with_pause();
            }
            break;
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
static void failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_RESOLVED);
}

//BEV added this one
static void failsafe_rc_override_check()
{
    uint32_t last_rc_override_update_ms;

    // return immediately if never received override messages or not presently overriding
    if(failsafe.last_rc_override_ms == 0 || !failsafe.rc_override_active) {
        return;
    }

    // calc time since last gcs update
    last_rc_override_update_ms = millis() - failsafe.last_rc_override_ms;

    // check if all is well
    if( last_rc_override_update_ms < FS_RADIO_RC_OVERRIDE_TIMEOUT_MS) {
        // check for recovery
        if (failsafe.rc_override_fs) {
            failsafe_rc_override_off_event();
            set_failsafe_rc_override(false);
        }
        return;
    }

    // do nothing if failsafe already triggered or motors disarmed
    if( failsafe.rc_override_fs || !motors.armed()) {
        return;
    }

    if(last_rc_override_update_ms > FS_RADIO_RC_OVERRIDE_TIMEOUT_MS)
    {
        failsafe_rc_override_on_event();
        set_failsafe_rc_override(true);
    }
}

//BEV added this one
static void failsafe_rc_override_on_event()
{
    // if motors are not armed there is nothing to do
    if( !motors.armed() ) {
        return;
    }

    // used for AltHold, Guided, Loiter, RTL, Circle, OF_Loiter, Drift, Sport, Flip, Autotune, PosHold
    // if landed disarm
    if (ap.land_complete) {
        init_disarm_motors();

    // if far from home then RTL
    }else if(home_distance > wp_nav.get_wp_radius()) {
        if (!set_mode(RTL)){
            // if RTL fails because of no GPS, then LAND
            set_mode_land_with_pause();
        }
    }else{
        // We have no GPS or are very close to home so we will land
        set_mode_land_with_pause();
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RC_OVERRIDE, ERROR_CODE_FAILSAFE_OCCURRED);
}

//bev added this one
static void failsafe_rc_override_off_event()
{
    //log the error as resolved
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RC_OVERRIDE, ERROR_CODE_FAILSAFE_RESOLVED);
}

static void failsafe_battery_event(void)
{
    // return immediately if low battery event has already been triggered
    if (failsafe.battery) {
        return;
    }

    // failsafe check
    if (g.failsafe_battery_enabled != FS_BATT_DISABLED && motors.armed()) {
        switch(control_mode) {
            case STABILIZE:
            case ACRO:
                // if throttle is zero OR vehicle is landed disarm motors
                if (g.rc_3.control_in == 0 || ap.land_complete) {
                    init_disarm_motors();
                }else{
                    // set mode to RTL or LAND
                    if (g.failsafe_battery_enabled == FS_BATT_RTL && home_distance > wp_nav.get_wp_radius()) {
                        if (!set_mode(RTL)) {
                            set_mode_land_with_pause();
                        }
                    }else{
                        set_mode_land_with_pause();
                    }
                }
                break;
            case AUTO:
                // if mission has not started AND vehicle is landed, disarm motors
                if (!ap.auto_armed && ap.land_complete) {
                    init_disarm_motors();

                // set mode to RTL or LAND
                } else if (home_distance > wp_nav.get_wp_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode_land_with_pause();
                    }
                } else {
                    set_mode_land_with_pause();
                }
                break;
            default:
                // used for AltHold, Guided, Loiter, RTL, Circle, OF_Loiter, Drift, Sport, Flip, Autotune, PosHold
                // if landed disarm
                if (ap.land_complete) {
                    init_disarm_motors();

                // set mode to RTL or LAND
                } else if (g.failsafe_battery_enabled == FS_BATT_RTL && home_distance > wp_nav.get_wp_radius()) {
                    if (!set_mode(RTL)) {
                        set_mode_land_with_pause();
                    }
                } else {
                    set_mode_land_with_pause();
                }
                break;
        }
    }

    // set the low battery flag
    set_failsafe_battery(true);

    // warn the ground station and log to dataflash
    gcs_send_text_P(SEVERITY_LOW,PSTR("Low Battery!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_gps_check - check for gps failsafe
static void failsafe_gps_check()
{
    uint32_t last_gps_update_ms;

    // return immediately if gps failsafe is disabled or we have never had GPS lock
    if (g.failsafe_gps_enabled == FS_GPS_DISABLED || !ap.home_is_set) {
        // if we have just disabled the gps failsafe, ensure the gps failsafe event is cleared
        if (failsafe.gps) {
            failsafe_gps_off_event();
            set_failsafe_gps(false);
        }
        return;
    }

    // calc time since last gps update
    last_gps_update_ms = millis() - gps_glitch.last_good_update();

    // check if all is well
    if( last_gps_update_ms < FAILSAFE_GPS_TIMEOUT_MS) {
        // check for recovery from gps failsafe
        if( failsafe.gps ) {
            failsafe_gps_off_event();
            set_failsafe_gps(false);
        }
        return;
    }

    // do nothing if gps failsafe already triggered or motors disarmed
    if( failsafe.gps || !motors.armed()) {
        return;
    }

    // GPS failsafe event has occured
    // update state, warn the ground station and log to dataflash
    set_failsafe_gps(true);
    gcs_send_text_P(SEVERITY_LOW,PSTR("Lost GPS!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_OCCURRED);

    // take action based on flight mode and FS_GPS_ENABLED parameter
    if (mode_requires_GPS(control_mode) || g.failsafe_gps_enabled == FS_GPS_LAND_EVEN_STABILIZE) {
        if (g.failsafe_gps_enabled == FS_GPS_ALTHOLD && !failsafe.radio) {
            set_mode(ALT_HOLD);
        }else{
            set_mode_land_with_pause();
        }
    }

    // if flight mode is LAND ensure it's not the GPS controlled LAND
    if (control_mode == LAND) {
        land_do_not_use_GPS();
    }
}

// failsafe_gps_off_event - actions to take when GPS contact is restored
static void failsafe_gps_off_event(void)
{
    // log recovery of GPS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GPS, ERROR_CODE_FAILSAFE_RESOLVED);
}

// failsafe_gcs_check - check for ground station failsafe
static void failsafe_gcs_check()
{
    uint32_t last_gcs_update_ms;

    // return immediately if gcs failsafe is disabled, gcs has never been connected or we are not overriding rc controls from the gcs
    //BEV always enter failsafe if relying on rc_override and GCS is lost
    if((!g.failsafe_gcs) || (failsafe.last_heartbeat_ms == 0)) {
        return;
    }

    // calc time since last gcs update
    last_gcs_update_ms = millis() - failsafe.last_heartbeat_ms;

    // check if all is well
    if( (failsafe.gcs) && (last_gcs_update_ms < FS_GCS_TIMEOUT_MS) ) {
        failsafe_gcs_off_event();
        set_failsafe_gcs(false);
        return;
    }

    // do nothing if gcs failsafe already triggered or motors disarmed
    if( failsafe.gcs || !motors.armed()) {
        return;
    }

    //dont allow in stabilize or ALtHold
    //also reset the heartbeat counter so failsafe isn't triggered immediately on entry to another mode
    if(control_mode <= ALT_HOLD) {
        failsafe.last_heartbeat_ms = 0;
        return;
    }

    if(last_gcs_update_ms < FS_GCS_TIMEOUT_MS) {
        return;
    }//otherwise trigger failsafe

    // GCS failsafe event has occured
    // update state, log to dataflash
    set_failsafe_gcs(true);
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_OCCURRED);

    // clear overrides so that RC control can be regained with radio.
    hal.rcin->clear_overrides();
    failsafe.rc_override_active = false;

    // This is how to handle a failsafe.
    if(home_distance > wp_nav.get_wp_radius()) {
        if (!set_mode(RTL)) {
            set_mode_land_with_pause();
        }
    }else{
        // We have no GPS or are very close to home so we will land
        set_mode_land_with_pause();
    }
}

// failsafe_gcs_off_event - actions to take when GCS contact is restored
static void failsafe_gcs_off_event(void)
{
    // log recovery of GCS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_RESOLVED);
}

static void update_events()
{
    //ServoRelayEvents.update_events();
}

