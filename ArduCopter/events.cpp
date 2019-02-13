#include "Copter.h"

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
void Copter::failsafe_radio_on_event()
{
    // if motors are not armed there is nothing to do
    if( !motors->armed() ) {
        return;
    }

    if (should_disarm_on_failsafe()) {
        init_disarm_motors();
    } else {
        if (control_mode == AUTO && g.failsafe_throttle == FS_THR_ENABLED_CONTINUE_MISSION) {
            // continue mission
        } else if (control_mode == LAND &&
                   battery.has_failsafed() &&
                   battery.get_highest_failsafe_priority() <= FAILSAFE_LAND_PRIORITY) {
            // continue landing or other high priority failsafes
        } else {
            if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_RTL) {
                set_mode_RTL_or_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
            } else if (g.failsafe_throttle == FS_THR_ENABLED_CONTINUE_MISSION) {
                set_mode_RTL_or_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
            } else if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL) {
                set_mode_SmartRTL_or_RTL(MODE_REASON_RADIO_FAILSAFE);
            } else if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND) {
                set_mode_SmartRTL_or_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
            } else { // g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND
                set_mode_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
            }
        }
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
void Copter::failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_RESOLVED);
}

void Copter::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

    // failsafe check
    if (should_disarm_on_failsafe()) {
        init_disarm_motors();
    } else {
        switch ((Failsafe_Action)action) {
            case Failsafe_Action_None:
                return;
            case Failsafe_Action_Land:
                set_mode_land_with_pause(MODE_REASON_BATTERY_FAILSAFE);
                break;
            case Failsafe_Action_RTL:
                set_mode_RTL_or_land_with_pause(MODE_REASON_BATTERY_FAILSAFE);
                break;
            case Failsafe_Action_SmartRTL:
                set_mode_SmartRTL_or_RTL(MODE_REASON_BATTERY_FAILSAFE);
                break;
            case Failsafe_Action_SmartRTL_Land:
                set_mode_SmartRTL_or_land_with_pause(MODE_REASON_BATTERY_FAILSAFE);
                break;
            case Failsafe_Action_Terminate:
#if ADVANCED_FAILSAFE == ENABLED
                char battery_type_str[17];
                snprintf(battery_type_str, 17, "%s battery", type_str);
                g2.afs.gcs_terminate(true, battery_type_str);
#else
                init_disarm_motors();
#endif
        }
    }
}

// failsafe_gcs_check - check for ground station failsafe
void Copter::failsafe_gcs_check()
{
    uint32_t last_gcs_update_ms;

    // return immediately if gcs failsafe is disabled, gcs has never been connected or we are not overriding rc controls from the gcs and we are not in guided mode
    // this also checks to see if we have a GCS failsafe active, if we do, then must continue to process the logic for recovery from this state.
    if ((!failsafe.gcs)&&(g.failsafe_gcs == FS_GCS_DISABLED || failsafe.last_heartbeat_ms == 0 || (!RC_Channels::has_active_overrides() && control_mode != GUIDED))) {
        return;
    }

    // calc time since last gcs update
    // note: this only looks at the heartbeat from the device id set by g.sysid_my_gcs
    last_gcs_update_ms = millis() - failsafe.last_heartbeat_ms;

    // check if all is well
    if (last_gcs_update_ms < FS_GCS_TIMEOUT_MS) {
        // check for recovery from gcs failsafe
        if (failsafe.gcs) {
            failsafe_gcs_off_event();
            set_failsafe_gcs(false);
        }
        return;
    }

    // do nothing if gcs failsafe already triggered or motors disarmed
    if (failsafe.gcs || !motors->armed()) {
        return;
    }

    // GCS failsafe event has occurred
    // update state, log to dataflash
    set_failsafe_gcs(true);
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_OCCURRED);

    // clear overrides so that RC control can be regained with radio.
    RC_Channels::clear_overrides();

    if (should_disarm_on_failsafe()) {
        init_disarm_motors();
    } else {
        if (control_mode == AUTO && g.failsafe_gcs == FS_GCS_ENABLED_CONTINUE_MISSION) {
            // continue mission
        } else if (g.failsafe_gcs == FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL) {
            set_mode_SmartRTL_or_RTL(MODE_REASON_GCS_FAILSAFE);
        } else if (g.failsafe_gcs == FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND) {
            set_mode_SmartRTL_or_land_with_pause(MODE_REASON_GCS_FAILSAFE);
        } else { // g.failsafe_gcs == FS_GCS_ENABLED_ALWAYS_RTL
            set_mode_RTL_or_land_with_pause(MODE_REASON_GCS_FAILSAFE);
        }
    }
}

// failsafe_gcs_off_event - actions to take when GCS contact is restored
void Copter::failsafe_gcs_off_event(void)
{
    // log recovery of GCS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_RESOLVED);
}

// executes terrain failsafe if data is missing for longer than a few seconds
//  missing_data should be set to true if the vehicle failed to navigate because of missing data, false if navigation is proceeding successfully
void Copter::failsafe_terrain_check()
{
    // trigger with 5 seconds of failures while in AUTO mode
    bool valid_mode = (control_mode == AUTO || control_mode == GUIDED || control_mode == GUIDED_NOGPS || control_mode == RTL);
    bool timeout = (failsafe.terrain_last_failure_ms - failsafe.terrain_first_failure_ms) > FS_TERRAIN_TIMEOUT_MS;
    bool trigger_event = valid_mode && timeout;

    // check for clearing of event
    if (trigger_event != failsafe.terrain) {
        if (trigger_event) {
            failsafe_terrain_on_event();
        } else {
            Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_TERRAIN, ERROR_CODE_ERROR_RESOLVED);
            failsafe.terrain = false;
        }
    }
}

// set terrain data status (found or not found)
void Copter::failsafe_terrain_set_status(bool data_ok)
{
    uint32_t now = millis();

    // record time of first and latest failures (i.e. duration of failures)
    if (!data_ok) {
        failsafe.terrain_last_failure_ms = now;
        if (failsafe.terrain_first_failure_ms == 0) {
            failsafe.terrain_first_failure_ms = now;
        }
    } else {
        // failures cleared after 0.1 seconds of persistent successes
        if (now - failsafe.terrain_last_failure_ms > 100) {
            failsafe.terrain_last_failure_ms = 0;
            failsafe.terrain_first_failure_ms = 0;
        }
    }
}

// terrain failsafe action
void Copter::failsafe_terrain_on_event()
{
    failsafe.terrain = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL,"Failsafe: Terrain data missing");
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_TERRAIN, ERROR_CODE_FAILSAFE_OCCURRED);

    if (should_disarm_on_failsafe()) {
        init_disarm_motors();
#if MODE_RTL_ENABLED == ENABLED
    } else if (control_mode == RTL) {
        mode_rtl.restart_without_terrain();
#endif
    } else {
        set_mode_RTL_or_land_with_pause(MODE_REASON_TERRAIN_FAILSAFE);
    }
}

// check for gps glitch failsafe
void Copter::gpsglitch_check()
{
    // get filter status
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    bool gps_glitching = filt_status.flags.gps_glitching;

    // log start or stop of gps glitch.  AP_Notify update is handled from within AP_AHRS
    if (ap.gps_glitching != gps_glitching) {
        ap.gps_glitching = gps_glitching;
        if (gps_glitching) {
            Log_Write_Error(ERROR_SUBSYSTEM_GPS, ERROR_CODE_GPS_GLITCH);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch");
        } else {
            Log_Write_Error(ERROR_SUBSYSTEM_GPS, ERROR_CODE_ERROR_RESOLVED);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch cleared");
        }
    }
}

// set_mode_RTL_or_land_with_pause - sets mode to RTL if possible or LAND with 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_RTL_or_land_with_pause(mode_reason_t reason)
{
    // attempt to switch to RTL, if this fails then switch to Land
    if (!set_mode(RTL, reason)) {
        // set mode to land will trigger mode change notification to pilot
        set_mode_land_with_pause(reason);
    } else {
        // alert pilot to mode change
        AP_Notify::events.failsafe_mode_change = 1;
    }
}

// set_mode_SmartRTL_or_land_with_pause - sets mode to SMART_RTL if possible or LAND with 4 second delay before descent starts
// this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_SmartRTL_or_land_with_pause(mode_reason_t reason)
{
    // attempt to switch to SMART_RTL, if this failed then switch to Land
    if (!set_mode(SMART_RTL, reason)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SmartRTL Unavailable, Using Land Mode");
        set_mode_land_with_pause(reason);
    } else {
        AP_Notify::events.failsafe_mode_change = 1;
    }
}

// set_mode_SmartRTL_or_RTL - sets mode to SMART_RTL if possible or RTL if possible or LAND with 4 second delay before descent starts
// this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_SmartRTL_or_RTL(mode_reason_t reason)
{
    // attempt to switch to SmartRTL, if this failed then attempt to RTL
    // if that fails, then land
    if (!set_mode(SMART_RTL, reason)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SmartRTL Unavailable, Trying RTL Mode");
        set_mode_RTL_or_land_with_pause(reason);
    } else {
        AP_Notify::events.failsafe_mode_change = 1;
    }
}

bool Copter::should_disarm_on_failsafe() {
    if (ap.in_arming_delay) {
        return true;
    }

    switch (control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero OR vehicle is landed disarm motors
            return ap.throttle_zero || ap.land_complete;
        case AUTO:
            // if mission has not started AND vehicle is landed, disarm motors
            return !ap.auto_armed && ap.land_complete;
        default:
            // used for AltHold, Guided, Loiter, RTL, Circle, Drift, Sport, Flip, Autotune, PosHold
            // if landed disarm
            return ap.land_complete;
    }
}
