// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
void Sub::failsafe_radio_on_event()
{
//    // if motors are not armed there is nothing to do
//    if( !motors.armed() ) {
//        return;
//    }
//
//    if (should_disarm_on_failsafe()) {
//        init_disarm_motors();
//    } else {
//        if (control_mode == AUTO && g.failsafe_throttle == FS_THR_ENABLED_CONTINUE_MISSION) {
//            // continue mission
//        } else if (control_mode == LAND && g.failsafe_battery_enabled == FS_BATT_LAND && failsafe.battery) {
//            // continue landing
//        } else {
//            if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
//                set_mode_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
//            } else {
//                set_mode_RTL_or_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
//            }
//        }
//    }
//
//    // log the error to the dataflash
//    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
void Sub::failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_RESOLVED);
}

void Sub::failsafe_battery_event(void)
{
//    // return immediately if low battery event has already been triggered
//    if (failsafe.battery) {
//        return;
//    }
//
//    // failsafe check
//	if (g.failsafe_battery_enabled != FS_BATT_DISABLED && motors.armed()) {
//		if (should_disarm_on_failsafe()) {
//			init_disarm_motors();
//		} else {
//			if (g.failsafe_battery_enabled == FS_BATT_RTL || control_mode == AUTO) {
//				set_mode_RTL_or_land_with_pause(MODE_REASON_BATTERY_FAILSAFE);
//			} else {
//				set_mode_land_with_pause(MODE_REASON_BATTERY_FAILSAFE);
//			}
//		}
//	}
//
//    // set the low battery flag
//    set_failsafe_battery(true);
//
//    // warn the ground station and log to dataflash
//    gcs_send_text(MAV_SEVERITY_WARNING,"Low battery");
//    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

}

void Sub::set_leak_status(bool status) {
	AP_Notify::flags.leak_detected = status;

	// Do nothing if we are dry, or if leak failsafe action is disabled
	if(status == false || g.failsafe_leak == FS_LEAK_DISABLED) {
		failsafe.leak = false;
		return;
	}

	uint32_t tnow = AP_HAL::millis();

	// We have a leak
	// Always send a warning every 5 seconds
	if(tnow > failsafe.last_leak_warn_ms + 5000) {
		failsafe.last_leak_warn_ms = tnow;
		gcs_send_text(MAV_SEVERITY_WARNING, "Leak Detected");
	}

	// Do nothing if we have already triggered the failsafe action, or if the motors are disarmed
	if(failsafe.leak || !motors.armed()) {
		return;
	}

	failsafe.leak = true;

	// Handle failsafe action
	if(g.failsafe_leak == FS_LEAK_SURFACE && motors.armed()) {
		set_mode(SURFACE, MODE_REASON_LEAK_FAILSAFE);
	}
}

// failsafe_gcs_check - check for ground station failsafe
void Sub::failsafe_gcs_check()
{
    // return immediately if gcs failsafe action is disabled
    // this also checks to see if we have a GCS failsafe active, if we do, then must continue to process the logic for recovery from this state.
    if (!g.failsafe_gcs && g.failsafe_gcs == FS_GCS_DISABLED) {
    	return;
    }

    uint32_t tnow = AP_HAL::millis();

    // Check if we have gotten a GCS heartbeat recently (GCS sysid must match SYSID_MYGCS parameter)
    if (tnow < failsafe.last_heartbeat_ms + FS_GCS_TIMEOUT_MS) {
        // Log event if we are recovering from previous gcs failsafe
        if (failsafe.gcs) {
            Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_RESOLVED);
        }
        failsafe.gcs = false;
        return;
    }

    //////////////////////////////
    // GCS heartbeat has timed out
    //////////////////////////////

	// Send a warning every 5 seconds
	if(tnow > failsafe.last_gcs_warn_ms + 5000) {
		failsafe.last_gcs_warn_ms = tnow;
		gcs_send_text_fmt(MAV_SEVERITY_WARNING, "MYGCS: %d, heartbeat lost", g.sysid_my_gcs);
	}

    // do nothing if we have already triggered the failsafe action, or if the motors are disarmed
    if (failsafe.gcs || !motors.armed()) {
        return;
    }

    // update state, log to dataflash
    failsafe.gcs = true;
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_OCCURRED);

    // handle failsafe action
    if(g.failsafe_gcs == FS_GCS_DISARM) {
    	init_disarm_motors();
    } else if (g.failsafe_gcs == FS_GCS_HOLD && motors.armed()) {
    	set_mode(ALT_HOLD, MODE_REASON_GCS_FAILSAFE);
    } else if (g.failsafe_gcs == FS_GCS_SURFACE && motors.armed()) {
    	set_mode(SURFACE, MODE_REASON_GCS_FAILSAFE);
    }
}

// executes terrain failsafe if data is missing for longer than a few seconds
//  missing_data should be set to true if the vehicle failed to navigate because of missing data, false if navigation is proceeding successfully
void Sub::failsafe_terrain_check()
{
    // trigger with 5 seconds of failures while in AUTO mode
    bool valid_mode = (control_mode == AUTO || control_mode == GUIDED || control_mode == RTL);
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
void Sub::failsafe_terrain_set_status(bool data_ok)
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
void Sub::failsafe_terrain_on_event()
{
    failsafe.terrain = true;
    gcs_send_text(MAV_SEVERITY_CRITICAL,"Failsafe: Terrain data missing");
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_TERRAIN, ERROR_CODE_FAILSAFE_OCCURRED);

    if (should_disarm_on_failsafe()) {
        init_disarm_motors();
    } else if (control_mode == RTL) {
        rtl_restart_without_terrain();
    } else {
        set_mode_RTL_or_land_with_pause(MODE_REASON_TERRAIN_FAILSAFE);
    }
}

// set_mode_RTL_or_land_with_pause - sets mode to RTL if possible or LAND with 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Sub::set_mode_RTL_or_land_with_pause(mode_reason_t reason)
{
//    // attempt to switch to RTL, if this fails then switch to Land
//    if (!set_mode(RTL, reason)) {
//        // set mode to land will trigger mode change notification to pilot
//        set_mode_land_with_pause(reason);
//    } else {
//        // alert pilot to mode change
//        AP_Notify::events.failsafe_mode_change = 1;
//    }
}

bool Sub::should_disarm_on_failsafe() {
    switch(control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero OR vehicle is landed disarm motors
            return ap.throttle_zero || ap.land_complete;
            break;
        case AUTO:
            // if mission has not started AND vehicle is landed, disarm motors
            return !ap.auto_armed && ap.land_complete;
            break;
        default:
            // used for AltHold, Guided, Loiter, RTL, Circle, Drift, Sport, Flip, Autotune, PosHold
            // if landed disarm
            return ap.land_complete;
            break;
    }
}

void Sub::update_events()
{
    ServoRelayEvents.update_events();
}

