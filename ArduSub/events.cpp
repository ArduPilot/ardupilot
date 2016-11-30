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

void Sub::failsafe_internal_pressure_check() {

	if(g.failsafe_pressure == FS_PRESS_DISABLED) {
		return; // Nothing to do
	}

	uint32_t tnow = AP_HAL::millis();
	static uint32_t last_pressure_warn_ms;
	static uint32_t last_pressure_good_ms;
	if(barometer.get_pressure(0) < g.failsafe_pressure_max) {
		last_pressure_good_ms = tnow;
		last_pressure_warn_ms = tnow;
		failsafe.internal_pressure = false;
		return;
	}

	// 2 seconds with no readings below threshold triggers failsafe
	if(tnow > last_pressure_good_ms + 2000) {
		failsafe.internal_pressure = true;
	}

	// Warn every 5 seconds
	if(failsafe.internal_pressure && tnow > last_pressure_warn_ms + 5000) {
		last_pressure_warn_ms = tnow;
		gcs_send_text(MAV_SEVERITY_WARNING, "Internal pressure critical!");
	}
}

void Sub::failsafe_internal_temperature_check() {

	if(g.failsafe_temperature == FS_TEMP_DISABLED) {
		return; // Nothing to do
	}

	uint32_t tnow = AP_HAL::millis();
	static uint32_t last_temperature_warn_ms;
	static uint32_t last_temperature_good_ms;
	if(barometer.get_temperature(0) < g.failsafe_temperature_max) {
		last_temperature_good_ms = tnow;
		last_temperature_warn_ms = tnow;
		failsafe.internal_temperature = false;
		return;
	}

	// 2 seconds with no readings below threshold triggers failsafe
	if(tnow > last_temperature_good_ms + 2000) {
		failsafe.internal_temperature = true;
	}

	// Warn every 5 seconds
	if(failsafe.internal_temperature && tnow > last_temperature_warn_ms + 5000) {
		last_temperature_warn_ms = tnow;
		gcs_send_text(MAV_SEVERITY_WARNING, "Internal temperature critical!");
	}
}

void Sub::set_leak_status(bool status) {
	AP_Notify::flags.leak_detected = status;

	// Do nothing if we are dry, or if leak failsafe action is disabled
	if(status == false || g.failsafe_leak == FS_LEAK_DISABLED) {
		if(failsafe.leak) {
			Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_LEAK, ERROR_CODE_FAILSAFE_RESOLVED);
		}
		failsafe.leak = false;
		return;
	}

	uint32_t tnow = AP_HAL::millis();

	// We have a leak
	// Always send a warning every 5 seconds
	if(tnow > failsafe.last_leak_warn_ms + 5000) {
		failsafe.last_leak_warn_ms = tnow;
		gcs_send_text(MAV_SEVERITY_CRITICAL, "Leak Detected");
	}

	// Do nothing if we have already triggered the failsafe action, or if the motors are disarmed
	if(failsafe.leak) {
		return;
	}

	failsafe.leak = true;
	
	Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_LEAK, ERROR_CODE_FAILSAFE_OCCURRED);

	// Handle failsafe action
	if(failsafe.leak && g.failsafe_leak == FS_LEAK_SURFACE && motors.armed()) {
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
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Failsafe terrain triggered");
            failsafe_terrain_on_event();
        } else {
            Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_TERRAIN, ERROR_CODE_ERROR_RESOLVED);
            failsafe.terrain = false;
        }
    }
}

// This gets called if mission items are in ALT_ABOVE_TERRAIN frame
// Terrain failure occurs when terrain data is not found, or rangefinder is not enabled or healthy
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
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_TERRAIN, ERROR_CODE_FAILSAFE_OCCURRED);

    // If rangefinder is enabled, we can recover from this failsafe
    if(!rangefinder_state.enabled || !auto_terrain_recover_start()) {
    	failsafe_terrain_act();
    }


}

// Recovery failed, take action
void Sub::failsafe_terrain_act() {
    switch (g.failsafe_terrain) {
    case FS_TERRAIN_HOLD:
		if(!set_mode(POSHOLD, MODE_REASON_TERRAIN_FAILSAFE)) {
			set_mode(ALT_HOLD, MODE_REASON_TERRAIN_FAILSAFE);
		}
		AP_Notify::events.failsafe_mode_change = 1;
		break;

    case FS_TERRAIN_SURFACE:
		set_mode(SURFACE, MODE_REASON_TERRAIN_FAILSAFE);
    	AP_Notify::events.failsafe_mode_change = 1;
		break;

    case FS_TERRAIN_DISARM:
    default:
		init_disarm_motors();
    }
}

bool Sub::should_disarm_on_failsafe() {
    switch(control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero OR vehicle is landed disarm motors
            return ap.throttle_zero;
            break;
        case AUTO:
            // if mission has not started AND vehicle is landed, disarm motors
            return !ap.auto_armed;
            break;
        default:
            // used for AltHold, Guided, Loiter, RTL, Circle, Drift, Sport, Flip, Autotune, PosHold
            // if landed disarm
//            return ap.land_complete;
        	return false;
            break;
    }
}

void Sub::update_events()
{
    ServoRelayEvents.update_events();
}

