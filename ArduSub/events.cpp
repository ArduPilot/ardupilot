// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
void Sub::failsafe_radio_on_event()
{
    // if motors are not armed there is nothing to do
    if( !motors.armed() ) {
        return;
    }

    if (should_disarm_on_failsafe()) {
        init_disarm_motors();
    } else {
        if (control_mode == AUTO && g.failsafe_throttle == FS_THR_ENABLED_CONTINUE_MISSION) {
            // continue mission
        } else if (control_mode == LAND && g.failsafe_battery_enabled == FS_BATT_LAND && failsafe.battery) {
            // continue landing
        } else {
            if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                set_mode_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
            } else {
                set_mode_RTL_or_land_with_pause(MODE_REASON_RADIO_FAILSAFE);
            }
        }
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

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
    // return immediately if low battery event has already been triggered
    if (failsafe.battery) {
        return;
    }

    // failsafe check
	if (g.failsafe_battery_enabled != FS_BATT_DISABLED && motors.armed()) {
		if (should_disarm_on_failsafe()) {
			init_disarm_motors();
		} else {
			if (g.failsafe_battery_enabled == FS_BATT_RTL || control_mode == AUTO) {
				set_mode_RTL_or_land_with_pause(MODE_REASON_BATTERY_FAILSAFE);
			} else {
				set_mode_land_with_pause(MODE_REASON_BATTERY_FAILSAFE);
			}
		}
	}

    // set the low battery flag
    set_failsafe_battery(true);

    // warn the ground station and log to dataflash
    gcs_send_text(MAV_SEVERITY_WARNING,"Low battery");
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_gcs_check - check for ground station failsafe
void Sub::failsafe_gcs_check()
{
    uint32_t last_gcs_update_ms;

    // return immediately if gcs failsafe is disabled, gcs has never been connected or we are not overriding rc controls from the gcs and we are not in guided mode
    // this also checks to see if we have a GCS failsafe active, if we do, then must continue to process the logic for recovery from this state.
    if ((!failsafe.gcs) && (g.failsafe_gcs == FS_GCS_DISABLED)) {
    	return;
    }

    // calc time since last gcs update
    // note: this only looks at the heartbeat from the device id set by g.sysid_my_gcs
    last_gcs_update_ms = AP_HAL::millis() - failsafe.last_heartbeat_ms;

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
    if (failsafe.gcs || !motors.armed()) {
        return;
    }

    // GCS failsafe event has occured
    // update state, log to dataflash
    set_failsafe_gcs(true);
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_OCCURRED);

    // clear overrides so that RC control can be regained with radio.
    hal.rcin->clear_overrides();
    failsafe.rc_override_active = false;

    // Disarm the motors, pilot has lost all contact/control over vehicle.
    init_disarm_motors();
}

// failsafe_gcs_off_event - actions to take when GCS contact is restored
void Sub::failsafe_gcs_off_event(void)
{
    // log recovery of GCS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_RESOLVED);
}

// set_mode_RTL_or_land_with_pause - sets mode to RTL if possible or LAND with 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Sub::set_mode_RTL_or_land_with_pause(mode_reason_t reason)
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

