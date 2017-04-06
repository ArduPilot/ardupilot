#include "Sub.h"

/*
 * failsafe.cpp
 * Failsafe checks and actions
 */

static bool failsafe_enabled = false;
static uint16_t failsafe_last_mainLoop_count;
static uint32_t failsafe_last_timestamp;
static bool in_failsafe;

// Enable mainloop lockup failsafe
void Sub::mainloop_failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = micros();
}

// Disable mainloop lockup failsafe
// Used when we know we are going to delay the mainloop significantly.
void Sub::mainloop_failsafe_disable()
{
    failsafe_enabled = false;
}

// This function is called from the core timer interrupt at 1kHz.
// This checks that the mainloop is running, and has not locked up.
void Sub::mainloop_failsafe_check()
{
    uint32_t tnow = AP_HAL::micros();

    if (mainLoop_count != failsafe_last_mainLoop_count) {
        // the main loop is running, all is OK
        failsafe_last_mainLoop_count = mainLoop_count;
        failsafe_last_timestamp = tnow;
        if (in_failsafe) {
            in_failsafe = false;
            Log_Write_Error(ERROR_SUBSYSTEM_CPU,ERROR_CODE_FAILSAFE_RESOLVED);
        }
        return;
    }

    if (!in_failsafe && failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors.
        in_failsafe = true;
        // reduce motors to minimum (we do not immediately disarm because we want to log the failure)
        if (motors.armed()) {
            motors.output_min();
        }
        // log an error
        Log_Write_Error(ERROR_SUBSYSTEM_CPU,ERROR_CODE_FAILSAFE_OCCURRED);
    }

    if (failsafe_enabled && in_failsafe && tnow - failsafe_last_timestamp > 1000000) {
        // disarm motors every second
        failsafe_last_timestamp = tnow;
        if (motors.armed()) {
            motors.armed(false);
            motors.output();
        }
    }
}

// Battery failsafe check
// Check the battery voltage and remaining capacity
void Sub::failsafe_battery_check(void)
{
    // Do nothing if the failsafe is disabled, or if we are disarmed
    if (g.failsafe_battery_enabled == FS_BATT_DISABLED || !motors.armed()) {
        failsafe.battery = false;
        AP_Notify::flags.failsafe_battery = false;
        return; // Failsafe disabled, nothing to do
    }

    if (!battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        failsafe.battery = false;
        AP_Notify::flags.failsafe_battery = false;
        return; // Battery is doing well
    }

    // Always warn when failsafe condition is met
    if (AP_HAL::millis() > failsafe.last_battery_warn_ms + 20000) {
        failsafe.last_battery_warn_ms = AP_HAL::millis();
        gcs_send_text(MAV_SEVERITY_WARNING, "Low battery");
    }

    // Don't do anything if failsafe has already been set
    if (failsafe.battery) {
        return;
    }

    failsafe.battery = true;
    AP_Notify::flags.failsafe_battery = true;

    // Log failsafe
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

    switch(g.failsafe_battery_enabled) {
    case FS_BATT_SURFACE:
        set_mode(SURFACE, MODE_REASON_BATTERY_FAILSAFE);
        break;
    case FS_BATT_DISARM:
        init_disarm_motors();
        break;
    default:
        break;
    }
}

// MANUAL_CONTROL failsafe check
// Make sure that we are receiving MANUAL_CONTROL at an appropriate interval
void Sub::failsafe_manual_control_check()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    uint32_t tnow = AP_HAL::millis();

    // Require at least 0.5 Hz update
    if (tnow > failsafe.last_manual_control_ms + 2000) {
        if (!failsafe.manual_control) {
            failsafe.manual_control = true;
            set_neutral_controls();
            init_disarm_motors();
            Log_Write_Error(ERROR_SUBSYSTEM_INPUT, ERROR_CODE_FAILSAFE_OCCURRED);
            gcs_send_text(MAV_SEVERITY_CRITICAL, "Lost manual control");
        }
        return;
    }

    failsafe.manual_control = false;
#endif
}

// Internal pressure failsafe check
// Check if the internal pressure of the watertight electronics enclosure
// has exceeded the maximum specified by the FS_PRESS_MAX parameter
void Sub::failsafe_internal_pressure_check()
{

    if (g.failsafe_pressure == FS_PRESS_DISABLED) {
        return; // Nothing to do
    }

    uint32_t tnow = AP_HAL::millis();
    static uint32_t last_pressure_warn_ms;
    static uint32_t last_pressure_good_ms;
    if (barometer.get_pressure(0) < g.failsafe_pressure_max) {
        last_pressure_good_ms = tnow;
        last_pressure_warn_ms = tnow;
        failsafe.internal_pressure = false;
        return;
    }

    // 2 seconds with no readings below threshold triggers failsafe
    if (tnow > last_pressure_good_ms + 2000) {
        failsafe.internal_pressure = true;
    }

    // Warn every 30 seconds
    if (failsafe.internal_pressure && tnow > last_pressure_warn_ms + 30000) {
        last_pressure_warn_ms = tnow;
        gcs_send_text(MAV_SEVERITY_WARNING, "Internal pressure critical!");
    }
}

// Internal temperature failsafe check
// Check if the internal temperature of the watertight electronics enclosure
// has exceeded the maximum specified by the FS_TEMP_MAX parameter
void Sub::failsafe_internal_temperature_check()
{

    if (g.failsafe_temperature == FS_TEMP_DISABLED) {
        return; // Nothing to do
    }

    uint32_t tnow = AP_HAL::millis();
    static uint32_t last_temperature_warn_ms;
    static uint32_t last_temperature_good_ms;
    if (barometer.get_temperature(0) < g.failsafe_temperature_max) {
        last_temperature_good_ms = tnow;
        last_temperature_warn_ms = tnow;
        failsafe.internal_temperature = false;
        return;
    }

    // 2 seconds with no readings below threshold triggers failsafe
    if (tnow > last_temperature_good_ms + 2000) {
        failsafe.internal_temperature = true;
    }

    // Warn every 30 seconds
    if (failsafe.internal_temperature && tnow > last_temperature_warn_ms + 30000) {
        last_temperature_warn_ms = tnow;
        gcs_send_text(MAV_SEVERITY_WARNING, "Internal temperature critical!");
    }
}

// Check if we are leaking and perform appropiate action
void Sub::failsafe_leak_check()
{
    bool status = leak_detector.get_status();

    AP_Notify::flags.leak_detected = status;

    // Do nothing if we are dry, or if leak failsafe action is disabled
    if (status == false || g.failsafe_leak == FS_LEAK_DISABLED) {
        if (failsafe.leak) {
            Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_LEAK, ERROR_CODE_FAILSAFE_RESOLVED);
        }
        failsafe.leak = false;
        return;
    }

    uint32_t tnow = AP_HAL::millis();

    // We have a leak
    // Always send a warning every 20 seconds
    if (tnow > failsafe.last_leak_warn_ms + 20000) {
        failsafe.last_leak_warn_ms = tnow;
        gcs_send_text(MAV_SEVERITY_CRITICAL, "Leak Detected");
    }

    // Do nothing if we have already triggered the failsafe action, or if the motors are disarmed
    if (failsafe.leak) {
        return;
    }

    failsafe.leak = true;

    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_LEAK, ERROR_CODE_FAILSAFE_OCCURRED);

    // Handle failsafe action
    if (failsafe.leak && g.failsafe_leak == FS_LEAK_SURFACE && motors.armed()) {
        set_mode(SURFACE, MODE_REASON_LEAK_FAILSAFE);
    }
}

// failsafe_gcs_check - check for ground station failsafe
void Sub::failsafe_gcs_check()
{
    // return immediately if we have never had contact with a gcs, or if gcs failsafe action is disabled
    // this also checks to see if we have a GCS failsafe active, if we do, then must continue to process the logic for recovery from this state.
    if (failsafe.last_heartbeat_ms == 0 || (!g.failsafe_gcs && g.failsafe_gcs == FS_GCS_DISABLED)) {
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

    // Send a warning every 30 seconds
    if (tnow > failsafe.last_gcs_warn_ms + 30000) {
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
    if (g.failsafe_gcs == FS_GCS_DISARM) {
        init_disarm_motors();
    } else if (g.failsafe_gcs == FS_GCS_HOLD && motors.armed()) {
        set_mode(ALT_HOLD, MODE_REASON_GCS_FAILSAFE);
    } else if (g.failsafe_gcs == FS_GCS_SURFACE && motors.armed()) {
        set_mode(SURFACE, MODE_REASON_GCS_FAILSAFE);
    }
}

#define CRASH_CHECK_TRIGGER_MS          2000    // 2 seconds inverted indicates a crash
#define CRASH_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees beyond angle max is signal we are inverted

// Check for a crash
// The vehicle is considered crashed if the angle error exceeds a specified limit for more than 2 seconds
void Sub::failsafe_crash_check()
{
    static uint32_t last_crash_check_pass_ms;
    uint32_t tnow = AP_HAL::millis();

    // return immediately if disarmed, or crash checking disabled
    if (!motors.armed() || g.fs_crash_check == FS_CRASH_DISABLED) {
        last_crash_check_pass_ms = tnow;
        failsafe.crash = false;
        return;
    }

    // return immediately if we are not in an angle stabilized flight mode
    if (control_mode == ACRO || control_mode == MANUAL) {
        last_crash_check_pass_ms = tnow;
        failsafe.crash = false;
        return;
    }

    // check for angle error over 30 degrees
    const float angle_error = attitude_control.get_att_error_angle_deg();
    if (angle_error <= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        last_crash_check_pass_ms = tnow;
        failsafe.crash = false;
        return;
    }

    if (tnow < last_crash_check_pass_ms + CRASH_CHECK_TRIGGER_MS) {
        return;
    }

    // Conditions met, we are in failsafe

    // Send warning to GCS
    if (tnow > failsafe.last_crash_warn_ms + 20000) {
        failsafe.last_crash_warn_ms = tnow;
        gcs_send_text(MAV_SEVERITY_WARNING,"Crash detected");
    }

    // Only perform failsafe action once
    if (failsafe.crash) {
        return;
    }

    failsafe.crash = true;
    // log an error in the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_CRASH_CHECK, ERROR_CODE_CRASH_CHECK_CRASH);

    // disarm motors
    if (g.fs_crash_check == FS_CRASH_DISARM) {
        init_disarm_motors();
    }
}

// executes terrain failsafe if data is missing for longer than a few seconds
//  missing_data should be set to true if the vehicle failed to navigate because of missing data, false if navigation is proceeding successfully
void Sub::failsafe_terrain_check()
{
    // trigger with 5 seconds of failures while in AUTO mode
    bool valid_mode = (control_mode == AUTO || control_mode == GUIDED);
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
    if (!rangefinder_state.enabled || !auto_terrain_recover_start()) {
        failsafe_terrain_act();
    }


}

// Recovery failed, take action
void Sub::failsafe_terrain_act()
{
    switch (g.failsafe_terrain) {
    case FS_TERRAIN_HOLD:
        if (!set_mode(POSHOLD, MODE_REASON_TERRAIN_FAILSAFE)) {
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
