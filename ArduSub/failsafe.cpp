#include "Sub.h"

/*
 * failsafe.cpp
 * Failsafe checks and actions
 */

static bool failsafe_enabled = false;
static uint16_t failsafe_last_ticks;
static uint32_t failsafe_last_timestamp;
static bool in_failsafe;

// Enable mainloop lockup failsafe
void Sub::mainloop_failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = AP_HAL::micros();
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

    const uint16_t ticks = scheduler.ticks();
    if (ticks != failsafe_last_ticks) {
        // the main loop is running, all is OK
        failsafe_last_ticks = ticks;
        failsafe_last_timestamp = tnow;
        if (in_failsafe) {
            in_failsafe = false;
            LOGGER_WRITE_ERROR(LogErrorSubsystem::CPU,LogErrorCode::FAILSAFE_RESOLVED);
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
        LOGGER_WRITE_ERROR(LogErrorSubsystem::CPU,LogErrorCode::FAILSAFE_OCCURRED);
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

void Sub::failsafe_sensors_check()
{
    if (!ap.depth_sensor_present) {
        return;
    }

    // We need a depth sensor to do any sort of auto z control
    if (sensor_health.depth) {
        if (failsafe.sensor_health) {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_SENSORS, LogErrorCode::ERROR_RESOLVED);
            failsafe.sensor_health = false;
        }
        return;
    }

    // only report once
    if (failsafe.sensor_health) {
        return;
    }

    failsafe.sensor_health = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Depth sensor error!");
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_SENSORS, LogErrorCode::BAD_DEPTH);

    if (control_mode == Mode::Number::ALT_HOLD || control_mode == Mode::Number::SURFACE || sub.flightmode->requires_GPS()) {
        // This should always succeed
        if (!set_mode(Mode::Number::MANUAL, ModeReason::BAD_DEPTH)) {
            // We should never get here
            arming.disarm(AP_Arming::Method::BADFLOWOFCONTROL);
        }
    }
}

void Sub::failsafe_ekf_check()
{
    static uint32_t last_ekf_good_ms = 0;

    if (g.fs_ekf_action == FS_EKF_ACTION_DISABLED) {
        last_ekf_good_ms = AP_HAL::millis();
        failsafe.ekf = false;
        AP_Notify::flags.ekf_bad = false;
        return;
    }

    float posVar, hgtVar, tasVar;
    Vector3f magVar;
    float compass_variance;
    float vel_variance;
    ahrs.get_variances(vel_variance, posVar, hgtVar, magVar, tasVar);
    compass_variance = magVar.length();

    if (compass_variance < g.fs_ekf_thresh && vel_variance < g.fs_ekf_thresh) {
        last_ekf_good_ms = AP_HAL::millis();
        failsafe.ekf = false;
        AP_Notify::flags.ekf_bad = false;
        return;
    }

    // Bad EKF for 2 solid seconds triggers failsafe
    if (AP_HAL::millis() < last_ekf_good_ms + 2000) {
        failsafe.ekf = false;
        AP_Notify::flags.ekf_bad = false;
        return;
    }

    // Only trigger failsafe once
    if (failsafe.ekf) {
        return;
    }

    failsafe.ekf = true;
    AP_Notify::flags.ekf_bad = true;

    LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_BAD_VARIANCE);

    if (AP_HAL::millis() > failsafe.last_ekf_warn_ms + 20000) {
        failsafe.last_ekf_warn_ms = AP_HAL::millis();
        gcs().send_text(MAV_SEVERITY_WARNING, "EKF bad");
    }

    if (g.fs_ekf_action == FS_EKF_ACTION_DISARM) {
        arming.disarm(AP_Arming::Method::EKFFAILSAFE);
    }
}

// Battery failsafe handler
void Sub::handle_battery_failsafe(const char* type_str, const int8_t action)
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);

    switch((Failsafe_Action)action) {
        case Failsafe_Action_Surface:
            set_mode(Mode::Number::SURFACE, ModeReason::BATTERY_FAILSAFE);
            break;
        case Failsafe_Action_Disarm:
            arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
            break;
        case Failsafe_Action_Warn:
        case Failsafe_Action_None:
            break;
    }
}

// Make sure that we are receiving pilot input at an appropriate interval
void Sub::failsafe_pilot_input_check()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    if (g.failsafe_pilot_input == FS_PILOT_INPUT_DISABLED) {
        failsafe.pilot_input = false;
        return;
    }

    if (AP_HAL::millis() < failsafe.last_pilot_input_ms + g.failsafe_pilot_input_timeout * 1000.0f) {
        failsafe.pilot_input = false; // We've received an update from the pilot within the timeout period
        return;
    }

    if (failsafe.pilot_input) {
        return; // only act once
    }

    failsafe.pilot_input = true;

    LOGGER_WRITE_ERROR(LogErrorSubsystem::PILOT_INPUT, LogErrorCode::FAILSAFE_OCCURRED);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Lost manual control");

    set_neutral_controls();

    if(g.failsafe_pilot_input == FS_PILOT_INPUT_DISARM) {
        arming.disarm(AP_Arming::Method::PILOT_INPUT_FAILSAFE);
    }
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
        gcs().send_text(MAV_SEVERITY_WARNING, "Internal pressure critical!");
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
        gcs().send_text(MAV_SEVERITY_WARNING, "Internal temperature critical!");
    }
}

// Check if we are leaking and perform appropriate action
void Sub::failsafe_leak_check()
{
    bool status = leak_detector.get_status();

    // Do nothing if we are dry, or if leak failsafe action is disabled
    if (status == false || g.failsafe_leak == FS_LEAK_DISABLED) {
        if (failsafe.leak) {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_LEAK, LogErrorCode::FAILSAFE_RESOLVED);
        }
        AP_Notify::flags.leak_detected = false;
        failsafe.leak = false;
        return;
    }

    AP_Notify::flags.leak_detected = status;

    uint32_t tnow = AP_HAL::millis();

    // We have a leak
    // Always send a warning every 20 seconds
    if (tnow > failsafe.last_leak_warn_ms + 20000) {
        failsafe.last_leak_warn_ms = tnow;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Leak Detected");
    }

    // Do nothing if we have already triggered the failsafe action, or if the motors are disarmed
    if (failsafe.leak) {
        return;
    }

    failsafe.leak = true;

    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_LEAK, LogErrorCode::FAILSAFE_OCCURRED);

    // Handle failsafe action
    if (failsafe.leak && g.failsafe_leak == FS_LEAK_SURFACE && motors.armed()) {
        set_mode(Mode::Number::SURFACE, ModeReason::LEAK_FAILSAFE);
    }
}

// failsafe_gcs_check - check for ground station failsafe
void Sub::failsafe_gcs_check()
{
    // return immediately if we have never had contact with a gcs, or if gcs failsafe action is disabled
    // this also checks to see if we have a GCS failsafe active, if we do, then must continue to process the logic for recovery from this state.
    if (!g.failsafe_gcs && g.failsafe_gcs == FS_GCS_DISABLED) {
        return;
    }

    const uint32_t gcs_last_seen_ms = gcs().sysid_myggcs_last_seen_time_ms();
    if (gcs_last_seen_ms == 0) {
        // we've never seen a GCS, so we don't failsafe if we stop seeing it
        return;
    }

    uint32_t tnow = AP_HAL::millis();

    // Check if we have gotten a GCS heartbeat recently (GCS sysid must match SYSID_MYGCS parameter)
    if (tnow - gcs_last_seen_ms < FS_GCS_TIMEOUT_MS) {
        // Log event if we are recovering from previous gcs failsafe
        if (failsafe.gcs) {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_RESOLVED);
        }
        failsafe.gcs = false;
        return;
    }

    //////////////////////////////
    // GCS heartbeat has timed out
    //////////////////////////////

    // Send a warning every 30 seconds
    if (tnow - failsafe.last_gcs_warn_ms > 30000) {
        failsafe.last_gcs_warn_ms = tnow;
        gcs().send_text(MAV_SEVERITY_WARNING, "MYGCS: %u, heartbeat lost", g.sysid_my_gcs.get());
    }

    // do nothing if we have already triggered the failsafe action, or if the motors are disarmed
    if (failsafe.gcs || !motors.armed()) {
        return;
    }

    failsafe.gcs = true;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_OCCURRED);

    // handle failsafe action
    if (g.failsafe_gcs == FS_GCS_DISARM) {
        arming.disarm(AP_Arming::Method::GCSFAILSAFE);
    } else if (g.failsafe_gcs == FS_GCS_HOLD && motors.armed()) {
        if (!set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_FAILSAFE)) {
            arming.disarm(AP_Arming::Method::GCS_FAILSAFE_HOLDFAILED);
        }
    } else if (g.failsafe_gcs == FS_GCS_SURFACE && motors.armed()) {
        if (!set_mode(Mode::Number::SURFACE, ModeReason::GCS_FAILSAFE)) {
            arming.disarm(AP_Arming::Method::GCS_FAILSAFE_SURFACEFAILED);
        }
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
    if (control_mode == Mode::Number::ACRO || control_mode == Mode::Number::MANUAL) {
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
        gcs().send_text(MAV_SEVERITY_WARNING,"Crash detected");
    }

    // Only perform failsafe action once
    if (failsafe.crash) {
        return;
    }

    failsafe.crash = true;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::CRASH_CHECK, LogErrorCode::CRASH_CHECK_CRASH);

    // disarm motors
    if (g.fs_crash_check == FS_CRASH_DISARM) {
        arming.disarm(AP_Arming::Method::CRASH);
    }
}

// executes terrain failsafe if data is missing for longer than a few seconds
//  missing_data should be set to true if the vehicle failed to navigate because of missing data, false if navigation is proceeding successfully
void Sub::failsafe_terrain_check()
{
    // trigger with 5 seconds of failures while in AUTO mode
    bool valid_mode = (control_mode == Mode::Number::AUTO || control_mode == Mode::Number::GUIDED);
    bool timeout = (failsafe.terrain_last_failure_ms - failsafe.terrain_first_failure_ms) > FS_TERRAIN_TIMEOUT_MS;
    bool trigger_event = valid_mode && timeout;

    // check for clearing of event
    if (trigger_event != failsafe.terrain) {
        if (trigger_event) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"Failsafe terrain triggered");
            failsafe_terrain_on_event();
        } else {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::ERROR_RESOLVED);
            failsafe.terrain = false;
        }
    }
}

// This gets called if mission items are in ALT_ABOVE_TERRAIN frame
// Terrain failure occurs when terrain data is not found, or rangefinder is not enabled or healthy
// set terrain data status (found or not found)
void Sub::failsafe_terrain_set_status(bool data_ok)
{
    uint32_t now = AP_HAL::millis();

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
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::FAILSAFE_OCCURRED);

    // If rangefinder is enabled, we can recover from this failsafe
    if (!rangefinder_state.enabled || !sub.mode_auto.auto_terrain_recover_start()) {
        failsafe_terrain_act();
    }


}

// Recovery failed, take action
void Sub::failsafe_terrain_act()
{
    switch (g.failsafe_terrain) {
    case FS_TERRAIN_HOLD:
        if (!set_mode(Mode::Number::POSHOLD, ModeReason::TERRAIN_FAILSAFE)) {
            set_mode(Mode::Number::ALT_HOLD, ModeReason::TERRAIN_FAILSAFE);
        }
        AP_Notify::events.failsafe_mode_change = 1;
        break;

    case FS_TERRAIN_SURFACE:
        set_mode(Mode::Number::SURFACE, ModeReason::TERRAIN_FAILSAFE);
        AP_Notify::events.failsafe_mode_change = 1;
        break;

    case FS_TERRAIN_DISARM:
    default:
        arming.disarm(AP_Arming::Method::TERRAINFAILSAFE);
    }
}
