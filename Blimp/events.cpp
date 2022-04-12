#include "Blimp.h"

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */


bool Blimp::failsafe_option(FailsafeOption opt) const
{
    return (g2.fs_options & (uint32_t)opt);
}

void Blimp::failsafe_radio_on_event()
{
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_OCCURRED);

    // set desired action based on FS_THR_ENABLE parameter
    Failsafe_Action desired_action;
    switch (g.failsafe_throttle) {
    case FS_THR_DISABLED:
        desired_action = Failsafe_Action_None;
        break;
    case FS_THR_ENABLED_ALWAYS_LAND:
        desired_action = Failsafe_Action_Land;
        break;
    default:
        desired_action = Failsafe_Action_Land;
    }

    // Conditions to deviate from FS_THR_ENABLE selection and send specific GCS warning
    if (should_disarm_on_failsafe()) {
        // should immediately disarm when we're on the ground
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe - Disarming");
        arming.disarm(AP_Arming::Method::RADIOFAILSAFE);
        desired_action = Failsafe_Action_None;

    } else if (flightmode->is_landing() && ((battery.has_failsafed() && battery.get_highest_failsafe_priority() <= FAILSAFE_LAND_PRIORITY))) {
        // Allow landing to continue when battery failsafe requires it (not a user option)
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio + Battery Failsafe - Continuing Landing");
        desired_action = Failsafe_Action_Land;

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING)) {
        // Allow landing to continue when FS_OPTIONS is set to continue landing
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe - Continuing Landing");
        desired_action = Failsafe_Action_Land;

    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe");
    }

    // Call the failsafe action handler
    do_failsafe_action(desired_action, ModeReason::RADIO_FAILSAFE);
}

// failsafe_off_event - respond to radio contact being regained
void Blimp::failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_RESOLVED);
    gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe Cleared");
}

void Blimp::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);

    Failsafe_Action desired_action = (Failsafe_Action)action;

    // Conditions to deviate from BATT_FS_XXX_ACT parameter setting
    if (should_disarm_on_failsafe()) {
        // should immediately disarm when we're on the ground
        arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
        desired_action = Failsafe_Action_None;
        gcs().send_text(MAV_SEVERITY_WARNING, "Battery Failsafe - Disarming");

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING) && desired_action != Failsafe_Action_None) {
        // Allow landing to continue when FS_OPTIONS is set to continue when landing
        desired_action = Failsafe_Action_Land;
        gcs().send_text(MAV_SEVERITY_WARNING, "Battery Failsafe - Continuing Landing");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Battery Failsafe");
    }

    // Battery FS options already use the Failsafe_Options enum. So use them directly.
    do_failsafe_action(desired_action, ModeReason::BATTERY_FAILSAFE);

}
// failsafe_gcs_check - check for ground station failsafe
void Blimp::failsafe_gcs_check()
{
    // Bypass GCS failsafe checks if disabled or GCS never connected
    if (g.failsafe_gcs == FS_GCS_DISABLED) {
        return;
    }

    const uint32_t gcs_last_seen_ms = gcs().sysid_myggcs_last_seen_time_ms();
    if (gcs_last_seen_ms == 0) {
         return;
     }

    // calc time since last gcs update
    // note: this only looks at the heartbeat from the device id set by g.sysid_my_gcs
    const uint32_t last_gcs_update_ms = millis() - gcs_last_seen_ms;
    const uint32_t gcs_timeout_ms = uint32_t(constrain_float(g2.fs_gcs_timeout * 1000.0f, 0.0f, UINT32_MAX));

    // Determine which event to trigger
    if (last_gcs_update_ms < gcs_timeout_ms && failsafe.gcs) {
        // Recovery from a GCS failsafe
        set_failsafe_gcs(false);
        // failsafe_gcs_off_event();

    } else if (last_gcs_update_ms < gcs_timeout_ms && !failsafe.gcs) {
        // No problem, do nothing

    } else if (last_gcs_update_ms > gcs_timeout_ms && failsafe.gcs) {
        // Already in failsafe, do nothing

    } else if (last_gcs_update_ms > gcs_timeout_ms && !failsafe.gcs) {
        // New GCS failsafe event, trigger events
        set_failsafe_gcs(true);
        arming.disarm(AP_Arming::Method::GCSFAILSAFE); // failsafe_gcs_on_event() should replace this when written
    }
}

bool Blimp::should_disarm_on_failsafe()
{
    if (ap.in_arming_delay) {
        return true;
    }

    switch (control_mode) {
    case Mode::Number::MANUAL:
    default:
        // if landed disarm
        return ap.land_complete;
    }
}


void Blimp::do_failsafe_action(Failsafe_Action action, ModeReason reason)
{

    // Execute the specified desired_action
    switch (action) {
    case Failsafe_Action_None:
        return;
    case Failsafe_Action_Land:
        set_mode_land_with_pause(reason);
        break;
    case Failsafe_Action_Terminate: {
        arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
    }
    break;
    }
}

// check for gps glitch failsafe
void Blimp::gpsglitch_check()
{
    // get filter status
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    bool gps_glitching = filt_status.flags.gps_glitching;

    // log start or stop of gps glitch.  AP_Notify update is handled from within AP_AHRS
    if (ap.gps_glitching != gps_glitching) {
        ap.gps_glitching = gps_glitching;
        if (gps_glitching) {
            AP::logger().Write_Error(LogErrorSubsystem::GPS, LogErrorCode::GPS_GLITCH);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch");
        } else {
            AP::logger().Write_Error(LogErrorSubsystem::GPS, LogErrorCode::ERROR_RESOLVED);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch cleared");
        }
    }
}
