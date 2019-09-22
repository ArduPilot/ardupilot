#include "Copter.h"

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */

bool Copter::failsafe_option(FailsafeOption opt) const
{
    return (g2.fs_options & (uint32_t)opt);
}

void Copter::failsafe_radio_on_event()
{
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_OCCURRED);

    // set desired action based on FS_THR_ENABLE parameter
    Failsafe_Action desired_action;
    switch (g.failsafe_throttle) {
        case FS_THR_DISABLED:
            desired_action = Failsafe_Action_None;
            break;
        case FS_THR_ENABLED_ALWAYS_RTL:
        case FS_THR_ENABLED_CONTINUE_MISSION:
            desired_action = Failsafe_Action_RTL;
            break;
        case FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL:
            desired_action = Failsafe_Action_SmartRTL;
            break;
        case FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND:
            desired_action = Failsafe_Action_SmartRTL_Land;
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
        arming.disarm();
        desired_action = Failsafe_Action_None;

    } else if (flightmode->is_landing() && ((battery.has_failsafed() && battery.get_highest_failsafe_priority() <= FAILSAFE_LAND_PRIORITY))) {
        // Allow landing to continue when battery failsafe requires it (not a user option)
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio + Battery Failsafe - Continuing Landing");
        desired_action = Failsafe_Action_Land;

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING)) {
        // Allow landing to continue when FS_OPTIONS is set to continue landing
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe - Continuing Landing");
        desired_action = Failsafe_Action_Land;

    } else if (control_mode == Mode::Number::AUTO && failsafe_option(FailsafeOption::RC_CONTINUE_IF_AUTO)) {
        // Allow mission to continue when FS_OPTIONS is set to continue mission
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe - Continuing Auto Mode");       
        desired_action = Failsafe_Action_None;

    } else if ((flightmode->in_guided_mode()) &&
      (failsafe_option(FailsafeOption::RC_CONTINUE_IF_GUIDED)) && (g.failsafe_gcs != FS_GCS_DISABLED)) {
        // Allow guided mode to continue when FS_OPTIONS is set to continue in guided mode.  Only if the GCS failsafe is enabled.
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe - Continuing Guided Mode");
        desired_action = Failsafe_Action_None;

    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe");
    }

    // Call the failsafe action handler
    do_failsafe_action(desired_action, ModeReason::RADIO_FAILSAFE);
}

// failsafe_off_event - respond to radio contact being regained
void Copter::failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_RESOLVED);
    gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe Cleared");
}

void Copter::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);

    Failsafe_Action desired_action = (Failsafe_Action)action;

    // Conditions to deviate from BATT_FS_XXX_ACT parameter setting
    if (should_disarm_on_failsafe()) {
        // should immediately disarm when we're on the ground
        arming.disarm();
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
void Copter::failsafe_gcs_check()
{
    // Bypass GCS failsafe checks if disabled or GCS never connected
    if (g.failsafe_gcs == FS_GCS_DISABLED || failsafe.last_heartbeat_ms == 0) {
        return;
    }

    // calc time since last gcs update
    // note: this only looks at the heartbeat from the device id set by g.sysid_my_gcs
    const uint32_t last_gcs_update_ms = millis() - failsafe.last_heartbeat_ms;

    // Determine which event to trigger
    if (last_gcs_update_ms < FS_GCS_TIMEOUT_MS && failsafe.gcs) {
        // Recovery from a GCS failsafe
        set_failsafe_gcs(false);
        failsafe_gcs_off_event();

    } else if (last_gcs_update_ms < FS_GCS_TIMEOUT_MS && !failsafe.gcs) {
        // No problem, do nothing

    } else if (last_gcs_update_ms > FS_GCS_TIMEOUT_MS && failsafe.gcs) {
        // Already in failsafe, do nothing

    } else if (last_gcs_update_ms > FS_GCS_TIMEOUT_MS && !failsafe.gcs) {
        // New GCS failsafe event, trigger events
        set_failsafe_gcs(true);
        failsafe_gcs_on_event();
    }
}

// failsafe_gcs_on_event - actions to take when GCS contact is lost
void Copter::failsafe_gcs_on_event(void)
{
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_OCCURRED);
    RC_Channels::clear_overrides();

    // convert the desired failsafe response to the Failsafe_Action enum
    Failsafe_Action desired_action;
    switch (g.failsafe_gcs) {
        case FS_GCS_DISABLED:
            desired_action = Failsafe_Action_None;
            break;
        case FS_GCS_ENABLED_ALWAYS_RTL:
        case FS_GCS_ENABLED_CONTINUE_MISSION:
            desired_action = Failsafe_Action_RTL;
            break;
        case FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL:
            desired_action = Failsafe_Action_SmartRTL;
            break;
        case FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND:
            desired_action = Failsafe_Action_SmartRTL_Land;
            break;
        case FS_GCS_ENABLED_ALWAYS_LAND:
            desired_action = Failsafe_Action_Land;
            break;
        default: // if an invalid parameter value is set, the fallback is RTL
            desired_action = Failsafe_Action_RTL;
    }

    // Conditions to deviate from FS_GCS_ENABLE parameter setting
    if (!motors->armed()) {
        desired_action = Failsafe_Action_None;
        gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe");

    } else if (should_disarm_on_failsafe()) {
        // should immediately disarm when we're on the ground
        arming.disarm();
        desired_action = Failsafe_Action_None;
        gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe - Disarming");

    } else if (flightmode->is_landing() && ((battery.has_failsafed() && battery.get_highest_failsafe_priority() <= FAILSAFE_LAND_PRIORITY))) {
        // Allow landing to continue when battery failsafe requires it (not a user option)
        gcs().send_text(MAV_SEVERITY_WARNING, "GCS + Battery Failsafe - Continuing Landing");
        desired_action = Failsafe_Action_Land;

    } else if (flightmode->is_landing() && failsafe_option(FailsafeOption::CONTINUE_IF_LANDING)) {
        // Allow landing to continue when FS_OPTIONS is set to continue landing
        gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe - Continuing Landing");
        desired_action = Failsafe_Action_Land;

    } else if (control_mode == Mode::Number::AUTO && failsafe_option(FailsafeOption::GCS_CONTINUE_IF_AUTO)) {
        // Allow mission to continue when FS_OPTIONS is set to continue mission
        gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe - Continuing Auto Mode");
        desired_action = Failsafe_Action_None;

    } else if (failsafe_option(FailsafeOption::GCS_CONTINUE_IF_PILOT_CONTROL) && !flightmode->is_autopilot()) {
        // should continue when in a pilot controlled mode because FS_OPTIONS is set to continue in pilot controlled modes
        gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe - Continuing Pilot Control");
        desired_action = Failsafe_Action_None;
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe");
    }

    // Call the failsafe action handler
    do_failsafe_action(desired_action, ModeReason::GCS_FAILSAFE);
}

// failsafe_gcs_off_event - actions to take when GCS contact is restored
void Copter::failsafe_gcs_off_event(void)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe Cleared");
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_RESOLVED);
}

// executes terrain failsafe if data is missing for longer than a few seconds
//  missing_data should be set to true if the vehicle failed to navigate because of missing data, false if navigation is proceeding successfully
void Copter::failsafe_terrain_check()
{
    // trigger with 5 seconds of failures while in AUTO mode
    bool valid_mode = (control_mode == Mode::Number::AUTO ||
                       control_mode == Mode::Number::GUIDED ||
                       control_mode == Mode::Number::GUIDED_NOGPS ||
                       control_mode == Mode::Number::RTL);
    bool timeout = (failsafe.terrain_last_failure_ms - failsafe.terrain_first_failure_ms) > FS_TERRAIN_TIMEOUT_MS;
    bool trigger_event = valid_mode && timeout;

    // check for clearing of event
    if (trigger_event != failsafe.terrain) {
        if (trigger_event) {
            failsafe_terrain_on_event();
        } else {
            AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::ERROR_RESOLVED);
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
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::FAILSAFE_OCCURRED);

    if (should_disarm_on_failsafe()) {
        arming.disarm();
#if MODE_RTL_ENABLED == ENABLED
    } else if (control_mode == Mode::Number::RTL) {
        mode_rtl.restart_without_terrain();
#endif
    } else {
        set_mode_RTL_or_land_with_pause(ModeReason::TERRAIN_FAILSAFE);
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
            AP::logger().Write_Error(LogErrorSubsystem::GPS, LogErrorCode::GPS_GLITCH);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch");
        } else {
            AP::logger().Write_Error(LogErrorSubsystem::GPS, LogErrorCode::ERROR_RESOLVED);
            gcs().send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch cleared");
        }
    }
}

// set_mode_RTL_or_land_with_pause - sets mode to RTL if possible or LAND with 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_RTL_or_land_with_pause(ModeReason reason)
{
    // attempt to switch to RTL, if this fails then switch to Land
    if (!set_mode(Mode::Number::RTL, reason)) {
        // set mode to land will trigger mode change notification to pilot
        set_mode_land_with_pause(reason);
    } else {
        // alert pilot to mode change
        AP_Notify::events.failsafe_mode_change = 1;
    }
}

// set_mode_SmartRTL_or_land_with_pause - sets mode to SMART_RTL if possible or LAND with 4 second delay before descent starts
// this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_SmartRTL_or_land_with_pause(ModeReason reason)
{
    // attempt to switch to SMART_RTL, if this failed then switch to Land
    if (!set_mode(Mode::Number::SMART_RTL, reason)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SmartRTL Unavailable, Using Land Mode");
        set_mode_land_with_pause(reason);
    } else {
        AP_Notify::events.failsafe_mode_change = 1;
    }
}

// set_mode_SmartRTL_or_RTL - sets mode to SMART_RTL if possible or RTL if possible or LAND with 4 second delay before descent starts
// this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_SmartRTL_or_RTL(ModeReason reason)
{
    // attempt to switch to SmartRTL, if this failed then attempt to RTL
    // if that fails, then land
    if (!set_mode(Mode::Number::SMART_RTL, reason)) {
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
        case Mode::Number::STABILIZE:
        case Mode::Number::ACRO:
            // if throttle is zero OR vehicle is landed disarm motors
            return ap.throttle_zero || ap.land_complete;
        case Mode::Number::AUTO:
            // if mission has not started AND vehicle is landed, disarm motors
            return !ap.auto_armed && ap.land_complete;
        default:
            // used for AltHold, Guided, Loiter, RTL, Circle, Drift, Sport, Flip, Autotune, PosHold
            // if landed disarm
            return ap.land_complete;
    }
}


void Copter::do_failsafe_action(Failsafe_Action action, ModeReason reason){

    // Execute the specified desired_action
    switch (action) {
        case Failsafe_Action_None:
            return;
        case Failsafe_Action_Land:
            set_mode_land_with_pause(reason);
            break;
        case Failsafe_Action_RTL:
            set_mode_RTL_or_land_with_pause(reason);
            break;
        case Failsafe_Action_SmartRTL:
            set_mode_SmartRTL_or_RTL(reason);
            break;
        case Failsafe_Action_SmartRTL_Land:
            set_mode_SmartRTL_or_land_with_pause(reason);
            break;
        case Failsafe_Action_Terminate:
#if ADVANCED_FAILSAFE == ENABLED
            char battery_type_str[17];
            snprintf(battery_type_str, 17, "%s battery", type_str);
            g2.afs.gcs_terminate(true, battery_type_str);
#else
            arming.disarm();
#endif
    }
}

