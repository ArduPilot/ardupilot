/*
  failsafe support
  Andrew Tridgell, December 2011
 */

#include "Rover.h"

#include <stdio.h>

/*
  our failsafe strategy is to detect main loop lockup and disarm.
 */

/*
  this failsafe_check function is called from the core timer interrupt
  at 1kHz.
 */
void Rover::failsafe_check()
{
    static uint16_t last_ticks;
    static uint32_t last_timestamp;
    const uint32_t tnow = AP_HAL::micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != last_ticks) {
        // the main loop is running, all is OK
        last_ticks = ticks;
        last_timestamp = tnow;
        return;
    }

    if (tnow - last_timestamp > 200000) {
        // we have gone at least 0.2 seconds since the main loop
        // ran. That means we're in trouble, or perhaps are in
        // an initialisation routine or log erase. disarm the motors
        // To-Do: log error
        if (arming.is_armed()) {
            // disarm motors
            arming.disarm(AP_Arming::Method::CPUFAILSAFE);
        }
    }
}

/*
  called to set/unset a failsafe event.
 */
void Rover::failsafe_trigger(uint8_t failsafe_type, const char* type_str, bool on)
{
    uint8_t old_bits = failsafe.bits;
    if (on) {
        failsafe.bits |= failsafe_type;
    } else {
        failsafe.bits &= ~failsafe_type;
    }
    if (old_bits == 0 && failsafe.bits != 0) {
        // a failsafe event has started
        failsafe.start_time = millis();
    }
    if (failsafe.triggered != 0 && failsafe.bits == 0) {
        // a failsafe event has ended
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Failsafe Cleared", type_str);
    }

    failsafe.triggered &= failsafe.bits;

    if (failsafe_type == FAILSAFE_EVENT_GCS) {
        AP_Notify::flags.failsafe_gcs = on;
    }

    if ((failsafe.triggered == 0) &&
        (failsafe.bits != 0) &&
        (millis() - failsafe.start_time > g.fs_timeout * 1000) &&
        (control_mode != &mode_rtl) &&
        ((control_mode != &mode_hold || (g2.fs_options & (uint32_t)Failsafe_Options::Failsafe_Option_Active_In_Hold)))) {
        failsafe.triggered = failsafe.bits;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s Failsafe", type_str);

        // clear rc overrides
        RC_Channels::clear_overrides();

        if ((control_mode == &mode_auto) &&
            ((failsafe_type == FAILSAFE_EVENT_THROTTLE && g.fs_throttle_enabled == FS_THR_ENABLED_CONTINUE_MISSION) ||
             (failsafe_type == FAILSAFE_EVENT_GCS && g.fs_gcs_enabled == FS_GCS_ENABLED_CONTINUE_MISSION))) {
            // continue with mission in auto mode
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Failsafe - Continuing Auto Mode");
        } else {
            switch ((FailsafeAction)g.fs_action.get()) {
            case FailsafeAction::None:
                break;
            case FailsafeAction::SmartRTL:
                if (set_mode(mode_smartrtl, ModeReason::FAILSAFE)) {
                    break;
                }
                FALLTHROUGH;
            case FailsafeAction::RTL:
                if (set_mode(mode_rtl, ModeReason::FAILSAFE)) {
                    break;
                }
                FALLTHROUGH;
            case FailsafeAction::Hold:
                set_mode(mode_hold, ModeReason::FAILSAFE);
                break;
            case FailsafeAction::SmartRTL_Hold:
                if (!set_mode(mode_smartrtl, ModeReason::FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::FAILSAFE);
                }
                break;
            case FailsafeAction::Loiter_Hold:
                if (!set_mode(mode_loiter, ModeReason::FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::FAILSAFE);
                }
                break;
            case FailsafeAction::Terminate:
                arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
                break;
            }
        }
    }
}

void Rover::handle_battery_failsafe(const char* type_str, const int8_t action)
{
        switch ((FailsafeAction)action) {
            case FailsafeAction::None:
                break;
            case FailsafeAction::SmartRTL:
                if (set_mode(mode_smartrtl, ModeReason::BATTERY_FAILSAFE)) {
                    break;
                }
                FALLTHROUGH;
            case FailsafeAction::RTL:
                if (set_mode(mode_rtl, ModeReason::BATTERY_FAILSAFE)) {
                    break;
                }
                FALLTHROUGH;
            case FailsafeAction::Hold:
                set_mode(mode_hold, ModeReason::BATTERY_FAILSAFE);
                break;
            case FailsafeAction::Loiter_Hold:
                if (!set_mode(mode_loiter, ModeReason::BATTERY_FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::BATTERY_FAILSAFE);
                }
                break;
            case FailsafeAction::SmartRTL_Hold:
                if (!set_mode(mode_smartrtl, ModeReason::BATTERY_FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::BATTERY_FAILSAFE);
                }
                break;
            case FailsafeAction::Terminate:
#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
                char battery_type_str[17];
                snprintf(battery_type_str, 17, "%s battery", type_str);
                g2.afs.gcs_terminate(true, battery_type_str);
#else
                arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
#endif // AP_ROVER_ADVANCED_FAILSAFE_ENABLED
                break;
        }
}

#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
/*
   check for AFS failsafe check
 */
void Rover::afs_fs_check(void)
{
    // perform AFS failsafe checks
    g2.afs.check(failsafe.last_valid_rc_ms);
}
#endif
