/*
  failsafe support
  Andrew Tridgell, December 2011
 */

#include "Rover.h"

/*
  our failsafe strategy is to detect main loop lockup and switch to
  passing inputs straight from the RC inputs to RC outputs.
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
        // To-Do: log error to dataflash
        if (arming.is_armed()) {
            // disarm motors
            disarm_motors();
        }
    }
}

/*
  called to set/unset a failsafe event.
 */
void Rover::failsafe_trigger(uint8_t failsafe_type, bool on)
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
        gcs().send_text(MAV_SEVERITY_INFO, "Failsafe ended");
    }

    failsafe.triggered &= failsafe.bits;

    if (failsafe.triggered == 0 &&
        failsafe.bits != 0 &&
        millis() - failsafe.start_time > g.fs_timeout * 1000 &&
        control_mode != &mode_rtl &&
        control_mode != &mode_hold) {
        failsafe.triggered = failsafe.bits;
        gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe trigger 0x%x", static_cast<uint32_t>(failsafe.triggered));
        switch (g.fs_action) {
            case 0:
                break;
            case 1:
                if (!set_mode(mode_rtl, MODE_REASON_FAILSAFE)) {
                    set_mode(mode_hold, MODE_REASON_FAILSAFE);
                }
                break;
            case 2:
                set_mode(mode_hold, MODE_REASON_FAILSAFE);
                break;
            case 3:
                if (!set_mode(mode_smartrtl, MODE_REASON_FAILSAFE)) {
                    if (!set_mode(mode_rtl, MODE_REASON_FAILSAFE)) {
                        set_mode(mode_hold, MODE_REASON_FAILSAFE);
                    }
                }
                break;
            case 4:
                if (!set_mode(mode_smartrtl, MODE_REASON_FAILSAFE)) {
                    set_mode(mode_hold, MODE_REASON_FAILSAFE);
                }
                break;
        }
    }
}

void Rover::handle_battery_failsafe(const char* type_str, const int8_t action)
{
        switch ((Failsafe_Action)action) {
            case Failsafe_Action_None:
                break;
            case Failsafe_Action_SmartRTL:
                if (set_mode(mode_smartrtl, MODE_REASON_FAILSAFE)) {
                    break;
                }
                FALLTHROUGH;
            case Failsafe_Action_RTL:
                if (set_mode(mode_rtl, MODE_REASON_FAILSAFE)) {
                    break;
                }
                FALLTHROUGH;
            case Failsafe_Action_Hold:
                set_mode(mode_hold, MODE_REASON_FAILSAFE);
                break;
            case Failsafe_Action_SmartRTL_Hold:
                if (!set_mode(mode_smartrtl, MODE_REASON_FAILSAFE)) {
                    set_mode(mode_hold, MODE_REASON_FAILSAFE);
                }
                break;
            case Failsafe_Action_Terminate:
#if ADVANCED_FAILSAFE == ENABLED
                char battery_type_str[17];
                snprintf(battery_type_str, 17, "%s battery", type_str);
                g2.afs.gcs_terminate(true, battery_type_str);
#else
                disarm_motors();
#endif // ADVANCED_FAILSAFE == ENABLED
                break;
        }
}

#if ADVANCED_FAILSAFE == ENABLED
/*
   check for AFS failsafe check
 */
void Rover::afs_fs_check(void)
{
    // perform AFS failsafe checks
    g2.afs.check(rover.last_heartbeat_ms, rover.g2.fence.get_breaches() != 0, failsafe.last_valid_rc_ms);
}
#endif
