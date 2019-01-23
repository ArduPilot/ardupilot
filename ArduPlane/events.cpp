#include "Plane.h"

void Plane::failsafe_short_on_event(enum failsafe_state fstype, mode_reason_t reason)
{
    // This is how to handle a short loss of control signal failsafe.
    failsafe.state = fstype;
    failsafe.short_timer_ms = millis();
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event on: type=%u/reason=%u", fstype, reason);
    switch(control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case TRAINING:
        failsafe.saved_mode = control_mode;
        failsafe.saved_mode_set = true;
        if(g.fs_action_short == FS_ACTION_SHORT_FBWA) {
            set_mode(FLY_BY_WIRE_A, reason);
        } else {
            set_mode(CIRCLE, reason);
        }
        break;

    case QSTABILIZE:
    case QLOITER:
    case QHOVER:
    case QAUTOTUNE:
        failsafe.saved_mode = control_mode;
        failsafe.saved_mode_set = true;
        set_mode(QLAND, reason);
        break;
        
    case AUTO:
    case AVOID_ADSB:
    case GUIDED:
    case LOITER:
        if(g.fs_action_short != FS_ACTION_SHORT_BESTGUESS) {
            failsafe.saved_mode = control_mode;
            failsafe.saved_mode_set = true;
            if(g.fs_action_short == FS_ACTION_SHORT_FBWA) {
                set_mode(FLY_BY_WIRE_A, reason);
            } else {
                set_mode(CIRCLE, reason);
            }
        }
        break;

    case CIRCLE:
    case RTL:
    case QLAND:
    case QRTL:
    default:
        break;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode);
}

void Plane::failsafe_long_on_event(enum failsafe_state fstype, mode_reason_t reason)
{
    // This is how to handle a long loss of control signal failsafe.
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Long event on: type=%u/reason=%u", fstype, reason);
    //  If the GCS is locked up we allow control to revert to RC
    RC_Channels::clear_overrides();
    failsafe.state = fstype;
    switch(control_mode)
    {
    case MANUAL:
    case STABILIZE:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case TRAINING:
    case CIRCLE:
        if(g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if PARACHUTE == ENABLED
            parachute_release();
#endif
        } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
            set_mode(FLY_BY_WIRE_A, reason);
        } else {
            set_mode(RTL, reason);
        }
        break;

    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QAUTOTUNE:
        set_mode(QLAND, reason);
        break;
        
    case AUTO:
    case AVOID_ADSB:
    case GUIDED:
    case LOITER:
        if(g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if PARACHUTE == ENABLED
            parachute_release();
#endif
        } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
            set_mode(FLY_BY_WIRE_A, reason);
        } else if (g.fs_action_long == FS_ACTION_LONG_RTL) {
            set_mode(RTL, reason);
        }
        break;

    case RTL:
    case QLAND:
    case QRTL:
    default:
        break;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode);
}

void Plane::failsafe_short_off_event(mode_reason_t reason)
{
    // We're back in radio contact
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event off: reason=%u", reason);
    failsafe.state = FAILSAFE_NONE;

    // re-read the switch so we can return to our preferred mode
    // --------------------------------------------------------
    if (control_mode == CIRCLE && failsafe.saved_mode_set) {
        failsafe.saved_mode_set = false;
        set_mode(failsafe.saved_mode, reason);
    }
}

void Plane::failsafe_long_off_event(mode_reason_t reason)
{
    // We're back in radio contact
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Long event off: reason=%u", reason);
    failsafe.state = FAILSAFE_NONE;
}

void Plane::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    switch ((Failsafe_Action)action) {
        case Failsafe_Action_QLand:
            if (quadplane.available()) {
                plane.set_mode(QLAND, MODE_REASON_BATTERY_FAILSAFE);
                break;
            }
            FALLTHROUGH;
        case Failsafe_Action_Land:
            if (flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND && control_mode != QLAND) {
                // never stop a landing if we were already committed
                if (plane.mission.jump_to_landing_sequence()) {
                    plane.set_mode(AUTO, MODE_REASON_BATTERY_FAILSAFE);
                    break;
                }
            }
            FALLTHROUGH;
        case Failsafe_Action_RTL:
            if (flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND && control_mode != QLAND ) {
                // never stop a landing if we were already committed
                set_mode(RTL, MODE_REASON_BATTERY_FAILSAFE);
                aparm.throttle_cruise.load();
            }
            break;

        case Failsafe_Action_Terminate:
            char battery_type_str[17];
            snprintf(battery_type_str, 17, "%s battery", type_str);
            afs.gcs_terminate(true, battery_type_str);
            break;

        case Failsafe_Action_None:
            // don't actually do anything, however we should still flag the system as having hit a failsafe
            // and ensure all appropriate flags are going off to the user
            break;
    }
}
