#include "Plane.h"

void Plane::failsafe_short_on_event(enum failsafe_state fstype, ModeReason reason)
{
    // This is how to handle a short loss of control signal failsafe.
    failsafe.state = fstype;
    failsafe.short_timer_ms = millis();
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event on: type=%u/reason=%u", fstype, static_cast<unsigned>(reason));
    switch (control_mode->mode_number())
    {
    case Mode::Number::MANUAL:
    case Mode::Number::STABILIZE:
    case Mode::Number::ACRO:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
    case Mode::Number::TRAINING:
        failsafe.saved_mode_number = control_mode->mode_number();
        failsafe.saved_mode_set = true;
        if(g.fs_action_short == FS_ACTION_SHORT_FBWA) {
            set_mode(mode_fbwa, reason);
        } else {
            set_mode(mode_circle, reason);
        }
        break;

    case Mode::Number::QSTABILIZE:
    case Mode::Number::QLOITER:
    case Mode::Number::QHOVER:
    case Mode::Number::QAUTOTUNE:
    case Mode::Number::QACRO:
        failsafe.saved_mode_number = control_mode->mode_number();
        failsafe.saved_mode_set = true;
        if (quadplane.options & QuadPlane::OPTION_FS_QRTL) {
            set_mode(mode_qrtl, reason);
        } else {
            set_mode(mode_qland, reason);
        }
        break;
        
    case Mode::Number::AUTO:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::LOITER:
        if(g.fs_action_short != FS_ACTION_SHORT_BESTGUESS) {
            failsafe.saved_mode_number = control_mode->mode_number();
            failsafe.saved_mode_set = true;
            if(g.fs_action_short == FS_ACTION_SHORT_FBWA) {
                set_mode(mode_fbwa, reason);
            } else {
                set_mode(mode_circle, reason);
            }
        }
        break;

    case Mode::Number::CIRCLE:
    case Mode::Number::TAKEOFF:
    case Mode::Number::RTL:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
    case Mode::Number::INITIALISING:
        break;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode->mode_number());
}

void Plane::failsafe_long_on_event(enum failsafe_state fstype, ModeReason reason)
{
    // This is how to handle a long loss of control signal failsafe.
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Long event on: type=%u/reason=%u", fstype, static_cast<unsigned>(reason));
    //  If the GCS is locked up we allow control to revert to RC
    RC_Channels::clear_overrides();
    failsafe.state = fstype;
    switch (control_mode->mode_number())
    {
    case Mode::Number::MANUAL:
    case Mode::Number::STABILIZE:
    case Mode::Number::ACRO:
    case Mode::Number::FLY_BY_WIRE_A:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
    case Mode::Number::TRAINING:
    case Mode::Number::CIRCLE:
        if(g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if PARACHUTE == ENABLED
            parachute_release();
#endif
        } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
            set_mode(mode_fbwa, reason);
        } else {
            set_mode(mode_rtl, reason);
        }
        break;

    case Mode::Number::QSTABILIZE:
    case Mode::Number::QHOVER:
    case Mode::Number::QLOITER:
    case Mode::Number::QACRO:
    case Mode::Number::QAUTOTUNE:
        if (quadplane.options & QuadPlane::OPTION_FS_QRTL) {
            set_mode(mode_qrtl, reason);
        } else {
            set_mode(mode_qland, reason);
        }
        break;
        
    case Mode::Number::AUTO:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::LOITER:
        if(g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if PARACHUTE == ENABLED
            parachute_release();
#endif
        } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
            set_mode(mode_fbwa, reason);
        } else if (g.fs_action_long == FS_ACTION_LONG_RTL) {
            set_mode(mode_rtl, reason);
        }
        break;

    case Mode::Number::RTL:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
    case Mode::Number::TAKEOFF:
    case Mode::Number::INITIALISING:
        break;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode->mode_number());
}

void Plane::failsafe_short_off_event(ModeReason reason)
{
    // We're back in radio contact
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event off: reason=%u", static_cast<unsigned>(reason));
    failsafe.state = FAILSAFE_NONE;

    // re-read the switch so we can return to our preferred mode
    // --------------------------------------------------------
    if (control_mode == &mode_circle && failsafe.saved_mode_set) {
        failsafe.saved_mode_set = false;
        set_mode_by_number(failsafe.saved_mode_number, reason);
    }
}

void Plane::failsafe_long_off_event(ModeReason reason)
{
    // We're back in radio contact
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Long event off: reason=%u", static_cast<unsigned>(reason));
    failsafe.state = FAILSAFE_NONE;
}

void Plane::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    switch ((Failsafe_Action)action) {
        case Failsafe_Action_QLand:
            if (quadplane.available()) {
                plane.set_mode(mode_qland, ModeReason::BATTERY_FAILSAFE);
                break;
            }
            FALLTHROUGH;
        case Failsafe_Action_Land:
            if (flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND && control_mode != &mode_qland) {
                // never stop a landing if we were already committed
                if (plane.mission.is_best_land_sequence()) {
                    // continue mission as it will reach a landing in less distance
                    plane.mission.set_in_landing_sequence_flag(true);
                    break;
                }
                if (plane.mission.jump_to_landing_sequence()) {
                    plane.set_mode(mode_auto, ModeReason::BATTERY_FAILSAFE);
                    break;
                }
            }
            FALLTHROUGH;
        case Failsafe_Action_RTL:
            if (flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND && control_mode != &mode_qland && !quadplane.in_vtol_land_sequence()) {
                // never stop a landing if we were already committed
                if (g.rtl_autoland == 2 && plane.mission.is_best_land_sequence()) {
                    // continue mission as it will reach a landing in less distance
                    plane.mission.set_in_landing_sequence_flag(true);
                    break;
                }
                set_mode(mode_rtl, ModeReason::BATTERY_FAILSAFE);
                aparm.throttle_cruise.load();
            }
            break;

        case Failsafe_Action_Terminate:
#if ADVANCED_FAILSAFE == ENABLED
            char battery_type_str[17];
            snprintf(battery_type_str, 17, "%s battery", type_str);
            afs.gcs_terminate(true, battery_type_str);
#else
            arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
#endif
            break;

        case Failsafe_Action_Parachute:
#if PARACHUTE == ENABLED
            parachute_release();
#endif
            break;

        case Failsafe_Action_None:
            // don't actually do anything, however we should still flag the system as having hit a failsafe
            // and ensure all appropriate flags are going off to the user
            break;
    }
}
