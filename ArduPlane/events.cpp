#include "Plane.h"

// returns true if the vehicle is in landing sequence.  Intended only
// for use in failsafe code.
bool Plane::failsafe_in_landing_sequence() const
{
    if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        return true;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_land_sequence()) {
        return true;
    }
#endif
    if (mission.get_in_landing_sequence_flag()) {
        return true;
    }
    return false;
}

void Plane::rc_failsafe_short_on_event()
{
    // This is how to handle a short loss of control signal failsafe.
    failsafe.state = FAILSAFE_SHORT;
    failsafe.saved_mode_number = control_mode->mode_number();
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
        if(plane.emergency_landing) {
            set_mode(mode_fbwa, ModeReason::RADIO_FAILSAFE); // emergency landing switch overrides normal action to allow out of range landing
            break;
        }
        if(g.fs_action_short == FS_ACTION_SHORT_FBWA) {
            set_mode(mode_fbwa, ModeReason::RADIO_FAILSAFE);
        } else if (g.fs_action_short == FS_ACTION_SHORT_FBWB) {
            set_mode(mode_fbwb, ModeReason::RADIO_FAILSAFE);
        } else {
            set_mode(mode_circle, ModeReason::RADIO_FAILSAFE); // circle if action = 0 or 1 
        }
        break;

#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QLOITER:
    case Mode::Number::QHOVER:
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
#endif
    case Mode::Number::QACRO:
        if (quadplane.option_is_set(QuadPlane::Option::FS_RTL)) {
            set_mode(mode_rtl, ModeReason::RADIO_FAILSAFE);
        } else if (quadplane.option_is_set(QuadPlane::Option::FS_QRTL)) {
            set_mode(mode_qrtl, ModeReason::RADIO_FAILSAFE);
        } else {
            set_mode(mode_qland, ModeReason::RADIO_FAILSAFE);
        }
        break;
#endif // HAL_QUADPLANE_ENABLED

    case Mode::Number::AUTO:
#if MODE_AUTOLAND_ENABLED
    case Mode::Number::AUTOLAND:
#endif
        {
        if (failsafe_in_landing_sequence()) {
            // don't failsafe in a landing sequence
            break;
        }
        FALLTHROUGH;
    }
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::LOITER:
    case Mode::Number::THERMAL:
        if (g.fs_action_short != FS_ACTION_SHORT_BESTGUESS) { // if acton = 0(BESTGUESS) this group of modes take no action
            failsafe.saved_mode_number = control_mode->mode_number();
            if (g.fs_action_short == FS_ACTION_SHORT_FBWA) {
                set_mode(mode_fbwa, ModeReason::RADIO_FAILSAFE);
            } else if (g.fs_action_short == FS_ACTION_SHORT_FBWB) {
                set_mode(mode_fbwb, ModeReason::RADIO_FAILSAFE);
            } else {
                set_mode(mode_circle, ModeReason::RADIO_FAILSAFE);
            }
        }
         break;
    case Mode::Number::CIRCLE:  // these modes never take any short failsafe action and continue
    case Mode::Number::TAKEOFF:
    case Mode::Number::RTL:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
    case Mode::Number::LOITER_ALT_QLAND:
#endif
    case Mode::Number::INITIALISING:
        break;
    }
    if (failsafe.saved_mode_number != control_mode->mode_number()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "RC Short Failsafe: switched to %s", control_mode->name());
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "RC Short Failsafe On");
    }
}

void Plane::failsafe_long_on_event(enum failsafe_state fstype, ModeReason reason)
{

    if (reason == ModeReason::GCS_FAILSAFE) {
        AP_Notify::flags.failsafe_gcs = true;
    }

    // This is how to handle a long loss of control signal failsafe.
    // If the GCS is locked up we allow control to revert to RC
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
    case Mode::Number::LOITER:
    case Mode::Number::THERMAL:
    case Mode::Number::TAKEOFF:
        if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF && !(g.fs_action_long == FS_ACTION_LONG_GLIDE || g.fs_action_long == FS_ACTION_LONG_PARACHUTE)) {
            // don't failsafe if in initial climb of TAKEOFF mode and FS action is not parachute or glide
            // long failsafe will be re-called if still in fs after initial climb
            long_failsafe_pending = true;
            break;
        }

        if(plane.emergency_landing) {
            set_mode(mode_fbwa, reason); // emergency landing switch overrides normal action to allow out of range landing
            break;
        }
        if(g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if HAL_PARACHUTE_ENABLED
            parachute_release();
#endif
        } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
            set_mode(mode_fbwa, reason);
        } else if (g.fs_action_long == FS_ACTION_LONG_AUTO) {
            set_mode(mode_auto, reason);
#if MODE_AUTOLAND_ENABLED
        } else if (g.fs_action_long == FS_ACTION_LONG_AUTOLAND) {
            if (!set_mode(mode_autoland, reason)) {
               set_mode(mode_rtl, reason);
            }
#endif
        } else {
            set_mode(mode_rtl, reason);
        }
        break;

#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
    case Mode::Number::QHOVER:
    case Mode::Number::QLOITER:
    case Mode::Number::QACRO:
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
#endif
        if (quadplane.option_is_set(QuadPlane::Option::FS_RTL)) {
            set_mode(mode_rtl, reason);
        } else if (quadplane.option_is_set(QuadPlane::Option::FS_QRTL)) {
            set_mode(mode_qrtl, reason);
        } else {
            set_mode(mode_qland, reason);
        }
        break;
#endif  // HAL_QUADPLANE_ENABLED

    case Mode::Number::AUTO:
        if (failsafe_in_landing_sequence()) {
            // don't failsafe in a landing sequence
            break;
        }

#if HAL_QUADPLANE_ENABLED
        if (quadplane.in_vtol_takeoff()) {
            set_mode(mode_qland, reason);
            // QLAND if in VTOL takeoff
            break;
        }
#endif
        FALLTHROUGH;

    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:

        if(g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if HAL_PARACHUTE_ENABLED
            parachute_release();
#endif
        } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
            set_mode(mode_fbwa, reason);
        } else if (g.fs_action_long == FS_ACTION_LONG_AUTO) {
            set_mode(mode_auto, reason);
#if MODE_AUTOLAND_ENABLED
        } else if (g.fs_action_long == FS_ACTION_LONG_AUTOLAND) {
            if (!set_mode(mode_autoland, reason)) {
               set_mode(mode_rtl, reason);
            } 
#endif           
        } else if (g.fs_action_long == FS_ACTION_LONG_RTL) {
            set_mode(mode_rtl, reason);
        }
        break;
    case Mode::Number::RTL:
        if (g.fs_action_long == FS_ACTION_LONG_AUTO) {
            set_mode(mode_auto, reason);
#if MODE_AUTOLAND_ENABLED
        } else if (g.fs_action_long == FS_ACTION_LONG_AUTOLAND) {
            set_mode(mode_autoland, reason);
#endif
        }
        break;
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
    case Mode::Number::LOITER_ALT_QLAND:
#endif
    case Mode::Number::INITIALISING:
#if MODE_AUTOLAND_ENABLED
    case Mode::Number::AUTOLAND:
#endif
        break;
    }
    gcs().send_text(MAV_SEVERITY_WARNING, "%s Failsafe On: switched to %s", (reason == ModeReason:: GCS_FAILSAFE) ? "GCS" : "RC Long", control_mode->name());
}

void Plane::rc_failsafe_short_off_event()
{
    // We're back in radio contact
    gcs().send_text(MAV_SEVERITY_WARNING, "RC Short Failsafe Cleared");
    failsafe.state = FAILSAFE_NONE;
    // restore entry mode if desired but check that our current mode is still due to failsafe
    if (control_mode_reason == ModeReason::RADIO_FAILSAFE) { 
       set_mode_by_number(failsafe.saved_mode_number, ModeReason::RADIO_FAILSAFE_RECOVERY);
       gcs().send_text(MAV_SEVERITY_INFO,"Flight mode %s restored",control_mode->name());
    }
}

void Plane::failsafe_long_off_event(ModeReason reason)
{
    long_failsafe_pending = false;
    // We're back in radio contact with RC or GCS
    if (reason == ModeReason:: GCS_FAILSAFE) {
        AP_Notify::flags.failsafe_gcs = false;
        gcs().send_text(MAV_SEVERITY_WARNING, "GCS Failsafe Cleared");
    }
    else {
        gcs().send_text(MAV_SEVERITY_WARNING, "RC Long Failsafe Cleared");
    }
    failsafe.state = FAILSAFE_NONE;
}

void Plane::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    switch ((Failsafe_Action)action) {
#if HAL_QUADPLANE_ENABLED
        case Failsafe_Action_Loiter_alt_QLand:
            if (quadplane.available()) {
                plane.set_mode(mode_loiter_qland, ModeReason::BATTERY_FAILSAFE);
                break;
            }
            FALLTHROUGH;

        case Failsafe_Action_QLand:
            if (quadplane.available()) {
                plane.set_mode(mode_qland, ModeReason::BATTERY_FAILSAFE);
                break;
            }
            FALLTHROUGH;
#endif // HAL_QUADPLANE_ENABLED
        case Failsafe_Action_Land: {
            bool already_landing = flight_stage == AP_FixedWing::FlightStage::LAND;
#if HAL_QUADPLANE_ENABLED
            if (control_mode == &mode_qland || control_mode == &mode_loiter_qland) {
                already_landing = true;
            }
#endif
            if (!already_landing && plane.have_position) {
                // never stop a landing if we were already committed
                if (plane.mission.is_best_land_sequence(plane.current_loc)) {
                    // continue mission as it will reach a landing in less distance
                    plane.mission.set_in_landing_sequence_flag(true);
                    break;
                }
                if (plane.mission.jump_to_landing_sequence(plane.current_loc)) {
                    plane.set_mode(mode_auto, ModeReason::BATTERY_FAILSAFE);
                    break;
                }
            }
            FALLTHROUGH;
        }
        case Failsafe_Action_RTL:
        case Failsafe_Action_AUTOLAND_OR_RTL: {
            bool already_landing = flight_stage == AP_FixedWing::FlightStage::LAND;
#if HAL_QUADPLANE_ENABLED
            if (control_mode == &mode_qland || control_mode == &mode_loiter_qland ||
                quadplane.in_vtol_land_sequence()) {
                already_landing = true;
            }
#endif
#if MODE_AUTOLAND_ENABLED
            if (control_mode == &mode_autoland) {
                already_landing = true;
            }
#endif
            if (!already_landing) {
                // never stop a landing if we were already committed
                if ((g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START) && plane.have_position && plane.mission.is_best_land_sequence(plane.current_loc)) {
                    // continue mission as it will reach a landing in less distance
                    plane.mission.set_in_landing_sequence_flag(true);
                    break;
                }
                aparm.throttle_cruise.load();
#if MODE_AUTOLAND_ENABLED
                if (((Failsafe_Action)action == Failsafe_Action_AUTOLAND_OR_RTL) && set_mode(mode_autoland, ModeReason::BATTERY_FAILSAFE)) {
                    break;
                }
#endif
                set_mode(mode_rtl, ModeReason::BATTERY_FAILSAFE);
            }
            break;
        }

        case Failsafe_Action_Terminate:
#if AP_ADVANCEDFAILSAFE_ENABLED
            char battery_type_str[17];
            snprintf(battery_type_str, 17, "%s battery", type_str);
            afs.gcs_terminate(true, battery_type_str);
#else
            arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
#endif
            break;

        case Failsafe_Action_Parachute:
#if HAL_PARACHUTE_ENABLED
            parachute_release();
#endif
            break;

        case Failsafe_Action_None:
            // don't actually do anything, however we should still flag the system as having hit a failsafe
            // and ensure all appropriate flags are going off to the user
            break;
    }
}
