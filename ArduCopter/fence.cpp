#include "Copter.h"

// Code to integrate AC_Fence library with main ArduCopter code

#if AP_FENCE_ENABLED

// fence_check - ask fence library to check for breaches and initiate the response
// called at 1hz
void Copter::fence_check()
{
    const uint8_t orig_breaches = fence.get_breaches();

    bool is_landing_or_landed = flightmode->is_landing() || ap.land_complete  || !motors->armed();

    // check for new breaches; new_breaches is bitmask of fence types breached
    const uint8_t new_breaches = fence.check(is_landing_or_landed);

    // we still don't do anything when disarmed, but we do check for fence breaches.
    // fence pre-arm check actually checks if any fence has been breached 
    // that's not ever going to be true if we don't call check on AP_Fence while disarmed.
    if (!motors->armed()) {
        return;
    }

    // if there is a new breach take action
    if (new_breaches) {

        if (!copter.ap.land_complete) {
            fence.print_fence_message("breached", new_breaches);
        }

        // if the user wants some kind of response and motors are armed
        AC_Fence::Action fence_act = static_cast<AC_Fence::Action>(fence.get_action());
        if (fence_act != AC_Fence::Action::REPORT_ONLY ) {

            // disarm immediately if we think we are on the ground or in a manual flight mode with zero throttle
            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            if (ap.land_complete || (flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0))){
                arming.disarm(AP_Arming::Method::FENCEBREACH);

            } else {

                // if more than 100m outside the fence just force a land
                if (fence.get_breach_distance(new_breaches) > AC_FENCE_GIVE_UP_DISTANCE) {
                    set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                } else {
                    switch (fence_act) {
                    case AC_Fence::Action::RTL_AND_LAND:
                    default:
                        // switch to RTL, if that fails then Land
                        if (!set_mode(Mode::Number::RTL, ModeReason::FENCE_BREACHED)) {
                            set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        }
                        break;
                    case AC_Fence::Action::ALWAYS_LAND:
                        // if always land option mode is specified, land
                        set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        break;
                    case AC_Fence::Action::SMART_RTL:
                        // Try SmartRTL, if that fails, RTL, if that fails Land
                        if (!set_mode(Mode::Number::SMART_RTL, ModeReason::FENCE_BREACHED)) {
                            if (!set_mode(Mode::Number::RTL, ModeReason::FENCE_BREACHED)) {
                                set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                            }
                        }
                        break;
                    case AC_Fence::Action::BRAKE:
                        // Try Brake, if that fails Land
                        if (!set_mode(Mode::Number::BRAKE, ModeReason::FENCE_BREACHED)) {
                            set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        }
                        break;
                    case AC_Fence::Action::SMART_RTL_OR_LAND:
                        // Try SmartRTL, if that fails, Land
                        if (!set_mode(Mode::Number::SMART_RTL, ModeReason::FENCE_BREACHED)) {
                            set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        }
                        break;
                    }
                }
            }
        }

        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(new_breaches));

    } else if (orig_breaches && fence.get_breaches() == 0) {
        if (!copter.ap.land_complete) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Fence breach cleared");
        }
        // record clearing of breach
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
}

#endif
