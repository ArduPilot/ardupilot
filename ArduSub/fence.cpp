#include "Sub.h"

// Code to integrate AC_Fence library with main ArduSub code

#if AP_FENCE_ENABLED

// fence_check - ask fence library to check for breaches and initiate the response
// async fence checking io callback at 1Khz
void Sub::fence_checks_async()
{
    const uint32_t now = AP_HAL::millis();
    // ignore any fence activity when not armed
    if (!motors.armed()) {
        return;
    }

    if (!AP_HAL::timeout_expired(fence_breaches.last_check_ms, now, 333U)) { // 3Hz update rate
        return;
    }

    fence_breaches.last_check_ms = now;
    const uint8_t orig_breaches = fence.get_breaches();
    // check for new breaches; new_breaches is bitmask of fence types breached
    const uint8_t new_breaches = fence.check();

    // if the user wants some kind of response and motors are armed
    if (new_breaches) {
        if (fence.get_action() != AC_Fence::Action::REPORT_ONLY) {
            //
            //            // disarm immediately if we think we are on the ground or in a manual flight mode with zero throttle
            //            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            //            if (ap.land_complete || (mode_has_manual_throttle(control_mode) && ap.throttle_zero && !failsafe.manual_control && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0))){
            //                init_disarm_motors();
            //            }else{
            //                // if we are within 100m of the fence, RTL
            //                if (fence.get_breach_distance(new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
            //                    if (!set_mode(RTL, MODE_REASON_FENCE_BREACH)) {
            //                        set_mode(LAND, MODE_REASON_FENCE_BREACH);
            //                    }
            //                }else{
            //                    // if more than 100m outside the fence just force a land
            //                    set_mode(LAND, MODE_REASON_FENCE_BREACH);
            //                }
            //            }
        }

        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(fence_breaches.new_breaches));

    } else if (orig_breaches) {
        // record clearing of breach
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
}

#endif
