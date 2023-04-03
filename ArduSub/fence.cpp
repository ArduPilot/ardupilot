#include "Sub.h"

// Code to integrate AC_Fence library with main ArduSub code

#if AP_FENCE_ENABLED

// fence_check - ask fence library to check for breaches and initiate the response
// called at 1hz
void Sub::fence_check()
{
    // ignore any fence activity when not armed
    if (!motors.armed()) {
        return;
    }

    const uint8_t orig_breaches = fence.get_breaches();

    // check for new breaches; new_breaches is bitmask of fence types breached
    const uint8_t new_breaches = sub.fence.check();

    // if there is a new breach take action
    if (new_breaches) {

        // if the user wants some kind of response and motors are armed
        if (fence.get_action() != AC_FENCE_ACTION_REPORT_ONLY) {
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

        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(new_breaches));
    } else if (orig_breaches) {
        // record clearing of breach
        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
}

#endif
