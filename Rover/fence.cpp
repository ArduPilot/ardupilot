#include "Rover.h"

// fence_check - ask fence library to check for breaches and initiate the response
void Rover::fence_check()
{
#if AP_FENCE_ENABLED
    uint8_t new_breaches;  // the type of fence that has been breached
    const uint8_t orig_breaches = fence.get_breaches();

    // check for a breach
    new_breaches = fence.check();

    // return immediately if motors are not armed
    if (!arming.is_armed()) {
        return;
    }

    // if there is a new breach take action
    if (new_breaches) {
        // if the user wants some kind of response and motors are armed
        if (fence.get_action() != Failsafe_Action_None) {
            // if within 100m of the fence, it will take the action specified by the FENCE_ACTION parameter
            if (fence.get_breach_distance(new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
                switch (fence.get_action()) {
                case Failsafe_Action_None:
                    break;
                case Failsafe_Action_RTL:
                    if (!set_mode(mode_rtl, ModeReason::FENCE_BREACHED)) {
                        set_mode(mode_hold, ModeReason::FENCE_BREACHED);
                    }
                    break;
                case Failsafe_Action_Hold:
                    set_mode(mode_hold, ModeReason::FENCE_BREACHED);
                    break;
                case Failsafe_Action_SmartRTL:
                    if (!set_mode(mode_smartrtl, ModeReason::FENCE_BREACHED)) {
                        if (!set_mode(mode_rtl, ModeReason::FENCE_BREACHED)) {
                            set_mode(mode_hold, ModeReason::FENCE_BREACHED);
                        }
                    }
                    break;
                case Failsafe_Action_SmartRTL_Hold:
                    if (!set_mode(mode_smartrtl, ModeReason::FENCE_BREACHED)) {
                        set_mode(mode_hold, ModeReason::FENCE_BREACHED);
                    }
                    break;
                }
            } else {
                // if more than 100m outside the fence just force to HOLD
                set_mode(mode_hold, ModeReason::FENCE_BREACHED);
            }
        }
        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(new_breaches));

    } else if (orig_breaches) {
        // record clearing of breach
        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_FENCE,
                                 LogErrorCode::ERROR_RESOLVED);
    }
#endif // AP_FENCE_ENABLED
}
