#include "Rover.h"

// fence_check - ask fence library to check for breaches and initiate the response
void Rover::fence_check()
{
    uint8_t new_breaches;  // the type of fence that has been breached
    const uint8_t orig_breaches = g2.fence.get_breaches();

    // check for a breach
    new_breaches = g2.fence.check();

    // return immediately if motors are not armed
    if (!arming.is_armed()) {
        return;
    }

    // if there is a new breach take action
    if (new_breaches) {
        // if the user wants some kind of response and motors are armed
        if (g2.fence.get_action() != AC_FENCE_ACTION_REPORT_ONLY) {
            // if we are within 100m of the fence, RTL
            if (g2.fence.get_breach_distance(new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
                if (!set_mode(mode_rtl, MODE_REASON_FENCE_BREACH)) {
                    set_mode(mode_hold, MODE_REASON_FENCE_BREACH);
                }
            } else {
                // if more than 100m outside the fence just force to HOLD
                set_mode(mode_hold, MODE_REASON_FENCE_BREACH);
            }
        }
        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, new_breaches);

    } else if (orig_breaches) {
        // record clearing of breach
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, ERROR_CODE_ERROR_RESOLVED);
    }
}
