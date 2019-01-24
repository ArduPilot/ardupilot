#include "Copter.h"

// Code to integrate AC_Fence library with main ArduCopter code

#if AC_FENCE == ENABLED

// fence_check - ask fence library to check for breaches and initiate the response
// called at 1hz
void Copter::fence_check()
{
    const uint8_t orig_breaches = fence.get_breaches();

    // check for new breaches; new_breaches is bitmask of fence types breached
    const uint8_t new_breaches = fence.check();

    // we still don't do anything when disarmed, but we do check for fence breaches.
    // fence pre-arm check actually checks if any fence has been breached 
    // that's not ever going to be true if we don't call check on AP_Fence while disarmed.
    if (!motors->armed()) {
        return;
    }

    // if there is a new breach take action
    if (new_breaches) {

        // if the user wants some kind of response and motors are armed
        uint8_t fence_act = fence.get_action();
        if (fence_act != AC_FENCE_ACTION_REPORT_ONLY ) {

            // disarm immediately if we think we are on the ground or in a manual flight mode with zero throttle
            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            if (ap.land_complete || (flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0))){
                init_disarm_motors();

            } else {

                // if more than 100m outside the fence just force a land
                if (fence.get_breach_distance(new_breaches) > AC_FENCE_GIVE_UP_DISTANCE) {
                    set_mode(LAND, MODE_REASON_FENCE_BREACH);
                } else {
                    switch (fence_act) {
                    case AC_FENCE_ACTION_RTL_AND_LAND:
                    default:
                        // switch to RTL, if that fails then Land
                        if (!set_mode(RTL, MODE_REASON_FENCE_BREACH)) {
                            set_mode(LAND, MODE_REASON_FENCE_BREACH);
                        }
                        break;
                    case AC_FENCE_ACTION_ALWAYS_LAND:
                        // if always land option mode is specified, land
                        set_mode(LAND, MODE_REASON_FENCE_BREACH);
                        break;
                    case AC_FENCE_ACTION_SMART_RTL:
                        // Try SmartRTL, if that fails, RTL, if that fails Land
                        if (!set_mode(SMART_RTL, MODE_REASON_FENCE_BREACH)) {
                            if (!set_mode(RTL, MODE_REASON_FENCE_BREACH)) {
                                set_mode(LAND, MODE_REASON_FENCE_BREACH);
                            }
                        }
                        break;
                    case AC_FENCE_ACTION_BRAKE:
                        // Try Brake, if that fails Land
                        if (!set_mode(BRAKE, MODE_REASON_FENCE_BREACH)) {
                            set_mode(LAND, MODE_REASON_FENCE_BREACH);
                        }
                        break;
                    }
                }
            }
        }

        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, new_breaches);

    } else if (orig_breaches) {
        // record clearing of breach
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, ERROR_CODE_ERROR_RESOLVED);
    }
}

// fence_send_mavlink_status - send fence status to ground station
void Copter::fence_send_mavlink_status(mavlink_channel_t chan)
{   
    if (fence.enabled()) {
        // traslate fence library breach types to mavlink breach types
        uint8_t mavlink_breach_type = FENCE_BREACH_NONE;
        uint8_t breaches = fence.get_breaches();
        if ((breaches & AC_FENCE_TYPE_ALT_MAX) != 0) {
            mavlink_breach_type = FENCE_BREACH_MAXALT;
        }
        if ((breaches & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)) != 0) {
            mavlink_breach_type = FENCE_BREACH_BOUNDARY;
        }

        // send status
        mavlink_msg_fence_status_send(chan,
                                      (int8_t)(fence.get_breaches()!=0),
                                      fence.get_breach_count(),
                                      mavlink_breach_type,
                                      fence.get_breach_time());
    }
}

#endif
