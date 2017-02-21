/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

// Code to integrate AC_Fence library with main ArduCopter code

#if AC_FENCE == ENABLED

// fence_check - ask fence library to check for breaches and initiate the response
// called at 1hz
void Sub::fence_check()
{
    uint8_t new_breaches; // the type of fence that has been breached
    uint8_t orig_breaches = fence.get_breaches();

    // give fence library our current distance from home in meters
    fence.set_home_distance(home_distance*0.01f);

    // check for a breach
    new_breaches = fence.check_fence(current_loc.alt/100.0f);

    // return immediately if motors are not armed
    if(!motors.armed()) {
        return;
    }

    // if there is a new breach take action
    if( new_breaches != AC_FENCE_TYPE_NONE ) {

        // if the user wants some kind of response and motors are armed
        if(fence.get_action() != AC_FENCE_ACTION_REPORT_ONLY ) {

            // disarm immediately if we think we are on the ground or in a manual flight mode with zero throttle
            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            if (ap.land_complete || (mode_has_manual_throttle(control_mode) && ap.throttle_zero && !failsafe.radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0))){
                init_disarm_motors();
            }else{
                // if we are within 100m of the fence, RTL
                if (fence.get_breach_distance(new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
                    if (!set_mode(RTL, MODE_REASON_FENCE_BREACH)) {
                        set_mode(LAND, MODE_REASON_FENCE_BREACH);
                    }
                }else{
                    // if more than 100m outside the fence just force a land
                    set_mode(LAND, MODE_REASON_FENCE_BREACH);
                }
            }
        }

        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, new_breaches);
    }

    // record clearing of breach
    if(orig_breaches != AC_FENCE_TYPE_NONE && fence.get_breaches() == AC_FENCE_TYPE_NONE) {
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, ERROR_CODE_ERROR_RESOLVED);
    }
}

// fence_send_mavlink_status - send fence status to ground station
void Sub::fence_send_mavlink_status(mavlink_channel_t chan)
{   
    if (fence.enabled()) {
        // traslate fence library breach types to mavlink breach types
        uint8_t mavlink_breach_type = FENCE_BREACH_NONE;
        uint8_t breaches = fence.get_breaches();
        if ((breaches & AC_FENCE_TYPE_ALT_MAX) != 0) {
            mavlink_breach_type = FENCE_BREACH_MAXALT;
        }
        if ((breaches & AC_FENCE_TYPE_CIRCLE) != 0) {
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
