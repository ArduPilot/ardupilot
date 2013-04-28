/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Code to integrate AC_Fence library with main ArduCopter code

#if AC_FENCE == ENABLED

uint8_t lim_state = 0, lim_old_state = 0;

// fence_check - ask fence library to check for breaches and initiate the response
// called at 1hz
void fence_check()
{
    uint8_t new_breaches; // the type of fence that has been breached
    uint8_t orig_breaches = fence.get_breaches();

    // return immediately if motors are not armed
    if(!motors.armed()) {
        return;
    }

    // give fence library our current distance from home
    fence.set_home_distance(home_distance);

    // check for a breach
    new_breaches = fence.check_fence();

    // if there is a new breach take action
    if( new_breaches != AC_FENCE_TYPE_NONE ) {

        // if the user wants some kind of response and motors are armed
        if(fence.get_action() != AC_FENCE_ACTION_REPORT_ONLY ) {

            // disarm immediately we think we are on the ground
            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            if(control_mode <= ACRO && g.rc_3.control_in == 0 && !ap.failsafe_radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0)){
                init_disarm_motors();
            }else{
                // if we have a GPS
                if( ap.home_is_set ) {
                    // if the breach is of the big circle LAND
                    if((new_breaches & AC_FENCE_TYPE_BIG_CIRCLE) > 0) {
                        if(control_mode != LAND) {
                            set_mode(LAND);
                        }
                    }else{
                        // must be a small circle or altitude breach so try to RTL
                        if(control_mode != RTL) {
                            set_mode(RTL);
                        }
                    }
                }else{
                    // we have no GPS so LAND
                    if(control_mode != LAND) {
                        set_mode(LAND);
                    }
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
static void fence_send_mavlink_status(mavlink_channel_t chan)
{
    if (fence.enabled()) {
        mavlink_msg_limits_status_send(chan,
                                       (uint8_t) fence.enabled(),
                                       (uint32_t) fence.get_breach_time(),
                                       (uint32_t) 0,
                                       (uint32_t) 0,
                                       (uint32_t) 0,
                                       (uint16_t) fence.get_breach_count(),
                                       (uint8_t) fence.get_enabled_fences(),
                                       (uint8_t) 0,
                                       (uint8_t) fence.get_breaches());
    }
}

#endif
