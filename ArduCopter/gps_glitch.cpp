/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"
static uint8_t mode_post_glitch;
// poll NavEKF for gps quality as seen by it and take relevant action 
void Copter::gps_glitch_update() {

    if(g.fs_gps_glitch_type == FS_GPS_GLITCH_DISABLED) {
        return;
    }
    if(g.fs_gps_glitch_type == FS_GPS_GLITCH_ENABLED_NON_AUTO && 
       (control_mode == AUTO || control_mode == GUIDED)) {
        return;
    }
    bool glitch = ahrs.get_NavEKF().getGpsGlitchStatus();

    if (glitch && !failsafe.gps_glitch) {
        gps_glitch_on_event();
    } else if (!glitch && failsafe.gps_glitch) {
        gps_glitch_off_event();
    }

    if (failsafe.gps_glitch && control_mode != ALT_HOLD) {
        // if the mode has changed during gps glitch, don't return to LOITER on resolve
        gps_glitch_switch_mode_on_resolve = false;
    }
}

void Copter::gps_glitch_mode_change_commanded(uint8_t mode_commanded)
{
    // ensure that commanded mode switches to ALT_HOLD will cancel the switch back to FLY
    if (mode_commanded == ALT_HOLD) {
        gps_glitch_switch_mode_on_resolve = false;
    }
}

// decide gps glitch failsafe action as per mode
bool Copter::gps_glitch_action_mode(uint8_t mode) {
    mode_post_glitch = mode;
    switch(control_mode) {
        case LAND:
            return landing_with_GPS();
        case RTL:
            return rtl_state == RTL_Land;
        case GUIDED: // GUIDED for solo because shots modes use GUIDED
        case AUTO:
            mode_post_glitch = LOITER;
        case LOITER:
        case DRIFT:
        case BRAKE:
        case POSHOLD:
            return true;
        default:
            return false;
    }
    return false;
}

// action when gps glitch failsafe is triggered
void Copter::gps_glitch_on_event() {
    failsafe.gps_glitch = true;

    if (motors.armed() && gps_glitch_action_mode(control_mode) && !failsafe.radio) {
        if(set_mode(ALT_HOLD)) {
            gps_glitch_switch_mode_on_resolve = true;
        }
    }
}


// action when gps glitch has gone away
void Copter::gps_glitch_off_event() {
    failsafe.gps_glitch = false;

    if (gps_glitch_switch_mode_on_resolve) {
        set_mode(mode_post_glitch);
    }
}