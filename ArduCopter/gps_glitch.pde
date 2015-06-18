/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

bool gps_glitch_switch_mode_on_resolve = false;

static void gps_glitch_update() {
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

static bool gps_glitch_action_mode(uint8_t mode) {
    switch(control_mode) {
        case LAND:
            return landing_with_GPS();
        case RTL:
            return rtl_state == Land;
        case GUIDED: // GUIDED for solo because shots modes use GUIDED
        case LOITER:
        case DRIFT:
        case STOP:
        case POSHOLD:
            return true;
        default:
            return false;
    }
    return false;
}

static void gps_glitch_on_event() {
    failsafe.gps_glitch = true;

    if (gps_glitch_action_mode(control_mode) && !failsafe.radio) {
        gps_glitch_switch_mode_on_resolve = true;

        set_mode(ALT_HOLD);
    }
}

static void gps_glitch_off_event() {
    failsafe.gps_glitch = false;

    if (gps_glitch_switch_mode_on_resolve) {
        set_mode(LOITER);
    }
}
