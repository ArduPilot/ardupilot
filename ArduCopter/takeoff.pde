// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file contains the high-level takeoff logic for Loiter, PosHold, AltHold, Sport modes.
//   The take-off can be initiated from a GCS NAV_TAKEOFF command which includes a takeoff altitude
//   A safe takeoff speed is calculated and used to calculate a time_ms
//   the pos_control target is then slowly increased until time_ms expires

// return true if this flight mode supports user takeoff
//  must_nagivate is true if mode must also control horizontal position
bool current_mode_has_user_takeoff(bool with_navigation)
{
    switch (control_mode) {
        case GUIDED:
        case LOITER:
        case POSHOLD:
            return true;
        case ALT_HOLD:
        case SPORT:
            return !with_navigation;
        default:
            return false;
    }
}

// initiate user takeoff - called when MAVLink TAKEOFF command is received
static bool do_user_takeoff(float takeoff_alt_cm, bool must_navigate)
{
    if (motors.armed() && ap.land_complete && current_mode_has_user_takeoff(must_navigate) && takeoff_alt_cm > current_loc.alt) {
        switch(control_mode) {
            case GUIDED:
                set_auto_armed(true);
                guided_takeoff_start(takeoff_alt_cm);
                return true;
            case LOITER:
            case POSHOLD:
            case ALT_HOLD:
            case SPORT:
                set_auto_armed(true);
                takeoff_timer_start(pv_alt_above_origin(takeoff_alt_cm)-pos_control.get_pos_target().z);
                return true;
        }
    }
    return false;
}

// start takeoff to specified altitude above home
static void takeoff_timer_start(float alt)
{
    // calculate climb rate
    float speed = min(wp_nav.get_speed_up(), max(g.pilot_velocity_z_max*2.0f/3.0f, g.pilot_velocity_z_max-50.0f));

    // sanity check speed and target
    if (takeoff_state.running || speed <= 0.0f || alt <= 0.0f) {
        return;
    }

    // initialise takeoff state
    takeoff_state.running = true;
    takeoff_state.speed = speed;
    takeoff_state.start_ms = millis();
    takeoff_state.time_ms = (alt/takeoff_state.speed) * 1.0e3f;
}

// update takeoff timer and stop the takeoff after time_ms has passed
static void takeoff_timer_update()
{
    if (millis()-takeoff_state.start_ms >= takeoff_state.time_ms) {
        takeoff_state.running = false;
    }
}

// increase altitude target as part of takeoff
//   dt - time interval (in seconds) that this function is called
static void takeoff_increment_alt_target(float dt)
{
    if (takeoff_state.running) {
        Vector3f pos_target = pos_control.get_pos_target();
        pos_target.z += takeoff_state.speed*dt;
        pos_control.set_pos_target(pos_target);
    }
}

// get take off speed in cm/s
static float takeoff_get_speed()
{
    return takeoff_state.running?takeoff_state.speed:0.0f;
}
