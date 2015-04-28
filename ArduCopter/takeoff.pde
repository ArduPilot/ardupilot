// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


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
                tkoff_timer_start(pv_alt_above_origin(takeoff_alt_cm)-pos_control.get_pos_target().z);
                return true;
        }
    }
    return false;
}



static void tkoff_timer_start(float alt)
{
    float speed = min(wp_nav.get_speed_up(), max(g.pilot_velocity_z_max*2.0f/3.0f, g.pilot_velocity_z_max-50.0f));

    if (takeoff_state.running || speed <= 0.0f || alt <= 0.0f) {
        return;
    }

    takeoff_state.running = true;
    takeoff_state.speed = speed;
    takeoff_state.start_ms = millis();
    takeoff_state.time_ms = (alt/takeoff_state.speed) * 1.0e3f;
}

static void tkoff_timer_update()
{
    if (millis()-takeoff_state.start_ms >= takeoff_state.time_ms) {
        takeoff_state.running = false;
    }
}

static void tkoff_increment_alt_target(float dt)
{
    if (takeoff_state.running) {
        Vector3f pos_target = pos_control.get_pos_target();
        pos_target.z += takeoff_state.speed*dt;
        pos_control.set_pos_target(pos_target);
    }
}

static float tkoff_get_speed()
{
    return takeoff_state.running?takeoff_state.speed:0.0f;
}
