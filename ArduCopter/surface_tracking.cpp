#include "Copter.h"

#if AP_RANGEFINDER_ENABLED

// update_surface_offset - manages the vertical offset of the position controller to follow the measured ground or ceiling
//   level measured using the range finder.
void Copter::SurfaceTracking::update_surface_offset()
{
    // check for timeout
    const uint32_t now_ms = millis();
    const bool timeout = (now_ms - last_update_ms) > SURFACE_TRACKING_TIMEOUT_MS;

    // check tracking state and that range finders are healthy
    if (((surface == Surface::GROUND) && copter.rangefinder_alt_ok() && (copter.rangefinder_state.glitch_count == 0)) ||
        ((surface == Surface::CEILING) && copter.rangefinder_up_ok() && (copter.rangefinder_up_state.glitch_count == 0))) {

        // Get the appropriate surface distance state, the terrain offset is calculated in the surface distance lib. 
        AP_SurfaceDistance &rf_state = (surface == Surface::GROUND) ? copter.rangefinder_state : copter.rangefinder_up_state;

        // update position controller target offset to the surface's alt above the EKF origin
        copter.pos_control->set_pos_terrain_target_D_m(-rf_state.terrain_u_m);
        last_update_ms = now_ms;
        valid_for_logging = true;

        // reset target altitude if this controller has just been engaged
        // target has been changed between upwards vs downwards
        // or glitch has cleared
        if (timeout ||
            reset_target ||
            (last_glitch_cleared_ms != rf_state.glitch_cleared_ms)) {
            copter.pos_control->init_pos_terrain_D_m(-rf_state.terrain_u_m);
            reset_target = false;
            last_glitch_cleared_ms = rf_state.glitch_cleared_ms;
        }

    } else {
        // reset position controller offsets if surface tracking is inactive
        // flag target should be reset when/if it next becomes active
        if (timeout && !reset_target) {
            copter.pos_control->init_pos_terrain_D_m(0);
            valid_for_logging = false;
            reset_target = true;
        }
    }
}

// target has already been set by terrain following so do not initialise again
// this should be called by flight modes when switching from terrain following to surface tracking (e.g. ZigZag)
void Copter::SurfaceTracking::external_init()
{
    if ((surface == Surface::GROUND) && copter.rangefinder_alt_ok() && (copter.rangefinder_state.glitch_count == 0)) {
        last_update_ms = millis();
        reset_target = false;
    }
}

bool Copter::SurfaceTracking::get_target_dist_for_logging(float &target_dist_m) const
{
    if (!valid_for_logging || (surface == Surface::NONE)) {
        return false;
    }

    const float dir = (surface == Surface::GROUND) ? 1.0f : -1.0f;
    target_dist_m = dir * copter.pos_control->get_pos_desired_U_m();
    return true;
}

float Copter::SurfaceTracking::get_dist_for_logging() const
{
    return ((surface == Surface::CEILING) ? copter.rangefinder_up_state.alt_m : copter.rangefinder_state.alt_m);
}

// set direction
void Copter::SurfaceTracking::set_surface(Surface new_surface)
{
    if (surface == new_surface) {
        return;
    }
    // check we have a range finder in the correct direction
    if ((new_surface == Surface::GROUND) && !copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "SurfaceTracking: no downward rangefinder");
        AP_Notify::events.user_mode_change_failed = 1;
        return;
    }
    if ((new_surface == Surface::CEILING) && !copter.rangefinder.has_orientation(ROTATION_PITCH_90)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "SurfaceTracking: no upward rangefinder");
        AP_Notify::events.user_mode_change_failed = 1;
        return;
    }
    surface = new_surface;
    reset_target = true;
}

#endif  // AP_RANGEFINDER_ENABLED
