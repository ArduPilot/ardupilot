#include "Copter.h"

// update_surface_offset - manages the vertical offset of the position controller to follow the measured ground or ceiling
//   level measured using the range finder.
void Copter::SurfaceTracking::update_surface_offset()
{
#if RANGEFINDER_ENABLED == ENABLED
    // check for timeout
    const uint32_t now_ms = millis();
    const bool timeout = (now_ms - last_update_ms) > SURFACE_TRACKING_TIMEOUT_MS;

    // check tracking state and that range finders are healthy
    if (((surface == Surface::GROUND) && copter.rangefinder_alt_ok() && (copter.rangefinder_state.glitch_count == 0)) ||
        ((surface == Surface::CEILING) && copter.rangefinder_up_ok() && (copter.rangefinder_up_state.glitch_count == 0))) {

        // calculate surfaces height above the EKF origin
        // e.g. if vehicle is 10m above the EKF origin and rangefinder reports alt of 3m.  curr_surface_alt_above_origin_cm is 7m (or 700cm)
        RangeFinderState &rf_state = (surface == Surface::GROUND) ? copter.rangefinder_state : copter.rangefinder_up_state;
        const float dir = (surface == Surface::GROUND) ? 1.0f : -1.0f;
        const float curr_surface_alt_above_origin_cm = copter.inertial_nav.get_position_z_up_cm() - dir * rf_state.alt_cm;

        // update position controller target offset to the surface's alt above the EKF origin
        copter.pos_control->set_pos_offset_target_z_cm(curr_surface_alt_above_origin_cm);
        last_update_ms = now_ms;
        valid_for_logging = true;

        // reset target altitude if this controller has just been engaged
        // target has been changed between upwards vs downwards
        // or glitch has cleared
        if (timeout ||
            reset_target ||
            (last_glitch_cleared_ms != rf_state.glitch_cleared_ms)) {
            copter.pos_control->set_pos_offset_z_cm(curr_surface_alt_above_origin_cm);
            reset_target = false;
            last_glitch_cleared_ms = rf_state.glitch_cleared_ms;
        }

    } else {
        // reset position controller offsets if surface tracking is inactive
        // flag target should be reset when/if it next becomes active
        if (timeout) {
            copter.pos_control->set_pos_offset_z_cm(0);
            copter.pos_control->set_pos_offset_target_z_cm(0);
            reset_target = true;
        }
    }
#else
    copter.pos_control->set_pos_offset_z_cm(0);
    copter.pos_control->set_pos_offset_target_z_cm(0);
#endif
}


// get target altitude (in cm) above ground
// returns true if there is a valid target
bool Copter::SurfaceTracking::get_target_alt_cm(float &target_alt_cm) const
{
    // fail if we are not tracking downwards
    if (surface != Surface::GROUND) {
        return false;
    }
    // check target has been updated recently
    if (AP_HAL::millis() - last_update_ms > SURFACE_TRACKING_TIMEOUT_MS) {
        return false;
    }
    target_alt_cm = (copter.pos_control->get_pos_target_z_cm() - copter.pos_control->get_pos_offset_z_cm());
    return true;
}

// set target altitude (in cm) above ground
void Copter::SurfaceTracking::set_target_alt_cm(float _target_alt_cm)
{
    // fail if we are not tracking downwards
    if (surface != Surface::GROUND) {
        return;
    }
    copter.pos_control->set_pos_offset_z_cm(copter.inertial_nav.get_position_z_up_cm() - _target_alt_cm);
    last_update_ms = AP_HAL::millis();
}

bool Copter::SurfaceTracking::get_target_dist_for_logging(float &target_dist) const
{
    if (!valid_for_logging || (surface == Surface::NONE)) {
        return false;
    }

    const float dir = (surface == Surface::GROUND) ? 1.0f : -1.0f;
    target_dist = dir * (copter.pos_control->get_pos_target_z_cm() - copter.pos_control->get_pos_offset_z_cm()) * 0.01f;
    return true;
}

float Copter::SurfaceTracking::get_dist_for_logging() const
{
    return ((surface == Surface::CEILING) ? copter.rangefinder_up_state.alt_cm : copter.rangefinder_state.alt_cm) * 0.01f;
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
