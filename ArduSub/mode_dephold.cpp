#include "Sub.h"


bool ModeDephold::init(bool ignore_checks)
{ 
    return ModeAlthold::init(ignore_checks);
}

// depthhold_run - runs the depth hold controller
// should be called at 100hz or more
void ModeDephold::run()
{
    run_pre();
    control_depth();
    run_post();
}

void ModeDephold::control_depth()
{
    float target_climb_rate_cm_s = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -sub.get_pilot_speed_dn(), g.pilot_speed_up);

//    const AP_Doppler_Parameters &params = sub.inertial_doppler.parameters();
    Vector3f vel_body_mps;
    uint32_t t_ms = 0;
    float quality = 0.0f;
    DVL_LockState lock = DVL_LockState::NO_LOCK;
    bool dvl_ok = sub.inertial_doppler.get_velocity_body(vel_body_mps, t_ms, quality, lock);

    if (dvl_ok && fabsf(target_climb_rate_cm_s) < 0.05f) {
        const float dvl_vz_cm_s = vel_body_mps.z * 100.0f;
        target_climb_rate_cm_s = constrain_float(-dvl_vz_cm_s, -sub.get_pilot_speed_dn(), g.pilot_speed_up);
    }

    // desired_climb_rate returns 0 when within the deadzone.
    // we allow full control to the pilot, but as soon as there's no input, we handle being at surface/bottom
    if (fabsf(target_climb_rate_cm_s) < 0.05f)  {
        if (sub.ap.at_surface) {
            position_control->set_pos_target_z_cm(MIN(position_control->get_pos_target_z_cm(), g.surface_depth - 5.0f));
        } else if (sub.ap.at_bottom) {
            position_control->set_pos_target_z_cm(MAX(inertial_nav.get_position_z_up_cm() + 10.0f, position_control->get_pos_target_z_cm()));
        }
    }

    position_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cm_s);
    position_control->update_z_controller();
}
