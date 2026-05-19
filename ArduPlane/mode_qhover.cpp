#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQHover::_enter()
{
    // set vertical speed and acceleration limits
    // All limits must be positive
    pos_control->D_set_max_speed_accel_m(quadplane.get_pilot_velocity_z_max_dn_m(), quadplane.pilot_speed_z_max_up_ms, quadplane.pilot_accel_z_mss);
    pos_control->D_set_correction_speed_accel_m(quadplane.get_pilot_velocity_z_max_dn_m(), quadplane.pilot_speed_z_max_up_ms, quadplane.pilot_accel_z_mss);
    quadplane.set_climb_rate_ms(0);

    quadplane.init_throttle_wait();
    return true;
}

void ModeQHover::update()
{
    plane.mode_qstabilize.update();
}

/*
  control QHOVER mode
 */
void ModeQHover::run()
{
    quadplane.assist.check_VTOL_recovery();

    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

    if (quadplane.throttle_wait) {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplane.relax_attitude_control();
        pos_control->D_relax_controller(0);
    } else {
        plane.quadplane.assign_tilt_to_fwd_thr();
        quadplane.hold_hover(quadplane.get_pilot_desired_climb_rate_cms());
    }

    // Stabilize with fixed wing surfaces
    plane.stabilize_roll();
    plane.stabilize_pitch();

    // Center rudder
    output_rudder_and_steering(0.0);

    // possibly apply spin recovery
    quadplane.assist.output_spin_recovery();
}

#endif
