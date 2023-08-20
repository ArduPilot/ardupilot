#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQHover::_enter()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_velocity_z_max_up, quadplane.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_velocity_z_max_up, quadplane.pilot_accel_z);
    quadplane.set_climb_rate_cms(0);

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
        pos_control->relax_z_controller(0);
    } else {
        quadplane.hold_hover(quadplane.get_pilot_desired_climb_rate_cms());
    }

    // Stabilize with fixed wing surfaces
    plane.stabilize_roll();
    plane.stabilize_pitch();
}

#endif
