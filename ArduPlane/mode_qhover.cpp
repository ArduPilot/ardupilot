#include "mode.h"
#include "Plane.h"

bool ModeQHover::_enter()
{
    // set vertical speed and acceleration limits
    plane.quadplane.pos_control->set_max_speed_accel_z(-plane.quadplane.get_pilot_velocity_z_max_dn(), plane.quadplane.pilot_velocity_z_max_up, plane.quadplane.pilot_accel_z);
    plane.quadplane.pos_control->set_correction_speed_accel_z(-plane.quadplane.get_pilot_velocity_z_max_dn(), plane.quadplane.pilot_velocity_z_max_up, plane.quadplane.pilot_accel_z);
    plane.quadplane.set_climb_rate_cms(0, false);

    plane.quadplane.init_throttle_wait();
    return true;
}

void ModeQHover::update()
{
    plane.mode_qstabilize.update();
}

void ModeQHover::run()
{
    if (plane.quadplane.throttle_wait) {
        plane.quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        plane.quadplane.attitude_control->set_throttle_out(0, true, 0);
        plane.quadplane.relax_attitude_control();
        plane.quadplane.pos_control->relax_z_controller(0);
        return;
    }
    plane.quadplane.hold_hover(plane.quadplane.get_pilot_desired_climb_rate_cms());
}

