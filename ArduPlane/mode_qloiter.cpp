#include "mode.h"
#include "Plane.h"

bool ModeQLoiter::_enter()
{
    // initialise loiter
    plane.quadplane.loiter_nav->clear_pilot_desired_acceleration();
    plane.quadplane.loiter_nav->init_target();

    // set vertical speed and acceleration limits
    plane.quadplane.pos_control->set_max_speed_accel_z(-plane.quadplane.get_pilot_velocity_z_max_dn(), plane.quadplane.pilot_velocity_z_max_up, plane.quadplane.pilot_accel_z);
    plane.quadplane.pos_control->set_correction_speed_accel_z(-plane.quadplane.get_pilot_velocity_z_max_dn(), plane.quadplane.pilot_velocity_z_max_up, plane.quadplane.pilot_accel_z);

    plane.quadplane.init_throttle_wait();

    // remember initial pitch
    plane.quadplane.loiter_initial_pitch_cd = MAX(plane.ahrs.pitch_sensor, 0);

    // prevent re-init of target position
    plane.quadplane.last_loiter_ms = AP_HAL::millis();

    return true;
}

void ModeQLoiter::update()
{
    plane.mode_qstabilize.update();
}

// run quadplane loiter controller
void ModeQLoiter::run()
{
    if (plane.quadplane.throttle_wait) {
        plane.quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        plane.quadplane.attitude_control->set_throttle_out(0, true, 0);
        plane.quadplane.relax_attitude_control();
        plane.quadplane.pos_control->relax_z_controller(0);
        plane.quadplane.loiter_nav->clear_pilot_desired_acceleration();
        plane.quadplane.loiter_nav->init_target();
        return;
    }
    if (!plane.quadplane.motors->armed()) {
        _enter();
    }

    plane.quadplane.check_attitude_relax();

    if (plane.quadplane.should_relax()) {
        plane.quadplane.loiter_nav->soften_for_landing();
    }

    const uint32_t now = AP_HAL::millis();
    if (now - plane.quadplane.last_loiter_ms > 500) {
        plane.quadplane.loiter_nav->clear_pilot_desired_acceleration();
        plane.quadplane.loiter_nav->init_target();
    }
    plane.quadplane.last_loiter_ms = now;

    // motors use full range
    plane.quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set vertical speed and acceleration limits
    plane.quadplane.pos_control->set_max_speed_accel_z(-plane.quadplane.get_pilot_velocity_z_max_dn(), plane.quadplane.pilot_velocity_z_max_up, plane.quadplane.pilot_accel_z);

    // process pilot's roll and pitch input
    float target_roll_cd, target_pitch_cd;
    plane.quadplane.get_pilot_desired_lean_angles(target_roll_cd, target_pitch_cd, plane.quadplane.loiter_nav->get_angle_max_cd(), plane.quadplane.attitude_control->get_althold_lean_angle_max());
    plane.quadplane.loiter_nav->set_pilot_desired_acceleration(target_roll_cd, target_pitch_cd);
    
    // run loiter controller
    if (!plane.quadplane.pos_control->is_active_xy()) {
        plane.quadplane.pos_control->init_xy_controller();
    }
    plane.quadplane.loiter_nav->update();

    // nav roll and pitch are controller by loiter controller
    plane.nav_roll_cd = plane.quadplane.loiter_nav->get_roll();
    plane.nav_pitch_cd = plane.quadplane.loiter_nav->get_pitch();

    if (now - plane.quadplane.last_pidz_init_ms < (uint32_t)plane.quadplane.transition_time_ms*2 && !plane.quadplane.is_tailsitter()) {
        // we limit pitch during initial transition
        float pitch_limit_cd = linear_interpolate(plane.quadplane.loiter_initial_pitch_cd, plane.quadplane.aparm.angle_max,
                                                  now,
                                                  plane.quadplane.last_pidz_init_ms, plane.quadplane.last_pidz_init_ms+plane.quadplane.transition_time_ms*2);
        if (plane.nav_pitch_cd > pitch_limit_cd) {
            plane.nav_pitch_cd = pitch_limit_cd;
            plane.quadplane.pos_control->set_externally_limited_xy();
        }
    }
    
    
    // call attitude controller with conservative smoothing gain of 4.0f
    plane.quadplane.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                                  plane.nav_pitch_cd,
                                                                                  plane.quadplane.get_desired_yaw_rate_cds());

    if (plane.control_mode == &plane.mode_qland) {
        if (plane.quadplane.poscontrol.get_state() < QuadPlane::position_control_state::QPOS_LAND_FINAL && plane.quadplane.check_land_final()) {
            plane.quadplane.poscontrol.set_state(QuadPlane::position_control_state::QPOS_LAND_FINAL);
            plane.quadplane.setup_target_position();
            // cut IC engine if enabled
            if (plane.quadplane.land_icengine_cut != 0) {
                plane.g2.ice_control.engine_control(0, 0, 0);
            }
        }
        float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
        float descent_rate_cms = plane.quadplane.landing_descent_rate_cms(height_above_ground);

        if (plane.quadplane.poscontrol.get_state() == QuadPlane::position_control_state::QPOS_LAND_FINAL && (plane.quadplane.options & QuadPlane::OPTION_DISABLE_GROUND_EFFECT_COMP) == 0) {
            plane.ahrs.set_touchdown_expected(true);
        }

        plane.quadplane.set_climb_rate_cms(-descent_rate_cms, descent_rate_cms>0);
        plane.quadplane.check_land_complete();
    } else if (plane.control_mode == &plane.mode_guided && plane.quadplane.guided_takeoff) {
        plane.quadplane.set_climb_rate_cms(0, false);
    } else {
        // update altitude target and call position controller
        plane.quadplane.set_climb_rate_cms(plane.quadplane.get_pilot_desired_climb_rate_cms(), false);
    }
    plane.quadplane.run_z_controller();
}

