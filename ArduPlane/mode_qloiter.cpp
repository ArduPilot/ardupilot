#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQLoiter::_enter()
{
    // initialise loiter
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);
    pos_control->set_correction_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);

    quadplane.init_throttle_wait();

    // prevent re-init of target position
    quadplane.last_loiter_ms = AP_HAL::millis();

    // clear precland timestamp
    last_target_loc_set_ms = 0;

    return true;
}

void ModeQLoiter::update()
{
    plane.mode_qstabilize.update();
}

// run quadplane loiter controller
void ModeQLoiter::run()
{
    const uint32_t now = AP_HAL::millis();

#if AC_PRECLAND_ENABLED
    const uint32_t precland_timeout_ms = 250;
    /*
      see if precision landing or precision loiter is active with
      an override of the target location.

    */
    const uint32_t last_pos_set_ms = last_target_loc_set_ms;
    const uint32_t last_vel_set_ms = quadplane.poscontrol.last_velocity_match_ms;

    if (last_pos_set_ms != 0 && now - last_pos_set_ms < precland_timeout_ms) {
        // we have an active landing target override
        Vector2f rel_origin;
        if (plane.next_WP_loc.get_vector_xy_from_origin_NE(rel_origin)) {
            quadplane.pos_control->set_pos_target_xy_cm(rel_origin.x, rel_origin.y);
            last_target_loc_set_ms = 0;
        }
    }

    // allow for velocity override as well
    if (last_vel_set_ms != 0 && now - last_vel_set_ms < precland_timeout_ms) {
        // we have an active landing velocity override
        Vector2f target_accel;
        Vector2f target_speed_xy_cms{quadplane.poscontrol.velocity_match.x*100, quadplane.poscontrol.velocity_match.y*100};
        quadplane.pos_control->input_vel_accel_xy(target_speed_xy_cms, target_accel);
        quadplane.poscontrol.last_velocity_match_ms = 0;
    }
#endif // AC_PRECLAND_ENABLED

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
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();

        // Stabilize with fixed wing surfaces
        plane.stabilize_roll();
        plane.stabilize_pitch();
        return;
    }
    if (!quadplane.motors->armed()) {
        plane.mode_qloiter._enter();
    }

    if (quadplane.should_relax()) {
        loiter_nav->soften_for_landing();
    }

    if (now - quadplane.last_loiter_ms > 500) {
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
    }
    quadplane.last_loiter_ms = now;

    // motors use full range
    quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);

    // process pilot's roll and pitch input
    float target_roll_cd, target_pitch_cd;
    quadplane.get_pilot_desired_lean_angles(target_roll_cd, target_pitch_cd, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());
    loiter_nav->set_pilot_desired_acceleration(target_roll_cd, target_pitch_cd);
    
    // run loiter controller
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }
    loiter_nav->update();

    // nav roll and pitch are controller by loiter controller
    plane.nav_roll_cd = loiter_nav->get_roll();
    plane.nav_pitch_cd = loiter_nav->get_pitch();

    plane.quadplane.assign_tilt_to_fwd_thr();

    if (quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
        pos_control->set_externally_limited_xy();
    }

    // Pilot input, use yaw rate time constant
    quadplane.set_pilot_yaw_rate_time_constant();

    // call attitude controller with conservative smoothing gain of 4.0f
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                  plane.nav_pitch_cd,
                                                                  quadplane.get_desired_yaw_rate_cds());

    if (plane.control_mode == &plane.mode_qland) {
        if (poscontrol.get_state() < QuadPlane::QPOS_LAND_FINAL && quadplane.check_land_final()) {
            poscontrol.set_state(QuadPlane::QPOS_LAND_FINAL);
            quadplane.setup_target_position();
#if AP_ICENGINE_ENABLED
            // cut IC engine if enabled
            if (quadplane.land_icengine_cut != 0) {
                plane.g2.ice_control.engine_control(0, 0, 0, false);
            }
#endif  // AP_ICENGINE_ENABLED
        }
        float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
        float descent_rate_cms = quadplane.landing_descent_rate_cms(height_above_ground);

        if (poscontrol.get_state() == QuadPlane::QPOS_LAND_FINAL && !quadplane.option_is_set(QuadPlane::OPTION::DISABLE_GROUND_EFFECT_COMP)) {
            ahrs.set_touchdown_expected(true);
        }

        pos_control->land_at_climb_rate_cm(-descent_rate_cms, descent_rate_cms>0);
        quadplane.check_land_complete();
    } else if (plane.control_mode == &plane.mode_guided && quadplane.guided_takeoff) {
        quadplane.set_climb_rate_cms(0);
    } else {
        // update altitude target and call position controller
        quadplane.set_climb_rate_cms(quadplane.get_pilot_desired_climb_rate_cms());
    }
    quadplane.run_z_controller();

    // Stabilize with fixed wing surfaces
    plane.stabilize_roll();
    plane.stabilize_pitch();
}

#endif
