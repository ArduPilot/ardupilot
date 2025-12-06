#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQLoiter::_enter()
{
    // initialise loiter
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // set vertical speed and acceleration limits
    // All limits must be positive
    pos_control->D_set_max_speed_accel_m(quadplane.get_pilot_velocity_z_max_dn_m(), quadplane.pilot_speed_z_max_up_ms, quadplane.pilot_accel_z_mss);
    pos_control->D_set_correction_speed_accel_m(quadplane.get_pilot_velocity_z_max_dn_m(), quadplane.pilot_speed_z_max_up_ms, quadplane.pilot_accel_z_mss);

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
    if (quadplane.assist.check_VTOL_recovery()) {
        // use QHover to recover from extreme attitudes, this allows
        // for the fixed wing controller to handle the recovery
        plane.mode_qhover.run();
        return;
    }

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
        if (plane.next_WP_loc.get_vector_xy_from_origin_NE_cm(rel_origin)) {
            quadplane.pos_control->set_pos_desired_NE_m(rel_origin.topostype() * 0.01);
            last_target_loc_set_ms = 0;
        }
    }

    // allow for velocity override as well
    if (last_vel_set_ms != 0 && now - last_vel_set_ms < precland_timeout_ms) {
        // we have an active landing velocity override
        Vector2f target_accel;
        Vector2f target_speed_xy_ms{quadplane.poscontrol.velocity_match_ms.x, quadplane.poscontrol.velocity_match_ms.y};
        quadplane.pos_control->input_vel_accel_NE_m(target_speed_xy_ms, target_accel);
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
        pos_control->D_relax_controller(0);
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
    // All limits must be positive
    pos_control->D_set_max_speed_accel_m(quadplane.get_pilot_velocity_z_max_dn_m(), quadplane.pilot_speed_z_max_up_ms, quadplane.pilot_accel_z_mss);

    // process pilot's roll and pitch input
    float target_roll_cd, target_pitch_cd;
    quadplane.get_pilot_desired_lean_angles(target_roll_cd, target_pitch_cd, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());
    loiter_nav->set_pilot_desired_acceleration_cd(target_roll_cd, target_pitch_cd);
    
    // run loiter controller
    if (!pos_control->NE_is_active()) {
        pos_control->NE_init_controller();
    }
    loiter_nav->update();

    // nav roll and pitch are controller by loiter controller
    plane.nav_roll_cd = loiter_nav->get_roll_cd();
    plane.nav_pitch_cd = loiter_nav->get_pitch_cd();

    plane.quadplane.assign_tilt_to_fwd_thr();

    if (quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
        pos_control->NE_set_externally_limited();
    }

    // Pilot input, use yaw rate time constant
    quadplane.set_pilot_yaw_rate_time_constant();

    Vector3f target { plane.nav_roll_cd*0.01, plane.nav_pitch_cd*0.01, quadplane.get_desired_yaw_rate_cds() * 0.01 };

#if AP_PLANE_SYSTEMID_ENABLED
    auto &systemid = plane.g2.systemid;
    systemid.vtol_update();
    target += systemid.get_attitude_offset_deg();
#endif

    // call attitude controller with conservative smoothing gain of 4.0f
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target.x*100,
                                                                  target.y*100,
                                                                  target.z*100);

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
        float height_above_ground = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);
        float descent_rate_ms = quadplane.landing_descent_rate_ms(height_above_ground);

        if (poscontrol.get_state() == QuadPlane::QPOS_LAND_FINAL && !quadplane.option_is_set(QuadPlane::Option::DISABLE_GROUND_EFFECT_COMP)) {
            ahrs.set_touchdown_expected(true);
        }

        pos_control->D_set_pos_target_from_climb_rate_ms(-descent_rate_ms, descent_rate_ms>0);
        quadplane.check_land_complete();
    } else if (plane.control_mode == &plane.mode_guided && quadplane.guided_takeoff) {
        quadplane.set_climb_rate_ms(0);
    } else {
        // update altitude target and call position controller
        quadplane.set_climb_rate_ms(quadplane.get_pilot_desired_climb_rate_cms() * 0.01);
    }
    quadplane.run_z_controller();

    // Stabilize with fixed wing surfaces
    plane.stabilize_roll();
    plane.stabilize_pitch();

    // Center rudder
    output_rudder_and_steering(0.0);
}

#endif
