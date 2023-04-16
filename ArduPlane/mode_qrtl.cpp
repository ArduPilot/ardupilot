#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQRTL::_enter()
{
    // treat QRTL as QLAND if we are in guided wait takeoff state, to cope
    // with failsafes during GUIDED->AUTO takeoff sequence
    if (plane.quadplane.guided_wait_takeoff_on_mode_enter) {
       plane.set_mode(plane.mode_qland, ModeReason::QLAND_INSTEAD_OF_RTL);
       return true;
    }
    submode = SubMode::RTL;
    plane.prev_WP_loc = plane.current_loc;

    int32_t RTL_alt_abs_cm = plane.home.alt + quadplane.qrtl_alt*100UL;
    if (quadplane.motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
        // VTOL motors are active, either in VTOL flight or assisted flight
        Location destination = plane.calc_best_rally_or_home_location(plane.current_loc, RTL_alt_abs_cm);
        const float dist = plane.current_loc.get_distance(destination);
        const float radius = get_VTOL_return_radius();

        // Climb at least to a cone around home of hight of QRTL alt and radius of radius
        // Always climb up to at least Q_RTL_ALT_MIN, constrain Q_RTL_ALT_MIN between Q_LAND_FINAL_ALT and Q_RTL_ALT
        const float min_climb = constrain_float(quadplane.qrtl_alt_min, quadplane.land_final_alt, quadplane.qrtl_alt);
        const float target_alt = MAX(quadplane.qrtl_alt * (dist / MAX(radius, dist)), min_climb);


#if AP_TERRAIN_AVAILABLE
        const bool use_terrain = plane.terrain_enabled_in_mode(mode_number());
#else
        const bool use_terrain = false;
#endif

        const float dist_to_climb = target_alt - plane.relative_ground_altitude(plane.g.rangefinder_landing, use_terrain);
        if (is_positive(dist_to_climb)) {
            // climb before returning, only next waypoint altitude is used
            submode = SubMode::climb;
            plane.next_WP_loc = plane.current_loc;
#if AP_TERRAIN_AVAILABLE
            int32_t curent_alt_terrain_cm;
            if (use_terrain && plane.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, curent_alt_terrain_cm)) {
                plane.next_WP_loc.set_alt_cm(curent_alt_terrain_cm + dist_to_climb * 100UL, Location::AltFrame::ABOVE_TERRAIN);
                return true;
            }
#endif
            plane.next_WP_loc.set_alt_cm(plane.current_loc.alt + dist_to_climb * 100UL, plane.current_loc.get_alt_frame());
            return true;

        } else if (dist < radius) {
            // Above home "cone", return at curent altitude if lower than QRTL alt
            int32_t current_alt_abs_cm;
            if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_abs_cm)) {
                RTL_alt_abs_cm = MIN(RTL_alt_abs_cm, current_alt_abs_cm);
            }

            // we're close to destination and already running VTOL motors, don't transition and don't climb
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 d=%.1f r=%.1f", dist, radius);
            poscontrol.set_state(QuadPlane::QPOS_POSITION1);
        }
    }

    // use do_RTL() to setup next_WP_loc
    plane.do_RTL(RTL_alt_abs_cm);
    quadplane.poscontrol_init_approach();

    int32_t from_alt;
    int32_t to_alt;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,from_alt) && plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,to_alt)) {
        poscontrol.slow_descent = from_alt > to_alt;
        return true;
    }
    // default back to old method
    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
    return true;
}

void ModeQRTL::update()
{
    plane.mode_qstabilize.update();
}

/*
  handle QRTL mode
 */
void ModeQRTL::run()
{
    switch (submode) {
        case SubMode::climb: {
            // request zero velocity
            Vector2f vel, accel;
            pos_control->input_vel_accel_xy(vel, accel);
            quadplane.run_xy_controller();

            // nav roll and pitch are controller by position controller
            plane.nav_roll_cd = pos_control->get_roll_cd();
            plane.nav_pitch_cd = pos_control->get_pitch_cd();
            if (quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
                pos_control->set_externally_limited_xy();
            }
            // weathervane with no pilot input
            quadplane.disable_yaw_rate_time_constant();
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                          plane.nav_pitch_cd,
                                                                          quadplane.get_weathervane_yaw_rate_cds());

            // climb at full WP nav speed
            quadplane.set_climb_rate_cms(quadplane.wp_nav->get_default_speed_up());
            quadplane.run_z_controller();

            // Climb done when stopping point reaches target altitude
            Vector3p stopping_point;
            pos_control->get_stopping_point_z_cm(stopping_point.z);
            Location stopping_loc = Location(stopping_point.tofloat(), Location::AltFrame::ABOVE_ORIGIN);

            ftype alt_diff;
            if (!stopping_loc.get_alt_distance(plane.next_WP_loc, alt_diff) || is_positive(alt_diff)) {
                // climb finshed or cant get alt diff, head home
                submode = SubMode::RTL;
                plane.prev_WP_loc = plane.current_loc;

                int32_t RTL_alt_abs_cm = plane.home.alt + quadplane.qrtl_alt*100UL;
                Location destination = plane.calc_best_rally_or_home_location(plane.current_loc, RTL_alt_abs_cm);
                const float dist = plane.current_loc.get_distance(destination);
                const float radius = get_VTOL_return_radius();
                if (dist < radius) {
                    // if close to home return at current target altitude
                    int32_t target_alt_abs_cm;
                    if (plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, target_alt_abs_cm)) {
                        RTL_alt_abs_cm = MIN(RTL_alt_abs_cm, target_alt_abs_cm);
                    }
                    gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 d=%.1f r=%.1f", dist, radius);
                    poscontrol.set_state(QuadPlane::QPOS_POSITION1);
                }

                plane.do_RTL(RTL_alt_abs_cm);
                quadplane.poscontrol_init_approach();
                if (plane.current_loc.get_alt_distance(plane.next_WP_loc, alt_diff)) {
                    poscontrol.slow_descent = is_positive(alt_diff);
                } else {
                    // default back to old method
                    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
                }
            }
            break;
        }

        case SubMode::RTL: {
            quadplane.vtol_position_controller();
            if (poscontrol.get_state() > QuadPlane::QPOS_POSITION2) {
                // change target altitude to home alt
                plane.next_WP_loc.alt = plane.home.alt;
            }
            if (poscontrol.get_state() >= QuadPlane::QPOS_POSITION2) {
                // start landing logic
                quadplane.verify_vtol_land();
            }

            // when in approach allow stick mixing
            if (quadplane.poscontrol.get_state() == QuadPlane::QPOS_AIRBRAKE ||
                quadplane.poscontrol.get_state() == QuadPlane::QPOS_APPROACH) {
                plane.stabilize_stick_mixing_fbw();
            }
            break;
        }
    }
}

/*
  update target altitude for QRTL profile
 */
void ModeQRTL::update_target_altitude()
{
    /*
      update height target in approach
     */
    if ((submode != SubMode::RTL) || (plane.quadplane.poscontrol.get_state() != QuadPlane::QPOS_APPROACH)) {
        Mode::update_target_altitude();
        return;
    }

    /*
      initially approach at RTL_ALT_CM, then drop down to QRTL_ALT based on maximum sink rate from TECS,
      giving time to lose speed before we transition
     */
    const float radius = MAX(fabsf(plane.aparm.loiter_radius), fabsf(plane.g.rtl_radius));
    const float rtl_alt_delta = MAX(0, plane.g.RTL_altitude_cm*0.01 - plane.quadplane.qrtl_alt);
    const float sink_time = rtl_alt_delta / MAX(0.6*plane.TECS_controller.get_max_sinkrate(), 1);
    const float sink_dist = plane.aparm.airspeed_cruise_cm * 0.01 * sink_time;
    const float dist = plane.auto_state.wp_distance;
    const float rad_min = 2*radius;
    const float rad_max = 20*radius;
    float alt = linear_interpolate(0, rtl_alt_delta,
                                   dist,
                                   rad_min, MAX(rad_min, MIN(rad_max, rad_min+sink_dist)));
    Location loc = plane.next_WP_loc;
    loc.alt += alt*100;
    plane.set_target_altitude_location(loc);
    plane.altitude_error_cm = plane.calc_altitude_error_cm();
}

// only nudge during approach
bool ModeQRTL::allows_throttle_nudging() const
{
    return (submode == SubMode::RTL) && (plane.quadplane.poscontrol.get_state() == QuadPlane::QPOS_APPROACH);
}

// Return the radius from destination at which pure VTOL flight should be used, no transition to FW
float ModeQRTL::get_VTOL_return_radius() const
{
    return MAX(fabsf(plane.aparm.loiter_radius), fabsf(plane.g.rtl_radius)) * 1.5;
}

#endif
