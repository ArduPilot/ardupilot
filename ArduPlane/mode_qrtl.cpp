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
    approach_start.valid = false;
    plane.prev_WP_loc = plane.current_loc;

    int32_t RTL_alt_abs_cm = plane.home.alt + quadplane.qrtl_alt_m*100UL;
    if (quadplane.motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
        // VTOL motors are active, either in VTOL flight or assisted flight
        Location destination = plane.calc_best_rally_or_home_location(plane.current_loc, RTL_alt_abs_cm);
        const float dist = plane.current_loc.get_distance(destination);
        const float radius = get_VTOL_return_radius();

        // Climb at least to a cone around home of height of QRTL alt and radius of radius
        // Always climb up to at least Q_RTL_ALT_MIN, constrain Q_RTL_ALT_MIN between Q_LAND_FINAL_ALT and Q_RTL_ALT
        const float min_climb = constrain_float(quadplane.qrtl_alt_min_m, quadplane.land_final_alt_m, quadplane.qrtl_alt_m);
        const float target_alt = MAX(quadplane.qrtl_alt_m * (dist / MAX(radius, dist)), min_climb);


#if AP_TERRAIN_AVAILABLE
        const bool use_terrain = plane.terrain_enabled_in_mode(mode_number());
#else
        const bool use_terrain = false;
#endif

        const float dist_to_climb = target_alt - plane.relative_ground_altitude(RangeFinderUse::CLIMB, use_terrain);
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
            // Above home "cone", return at current altitude if lower than QRTL alt
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
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

    switch (submode) {
        case SubMode::climb: {
            // request zero velocity
            Vector2f vel, accel;
            pos_control->input_vel_accel_NE_m(vel, accel);
            quadplane.run_xy_controller();

            // nav roll and pitch are controller by position controller
            plane.nav_roll_cd = pos_control->get_roll_cd();
            plane.nav_pitch_cd = pos_control->get_pitch_cd();

            plane.quadplane.assign_tilt_to_fwd_thr();

            if (quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
                pos_control->NE_set_externally_limited();
            }
            // weathervane with no pilot input
            quadplane.disable_yaw_rate_time_constant();
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd,
                                                                          plane.nav_pitch_cd,
                                                                          quadplane.get_weathervane_yaw_rate_cds());

            // climb at full WP nav speed
            quadplane.set_climb_rate_ms(quadplane.wp_nav->get_default_speed_up_ms());
            quadplane.run_z_controller();

            // Climb done when stopping point reaches target altitude
            Vector3p stopping_point_ned_m;
            pos_control->get_stopping_point_D_m(stopping_point_ned_m.z);
            Location stopping_loc = Location::from_ekf_offset_NED_m(stopping_point_ned_m, Location::AltFrame::ABOVE_ORIGIN);

            ftype alt_diff;
            if (!stopping_loc.get_height_above(plane.next_WP_loc, alt_diff) || is_positive(alt_diff)) {
                // climb finished or cant get alt diff, head home
                submode = SubMode::RTL;
                plane.prev_WP_loc = plane.current_loc;

                int32_t RTL_alt_abs_cm = plane.home.alt + quadplane.qrtl_alt_m*100UL;
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
                if (plane.current_loc.get_height_above(plane.next_WP_loc, alt_diff)) {
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
                plane.next_WP_loc.copy_alt_from(plane.home);
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

    // Stabilize with fixed wing surfaces
    plane.stabilize_roll();
    plane.stabilize_pitch();
    plane.stabilize_yaw();
}

/*
  return current height above the QRTL destination altitude, in the same
  frame (terrain-relative if QRTL is terrain following, else absolute) the
  approach altitude ramp in update_target_altitude() is applied in.
  Returns false, leaving alt_delta_m unchanged, if no usable altitude data
  is available -- callers must not treat that the same as a zero delta
 */
bool ModeQRTL::calc_alt_delta_m(float &alt_delta_m) const
{
#if AP_TERRAIN_AVAILABLE
    if (plane.next_WP_loc.terrain_alt) {
        // QRTL is terrain following: compute the delta in the same
        // terrain-relative frame the ramp is applied in via
        // change_target_altitude(), so the two stay consistent over
        // sloping ground. next_WP_loc.alt is already the destination's
        // own terrain-relative target height, in cm. Don't fall back to
        // the absolute-altitude path below on failure -- that would mix
        // an AMSL delta into a terrain-relative ramp
        float current_terrain_height_m;
        if (!plane.terrain.height_above_terrain(current_terrain_height_m, true)) {
            return false;
        }
        alt_delta_m = current_terrain_height_m - (plane.next_WP_loc.alt * 0.01);
        return true;
    }
#endif
    int32_t current_alt_cm;
    int32_t dest_alt_cm;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, current_alt_cm) &&
        plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, dest_alt_cm)) {
        alt_delta_m = (current_alt_cm - dest_alt_cm) * 0.01;
        return true;
    }
    // no usable altitudes
    return false;
}

/*
  update target altitude for QRTL profile
 */
void ModeQRTL::update_target_altitude()
{
    /*
      update height target in approach. This continues through the airbrake,
      POSITION1 and POSITION2 stages -- the same window in which TECS keeps
      running (see QuadPlane::should_disable_TECS()) -- so that the altitude
      TECS is using is never stepped while an abrupt transition out of
      APPROACH/AIRBRAKE (e.g. thrust loss, low airspeed, bad attitude) can
      happen well before the ramp has reached RTL_ALTITUDE
     */
    const QuadPlane::position_control_state qpos_state = plane.quadplane.poscontrol.get_state();
    if ((submode != SubMode::RTL) ||
        (qpos_state < QuadPlane::QPOS_APPROACH) || (qpos_state > QuadPlane::QPOS_POSITION2)) {
        Mode::update_target_altitude();
        return;
    }

    /*
      initially approach at RTL_ALT_CM, then drop down to QRTL_ALT based on maximum sink rate from TECS,
      giving time to lose speed before we transition
     */
    const float radius = MAX(fabsf(float(plane.aparm.loiter_radius)), fabsf(float(plane.g.rtl_radius)));
    const float rtl_alt_delta = MAX(0, plane.g.RTL_altitude - plane.quadplane.qrtl_alt_m);
    const float sink_time = rtl_alt_delta / MAX(0.6*plane.TECS_controller.get_max_sinkrate(), 1);
    const float sink_dist = plane.aparm.airspeed_cruise * sink_time;
    const float dist = plane.auto_state.wp_distance;
    const float rad_min = 2*radius;
    const float rad_max = 20*radius;
    const float dist_rtl_alt_reached = MAX(rad_min, MIN(rad_max, rad_min+sink_dist));

    if (!approach_start.valid) {
        // latch where the approach began, so the ramp below is driven by our
        // own altitude at the start of the approach rather than the fixed wing
        // waypoint offset, which is cleared as the destination is neared.
        // If the altitude query fails, leave valid false so we retry next
        // cycle rather than latching a bogus zero delta that would silently
        // disable the ramp for the rest of the approach
        if (calc_alt_delta_m(approach_start.alt_delta_m)) {
            approach_start.dist_m = dist;
            approach_start.valid = true;
        }
    }

    // Set the target altitude to the QRTL altitude
    plane.set_target_altitude_location(plane.next_WP_loc);

    float alt;
    if (dist > dist_rtl_alt_reached) {
        /*
          still well short of home: instead of immediately targeting RTL_ALTITUDE,
          gradually descend from the altitude we were at when this approach leg
          started down to RTL_ALTITUDE, reaching it at dist_rtl_alt_reached.
          As for fixed wing waypoints, ALT_SLOPE_MIN gates this: setting it to
          zero disables the gradual descent, and altitude changes smaller than
          it are made immediately
         */
        const float alt_excess = approach_start.valid ? (approach_start.alt_delta_m - rtl_alt_delta) : 0;
        if (approach_start.valid && (plane.g.alt_slope_min > 0) && (alt_excess >= plane.g.alt_slope_min) &&
            (approach_start.dist_m > dist_rtl_alt_reached)) {
            alt = linear_interpolate(rtl_alt_delta, approach_start.alt_delta_m,
                                      dist,
                                      dist_rtl_alt_reached, approach_start.dist_m);
            // The ramp above is driven purely by the latched start point and
            // distance, so it doesn't know if we've since fallen below its
            // line, e.g. QRTL was entered while already sinking briskly. In
            // that case don't command a climb back up to the ramp -- follow
            // the aircraft down instead, floored at rtl_alt_delta so this
            // branch hands off to the dist_rtl_alt_reached boundary below at
            // the same altitude that boundary itself targets (still permits
            // climbing back to rtl_alt_delta if we're currently below it,
            // same as the pre-ramp behaviour of targeting RTL_ALTITUDE outright).
            // If this cycle's altitude query fails, skip the clamp rather than
            // folding the failure into a target of rtl_alt_delta
            float current_alt_delta_m;
            if (calc_alt_delta_m(current_alt_delta_m)) {
                alt = MIN(alt, MAX(rtl_alt_delta, current_alt_delta_m));
            }
        } else {
            // gradual descent disabled, nothing worth ramping down from, or
            // the altitude query has never yet succeeded this approach
            alt = rtl_alt_delta;
        }
    } else {
        // Close to home, descend from RTL alt to QRTL alt
        alt = linear_interpolate(0.0, rtl_alt_delta,
                                  dist,
                                  rad_min, dist_rtl_alt_reached);
    }

    // Adjust target altitude based on distance to home
    plane.change_target_altitude(alt * 100);
}

// only nudge during approach
bool ModeQRTL::allows_throttle_nudging() const
{
    return (submode == SubMode::RTL) && (plane.quadplane.poscontrol.get_state() == QuadPlane::QPOS_APPROACH);
}

// Return the radius from destination at which pure VTOL flight should be used, no transition to FW
float ModeQRTL::get_VTOL_return_radius() const
{
    return MAX(fabsf(float(plane.aparm.loiter_radius)), fabsf(float(plane.g.rtl_radius))) * 1.5;
}

#endif
