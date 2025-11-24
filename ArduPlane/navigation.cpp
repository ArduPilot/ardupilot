#include "Plane.h"

/*
  reset the total loiter angle
 */
void Plane::loiter_angle_reset(void)
{
    loiter.sum_cd = 0;
    loiter.total_cd = 0;
    loiter.reached_target_alt = false;
    loiter.unable_to_achieve_target_alt = false;
}

/*
  update the total angle we have covered in a loiter. Used to support
  commands to do N circles of loiter
 */
void Plane::loiter_angle_update(void)
{
    static const int32_t lap_check_interval_cd = 3*36000;

    const int32_t target_bearing_cd = nav_controller->target_bearing_cd();
    int32_t loiter_delta_cd;
    const bool reached_target = reached_loiter_target();

    if (loiter.sum_cd == 0 && !reached_target) {
        // we don't start summing until we are doing the real loiter
        loiter_delta_cd = 0;
    } else if (loiter.sum_cd == 0) {
        // use 1 cd for initial delta
        loiter_delta_cd = 1;
        loiter.start_lap_alt_cm = current_loc.alt;
        loiter.next_sum_lap_cd = lap_check_interval_cd;
    } else {
        loiter_delta_cd = target_bearing_cd - loiter.old_target_bearing_cd;
    }

    loiter.old_target_bearing_cd = target_bearing_cd;
    loiter_delta_cd = wrap_180_cd(loiter_delta_cd);
    loiter.sum_cd += loiter_delta_cd * loiter.direction;

    bool reached_target_alt = false;

    if (reached_target) {
        // once we reach the position target we start checking the
        // altitude target
        bool terrain_status_ok = false;
#if AP_TERRAIN_AVAILABLE
        /*
          if doing terrain following then we check against terrain
          target, fetch the terrain information
        */
        float altitude_agl = 0;
        if (target_altitude.terrain_following) {
            if (terrain.status() == AP_Terrain::TerrainStatusOK &&
                terrain.height_above_terrain(altitude_agl, true)) {
                terrain_status_ok = true;
            }
        }
        if (terrain_status_ok &&
            fabsF(altitude_agl - target_altitude.terrain_alt_cm*0.01) < 5) {
            reached_target_alt = true;
        } else
#endif
        if (!terrain_status_ok && labs(current_loc.alt - target_altitude.amsl_cm) < 500) {
            reached_target_alt = true;
        }
    }

    if (reached_target_alt) {
        loiter.reached_target_alt = true;
        loiter.unable_to_achieve_target_alt = false;
        loiter.next_sum_lap_cd = loiter.sum_cd + lap_check_interval_cd;

    } else if (!loiter.reached_target_alt && labs(loiter.sum_cd) >= loiter.next_sum_lap_cd) {
        // check every few laps for scenario where up/downward inhibit you from loitering up/down for too long
        loiter.unable_to_achieve_target_alt = labs(current_loc.alt - loiter.start_lap_alt_cm) < 500;
        loiter.start_lap_alt_cm = current_loc.alt;
        loiter.next_sum_lap_cd += lap_check_interval_cd;
    }
}

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
void Plane::navigate()
{
    // do not navigate with corrupt data
    // ---------------------------------
    if (!have_position) {
        return;
    }

    if (next_WP_loc.lat == 0 && next_WP_loc.lng == 0) {
        return;
    }

    check_home_alt_change();

    // waypoint distance from plane
    // ----------------------------
    auto_state.wp_distance = current_loc.get_distance(next_WP_loc);
    auto_state.wp_proportion = current_loc.line_path_proportion(prev_WP_loc, next_WP_loc);
    TECS_controller.set_path_proportion(auto_state.wp_proportion);

    // update total loiter angle
    loiter_angle_update();

    // control mode specific updates to navigation demands
    // ---------------------------------------------------
    control_mode->navigate();
}

// method intended for use in calc_airspeed_errors only
float Plane::mode_auto_target_airspeed_cm()
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.landing_with_fixed_wing_spiral_approach() &&
        ((vtol_approach_s.approach_stage == VTOLApproach::Stage::APPROACH_LINE) ||
         (vtol_approach_s.approach_stage == VTOLApproach::Stage::VTOL_LANDING))) {
        const float land_airspeed = TECS_controller.get_land_airspeed();
        if (is_positive(land_airspeed)) {
            return land_airspeed * 100;
        }
        // fallover to normal airspeed
        return aparm.airspeed_cruise*100;
    }
    if (quadplane.in_vtol_land_approach()) {
        return quadplane.get_land_airspeed_ms() * 100;
    }
#endif

    // normal AUTO mode and new_airspeed variable was set by
    // DO_CHANGE_SPEED command while in AUTO mode
    if (new_airspeed_cm > 0) {
        return new_airspeed_cm;
    }

    // fallover to normal airspeed
    return aparm.airspeed_cruise*100;
}

void Plane::calc_airspeed_errors()
{
    // Get the airspeed_estimate, update smoothed airspeed estimate
    // NOTE:  we use the airspeed estimate function not direct sensor
    //        as TECS may be using synthetic airspeed
    float airspeed_measured = 0.1;
    if (ahrs.airspeed_EAS(airspeed_measured)) {
        smoothed_airspeed = MAX(0.1, smoothed_airspeed * 0.8f + airspeed_measured * 0.2f);
    }

    // low pass filter speed scaler, with 1Hz cutoff, at 10Hz
    const float speed_scaler = calc_speed_scaler();
    const float cutoff_Hz = 2.0;
    const float dt = 0.1;
    surface_speed_scaler += calc_lowpass_alpha_dt(dt, cutoff_Hz) * (speed_scaler - surface_speed_scaler);


    // FBW_B/cruise airspeed target
    if (!failsafe.rc_failsafe && (control_mode == &mode_fbwb || control_mode == &mode_cruise)) {
        if (flight_option_enabled(FlightOptions::CRUISE_TRIM_AIRSPEED)) {
            target_airspeed_cm = aparm.airspeed_cruise*100;
        } else if (flight_option_enabled(FlightOptions::CRUISE_TRIM_THROTTLE)) {
            float control_min = 0.0f;
            float control_mid = 0.0f;
            const float control_max = channel_throttle->get_range();
            const float control_in = get_throttle_input();
            switch (channel_throttle->get_type()) {
            case RC_Channel::ControlType::ANGLE:
                    control_min = -control_max;
                    break;
            case RC_Channel::ControlType::RANGE:
                    control_mid = channel_throttle->get_control_mid();
                    break;
            }
            if (control_in <= control_mid) {
                target_airspeed_cm = linear_interpolate(aparm.airspeed_min * 100, aparm.airspeed_cruise*100,
                                                        control_in,
                                                        control_min, control_mid);
            } else {
                target_airspeed_cm = linear_interpolate(aparm.airspeed_cruise*100, aparm.airspeed_max * 100,
                                                        control_in,
                                                        control_mid, control_max);
            }
        } else {
            target_airspeed_cm = ((int32_t)(aparm.airspeed_max - aparm.airspeed_min) *
                                  get_throttle_input()) + ((int32_t)aparm.airspeed_min * 100);
        }
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    } else if (control_mode == &mode_guided && guided_state.target_airspeed_cm >  0.0) { // if offboard guided speed change cmd not set, then this section is skipped
        // offboard airspeed demanded
        uint32_t now = AP_HAL::millis();
        float delta = 1e-3f * (now - guided_state.target_airspeed_time_ms);
        guided_state.target_airspeed_time_ms = now;
        float delta_amt = 100 * delta * guided_state.target_airspeed_accel;
        target_airspeed_cm += delta_amt;

        //target_airspeed_cm recalculated then clamped to between MIN airspeed and MAX airspeed by constrain_float
        if (is_positive(guided_state.target_airspeed_accel)) {
            target_airspeed_cm = constrain_float(MIN(guided_state.target_airspeed_cm, target_airspeed_cm), aparm.airspeed_min *100, aparm.airspeed_max *100);
        } else {
            target_airspeed_cm = constrain_float(MAX(guided_state.target_airspeed_cm, target_airspeed_cm), aparm.airspeed_min *100, aparm.airspeed_max *100);
        }

#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED

#if HAL_SOARING_ENABLED
    } else if (g2.soaring_controller.is_active() && g2.soaring_controller.get_throttle_suppressed()) {
        if (control_mode == &mode_thermal) {
            float arspd = g2.soaring_controller.get_thermalling_target_airspeed();

            if (arspd > 0) {
                target_airspeed_cm = arspd * 100;
            } else {
                target_airspeed_cm = aparm.airspeed_cruise*100;
            }
        } else if (control_mode == &mode_auto) {
            float arspd = g2.soaring_controller.get_cruising_target_airspeed();

            if (arspd > 0) {
                target_airspeed_cm = arspd * 100;
            } else {
                target_airspeed_cm = aparm.airspeed_cruise*100;
            }
        }
#endif

    } else if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        // Landing airspeed target
        target_airspeed_cm = landing.get_target_airspeed_cm();
    } else if (control_mode == &mode_guided && new_airspeed_cm > 0) { //DO_CHANGE_SPEED overrides onboard guided speed commands, user would have re-enter guided mode to revert
                       target_airspeed_cm = new_airspeed_cm;
    } else if (control_mode == &mode_auto) {
        target_airspeed_cm = mode_auto_target_airspeed_cm();
#if HAL_QUADPLANE_ENABLED
    } else if (control_mode == &mode_qrtl && quadplane.in_vtol_land_approach()) {
        target_airspeed_cm = quadplane.get_land_airspeed_ms() * 100;
#endif
    } else {
        // Normal airspeed target for all other cases
        target_airspeed_cm = aparm.airspeed_cruise*100;
    }

    // Set target to current airspeed + ground speed undershoot,
    // but only when this is faster than the target airspeed commanded
    // above.
    if (control_mode->does_auto_throttle() &&
        groundspeed_undershoot_is_valid &&
        control_mode != &mode_circle) {
        /*
          calculate how much extra airspeed we need to target to
          achieve the desired ground speed in MIN_GROUNDSPEED

          we quantise the additional airspeed and apply a hysteresis
          in order to avoid triggering an oscillation in TECS
         */
        float target_airspeed = target_airspeed_cm*0.01;
        float EAS_undershoot = (groundspeed_undershoot*0.01) / ahrs.get_EAS2TAS();
        float min_gnd_target_airspeed = airspeed_measured + EAS_undershoot;
        float airspeed_target_offset = min_gnd_target_airspeed > target_airspeed? (min_gnd_target_airspeed - target_airspeed) : 0;

        // round up to nearest m/s
        airspeed_target_offset = int(airspeed_target_offset + 0.5);

        // apply some hysteresis
        if (airspeed_target_offset < last_groundspeed_undershoot_offset &&
            last_groundspeed_undershoot_offset - airspeed_target_offset < 1.2) {
            airspeed_target_offset = last_groundspeed_undershoot_offset;
        }
        last_groundspeed_undershoot_offset = airspeed_target_offset;

        target_airspeed_cm += airspeed_target_offset * 100;
    }

    // when using the special GUIDED mode features for slew control, don't allow airspeed nudging as it doesn't play nicely.
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    if (control_mode == &mode_guided && !is_zero(guided_state.target_airspeed_cm) && (airspeed_nudge_cm != 0)) {
        airspeed_nudge_cm = 0; //airspeed_nudge_cm forced to zero
    }
#endif

    // Bump up the target airspeed based on throttle nudging
    if (control_mode->allows_throttle_nudging() && airspeed_nudge_cm > 0) {
        target_airspeed_cm += airspeed_nudge_cm;
    }

    float airspeed_lower_bound = is_positive(aparm.airspeed_stall)
                                     ? aparm.airspeed_stall
                                     : aparm.airspeed_min;

    // Apply airspeed limit
    target_airspeed_cm = constrain_int32(target_airspeed_cm, airspeed_lower_bound*100, aparm.airspeed_max*100);

    // use the TECS view of the target airspeed for reporting, to take
    // account of the landing speed
    airspeed_error = TECS_controller.get_target_airspeed() - airspeed_measured;
}

void Plane::calc_gndspeed_undershoot()
{
    // Use the component of ground speed in the forward direction
    // This prevents flyaway if wind takes plane backwards
    Vector3f velNED;
    if (ahrs.have_inertial_nav() && ahrs.get_velocity_NED(velNED)) {
        const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
        Vector2f yawVect = Vector2f(rotMat.a.x,rotMat.b.x);
        if (!yawVect.is_zero()) {
            yawVect.normalize();
            float gndSpdFwd = yawVect * velNED.xy();
            groundspeed_undershoot_is_valid = aparm.min_groundspeed > 0;
            groundspeed_undershoot = groundspeed_undershoot_is_valid ? (aparm.min_groundspeed*100 - gndSpdFwd*100) : 0;
        }
    } else {
        groundspeed_undershoot_is_valid = false;
        groundspeed_undershoot = 0;
    }
}

// method intended to be used by update_loiter
void Plane::update_loiter_update_nav(uint16_t radius)
{
#if HAL_QUADPLANE_ENABLED
    if (loiter.start_time_ms != 0 &&
        quadplane.guided_mode_enabled()) {
        if (!auto_state.vtol_loiter) {
            auto_state.vtol_loiter = true;
            // reset loiter start time, so we don't consider the point
            // reached till we get much closer
            loiter.start_time_ms = 0;
            quadplane.guided_start();
        }
        return;
    }
#endif

#if HAL_QUADPLANE_ENABLED
    const bool quadplane_qrtl_switch = (control_mode == &mode_rtl && quadplane.available() && quadplane.rtl_mode == QuadPlane::RTL_MODE::SWITCH_QRTL);
#else
    const bool quadplane_qrtl_switch = false;
#endif

    if ((loiter.start_time_ms == 0 &&
         (control_mode == &mode_auto || control_mode == &mode_guided) &&
         auto_state.crosstrack &&
         current_loc.get_distance(next_WP_loc) > 3 * nav_controller->loiter_radius(radius)) ||
        quadplane_qrtl_switch) {
        /*
          if never reached loiter point and using crosstrack and somewhat far away from loiter point
          navigate to it like in auto-mode for normal crosstrack behavior

          we also use direct waypoint navigation if we are a quadplane
          that is going to be switching to QRTL when it gets within
          RTL_RADIUS
        */
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
        return;
    }
    nav_controller->update_loiter(next_WP_loc, radius, loiter.direction);
}

void Plane::update_loiter(uint16_t radius)
{
    if (radius <= 1) {
        // if radius is <=1 then use the general loiter radius. if it's small, use default
        radius = (abs(aparm.loiter_radius) <= 1) ? LOITER_RADIUS_DEFAULT : abs(aparm.loiter_radius);
        if (next_WP_loc.loiter_ccw == 1) {
            loiter.direction = -1;
        } else {
            loiter.direction = (aparm.loiter_radius < 0) ? -1 : 1;
        }
    }

    // the radius actually being used by the controller is required by other functions
    loiter.radius = (float)radius;

    update_loiter_update_nav(radius);

    if (loiter.start_time_ms == 0) {
        if (reached_loiter_target() ||
            auto_state.wp_proportion > 1) {
            // we've reached the target, start the timer
            loiter.start_time_ms = millis();
            if (control_mode->is_guided_mode()) {
                // starting a loiter in GUIDED means we just reached the target point
                gcs().send_mission_item_reached_message(0);
            }
#if HAL_QUADPLANE_ENABLED
            if (quadplane.guided_mode_enabled()) {
                quadplane.guided_start();
            }
#endif
        }
    }
}

/*
  handle speed and height control in FBWB, CRUISE, and optionally, LOITER mode.
  In this mode the elevator is used to change target altitude. The
  throttle is used to change target airspeed or throttle
 */
void Plane::update_fbwb_speed_height(void)
{
    uint32_t now = micros();
    if (now - target_altitude.last_elev_check_us >= 100000) {
        // we don't run this on every loop as it would give too small granularity on quadplanes at 300Hz, and
        // give below 1cm altitude change, which would result in no climb or descent
        float dt = (now - target_altitude.last_elev_check_us) * 1.0e-6;
        dt = constrain_float(dt, 0.1, 0.15);

        target_altitude.last_elev_check_us = now;

        float elevator_input = channel_pitch->get_control_in() * (1/4500.0);

        if (g.flybywire_elev_reverse) {
            elevator_input = -elevator_input;
        }

        bool input_stop_climb = !is_positive(elevator_input) && is_positive(target_altitude.last_elevator_input);
        bool input_stop_descent = !is_negative(elevator_input) && is_negative(target_altitude.last_elevator_input);
        if (input_stop_climb || input_stop_descent) {
            // user elevator input reached or passed zero, lock in the current altitude
            set_target_altitude_current();
        }

        float climb_rate = g.flybywire_climb_rate * elevator_input;
        climb_rate = constrain_float(climb_rate, -TECS_controller.get_max_sinkrate(), TECS_controller.get_max_climbrate());

        int32_t alt_change_cm = climb_rate * dt * 100;
        change_target_altitude(alt_change_cm);

#if HAL_SOARING_ENABLED
        if (g2.soaring_controller.is_active()) {
            if (g2.soaring_controller.get_throttle_suppressed()) {
                // we're in soaring mode with throttle suppressed
                set_target_altitude_current();
            } else {
                // we're in soaring mode climbing back to altitude. Set target to SOAR_ALT_CUTOFF plus 10m to ensure we positively climb
                // through SOAR_ALT_CUTOFF, thus triggering throttle suppression and return to glide.
                target_altitude.amsl_cm = 100*plane.g2.soaring_controller.get_alt_cutoff() + 1000 + AP::ahrs().get_home().alt;
            }
        }
#endif

        target_altitude.last_elevator_input = elevator_input;
    }

    check_fbwb_altitude();

    calc_throttle();
    calc_nav_pitch();
}

/*
  calculate the turn angle for the next leg of the mission
 */
void Plane::setup_turn_angle(void)
{
    int32_t next_ground_course_cd = mission.get_next_ground_course_cd(-1);
    if (next_ground_course_cd == -1) {
        // the mission library can't determine a turn angle, assume 90 degrees
        auto_state.next_turn_angle = 90.0f;
    } else {
        // get the heading of the current leg
        int32_t ground_course_cd = prev_WP_loc.get_bearing_to(next_WP_loc);

        // work out the angle we need to turn through
        auto_state.next_turn_angle = wrap_180_cd(next_ground_course_cd - ground_course_cd) * 0.01f;
    }
}

/*
  see if we have reached our loiter target
 */
bool Plane::reached_loiter_target(void)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_auto()) {
        return auto_state.wp_distance < 3;
    }
#endif
    return nav_controller->reached_loiter_target();
}
