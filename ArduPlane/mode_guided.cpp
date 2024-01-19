#include "mode.h"
#include "Plane.h"

bool ModeGuided::_enter()
{
    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    Location loc{plane.current_loc};

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.guided_mode_enabled()) {
        /*
          if using Q_GUIDED_MODE then project forward by the stopping distance
        */
        loc.offset_bearing(degrees(ahrs.groundspeed_vector().angle()),
                           plane.quadplane.stopping_distance());
    }
#endif

    // set guided radius to WP_LOITER_RAD on mode change.
    active_radius_m = 0;

    plane.set_guided_WP(loc);
    return true;
}

void ModeGuided::update()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        return;
    }
#endif

    // Received an external msg that guides roll in the last 3 seconds?
    if (plane.guided_state.last_forced_rpy_ms.x > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.x < 3000) {
        plane.nav_roll_cd = constrain_int32(plane.guided_state.forced_rpy_cd.x, -plane.roll_limit_cd, plane.roll_limit_cd);
        plane.update_load_factor();

#if OFFBOARD_GUIDED == ENABLED
    // guided_state.target_heading is radians at this point between -pi and pi ( defaults to -4 )
    // This function is used in Guided and AvoidADSB, check for guided
    } else if ((plane.control_mode == &plane.mode_guided) && (plane.guided_state.target_heading_type != GUIDED_HEADING_NONE) ) {
        uint32_t tnow = AP_HAL::millis();
        float delta = (tnow - plane.guided_state.target_heading_time_ms) * 1e-3f;
        plane.guided_state.target_heading_time_ms = tnow;

        float error = 0.0f;
        if (plane.guided_state.target_heading_type == GUIDED_HEADING_HEADING) {
            error = wrap_PI(plane.guided_state.target_heading - AP::ahrs().get_yaw());
        } else {
            Vector2f groundspeed = AP::ahrs().groundspeed_vector();
            error = wrap_PI(plane.guided_state.target_heading - atan2f(-groundspeed.y, -groundspeed.x) + M_PI);
        }

        float bank_limit = degrees(atanf(plane.guided_state.target_heading_accel_limit/GRAVITY_MSS)) * 1e2f;
        bank_limit = MIN(bank_limit, plane.roll_limit_cd);

        // push error into AC_PID
        const float desired = plane.g2.guidedHeading.update_error(error, delta, plane.guided_state.target_heading_limit);

        // Check for output saturation
        plane.guided_state.target_heading_limit = fabsf(desired) >= bank_limit;

        plane.nav_roll_cd = constrain_int32(desired, -bank_limit, bank_limit);
        plane.update_load_factor();

#endif // OFFBOARD_GUIDED == ENABLED
    } else {
        plane.calc_nav_roll();
    }

    if (plane.guided_state.last_forced_rpy_ms.y > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.y < 3000) {
        plane.nav_pitch_cd = constrain_int32(plane.guided_state.forced_rpy_cd.y, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    } else {
        plane.calc_nav_pitch();
    }

    // Received an external msg that guides throttle in the last 3 seconds?
    if (plane.aparm.throttle_cruise > 1 &&
            plane.guided_state.last_forced_throttle_ms > 0 &&
            millis() - plane.guided_state.last_forced_throttle_ms < 3000) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.guided_state.forced_throttle);
    } else {
        plane.calc_throttle();
    }

}

void ModeGuided::navigate()
{
    plane.update_loiter(active_radius_m);
}

bool ModeGuided::handle_guided_request(Location target_loc)
{
    // add home alt if needed
    if (target_loc.relative_alt) {
        target_loc.alt += plane.home.alt;
        target_loc.relative_alt = 0;
    }

    plane.set_guided_WP(target_loc);

    return true;
}

void ModeGuided::set_radius_and_direction(const float radius, const bool direction_is_ccw)
{
    // constrain to (uint16_t) range for update_loiter()
    active_radius_m = constrain_int32(fabsf(radius), 0, UINT16_MAX);
    plane.loiter.direction = direction_is_ccw ? -1 : 1;
}

void ModeGuided::update_target_altitude()
{
#if OFFBOARD_GUIDED == ENABLED
    if (((plane.guided_state.target_alt_time_ms != 0) || plane.guided_state.target_alt > -0.001 )) { // target_alt now defaults to -1, and _time_ms defaults to zero.
        // offboard altitude demanded
        uint32_t now = AP_HAL::millis();
        float delta = 1e-3f * (now - plane.guided_state.target_alt_time_ms);
        plane.guided_state.target_alt_time_ms = now;
        // determine delta accurately as a float
        float delta_amt_f = delta * plane.guided_state.target_alt_accel;
        // then scale x100 to match last_target_alt and convert to a signed int32_t as it may be negative
        int32_t delta_amt_i = (int32_t)(100.0 * delta_amt_f); 
        Location temp {};
        temp.alt = plane.guided_state.last_target_alt + delta_amt_i; // ...to avoid floats here, 
        if (is_positive(plane.guided_state.target_alt_accel)) {
            temp.alt = MIN(plane.guided_state.target_alt, temp.alt);
        } else {
            temp.alt = MAX(plane.guided_state.target_alt, temp.alt);
        }
        plane.guided_state.last_target_alt = temp.alt;
        plane.set_target_altitude_location(temp);
        plane.altitude_error_cm = plane.calc_altitude_error_cm();
    } else 
#endif // OFFBOARD_GUIDED == ENABLED
        {
        Mode::update_target_altitude();
    }
}
