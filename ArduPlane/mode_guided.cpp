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
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
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
