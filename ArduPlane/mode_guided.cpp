#include "mode.h"
#include "Plane.h"

Mode::Number ModeGuided::mode_number() const
{
#if AP_SCRIPTING_ENABLED
    // Shamelessly lie about being in guided to maintain pre-existing behaviour
    // In the future we should be able to report as a custom mode
    if (submode == SubMode::NAV_Script_time) {
        return plane.previous_mode->mode_number();
    }
#endif
    return Mode::Number::GUIDED;
}

const char *ModeGuided::name() const 
{
#if AP_SCRIPTING_ENABLED
    // Shamelessly lie about being in guided to maintain pre-existing behaviour
    // In the future we should be able to report as a custom mode
    if (submode == SubMode::NAV_Script_time) {
        return plane.previous_mode->name();
    }
#endif
    return "GUIDED";
}

const char *ModeGuided::name4() const
{
#if AP_SCRIPTING_ENABLED
    // Shamelessly lie about being in guided to maintain pre-existing behaviour
    // In the future we should be able to report as a custom mode
    if (submode == SubMode::NAV_Script_time) {
        return plane.previous_mode->name4();
    }
#endif
    return "GUID";
}

bool ModeGuided::_enter()
{
    // Return to default submode
    submode = SubMode::Guided;

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
        loc.offset_bearing(degrees(plane.ahrs.groundspeed_vector().angle()),
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
    switch (submode) {
        case SubMode::Guided: {
#if HAL_QUADPLANE_ENABLED
            if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
                plane.quadplane.guided_update();
                return;
            }
#endif
            plane.calc_nav_roll();
            plane.calc_nav_pitch();
            plane.calc_throttle();
            break;
        }

#if AP_SCRIPTING_ENABLED
        case SubMode::NAV_Script_time: {
            // Reset desired angles
            plane.nav_roll_cd = plane.ahrs.roll_sensor;
            plane.nav_pitch_cd = plane.ahrs.pitch_sensor;

            // Check for timeout
            plane.nav_scripting_check_timeout();

            // Reset to previous mode if command done
            if (!plane.nav_scripting_active()) {
                plane.set_mode(*plane.previous_mode, ModeReason::NAV_SCRIPTING_COMPLETE);
                // Should probably check for failure here, or add ability to force mode change
            }
            break;
        }
#endif
    }
}

void ModeGuided::navigate()
{
    switch (submode) {
        case SubMode::Guided:
            plane.update_loiter(active_radius_m);
            break;

#if AP_SCRIPTING_ENABLED
        case SubMode::NAV_Script_time:
            break;
#endif
    }
}

bool ModeGuided::handle_guided_request(Location target_loc)
{
#if AP_SCRIPTING_ENABLED
    if (submode == SubMode::NAV_Script_time) {
        return false;
    }
#endif

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
#if AP_SCRIPTING_ENABLED
    const bool in_script_time = submode == SubMode::NAV_Script_time;
#else
    const bool in_script_time = false;
#endif
    if (!in_script_time && ((plane.guided_state.target_alt_time_ms != 0) || plane.guided_state.target_alt > -0.001 )) { // target_alt now defaults to -1, and _time_ms defaults to zero.
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

void ModeGuided::set_submode(SubMode mode)
{
    submode = mode;
}
