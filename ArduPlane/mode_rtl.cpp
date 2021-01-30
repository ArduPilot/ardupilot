#include "mode.h"
#include "Plane.h"

bool ModeRTL::_enter()
{
    plane.prev_WP_loc = plane.current_loc;
    plane.do_RTL(plane.get_RTL_altitude());
    plane.rtl.done_climb = false;

    switch_QRTL();

    return true;
}

void ModeRTL::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();

    if (plane.g2.rtl_climb_min > 0) {
        /*
          when RTL first starts limit bank angle to LEVEL_ROLL_LIMIT
          until we have climbed by RTL_CLIMB_MIN meters
         */
        if (!plane.rtl.done_climb && (plane.current_loc.alt - plane.prev_WP_loc.alt)*0.01 > plane.g2.rtl_climb_min) {
            plane.rtl.done_climb = true;
        }
        if (!plane.rtl.done_climb) {
            plane.roll_limit_cd = MIN(plane.roll_limit_cd, plane.g.level_roll_limit*100);
            plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -plane.roll_limit_cd, plane.roll_limit_cd);
        }
    }
}

void ModeRTL::navigate()
{

    if ((AP_HAL::millis() - plane.last_mode_change_ms > 1000) && switch_QRTL()) {
        return;
    }

    if (plane.g.rtl_autoland == 1 &&
        !plane.auto_state.checked_for_autoland &&
        plane.reached_loiter_target() && 
        labs(plane.altitude_error_cm) < 1000) {
        // we've reached the RTL point, see if we have a landing sequence
        if (plane.mission.jump_to_landing_sequence()) {
            // switch from RTL -> AUTO
            plane.mission.set_force_resume(true);
            plane.set_mode(plane.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND);
        }

        // prevent running the expensive jump_to_landing_sequence
        // on every loop
        plane.auto_state.checked_for_autoland = true;
    }
    else if (plane.g.rtl_autoland == 2 &&
        !plane.auto_state.checked_for_autoland) {
        // Go directly to the landing sequence
        if (plane.mission.jump_to_landing_sequence()) {
            // switch from RTL -> AUTO
            plane.mission.set_force_resume(true);
            plane.set_mode(plane.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND);
        }

        // prevent running the expensive jump_to_landing_sequence
        // on every loop
        plane.auto_state.checked_for_autoland = true;
    }
    uint16_t radius = abs(plane.g.rtl_radius);
    if (radius > 0) {
        plane.loiter.direction = (plane.g.rtl_radius < 0) ? -1 : 1;
    }

    plane.update_loiter(radius);
}


// Switch to QRTL if enabled and within radius
bool ModeRTL::switch_QRTL()
{
    if (!plane.quadplane.available() || (plane.quadplane.rtl_mode != 1)) {
        return false;
    }

    uint16_t qrtl_radius = abs(plane.g.rtl_radius);
    if (qrtl_radius == 0) {
        qrtl_radius = abs(plane.aparm.loiter_radius);
    }

    if (plane.nav_controller->reached_loiter_target() ||
         plane.current_loc.past_interval_finish_line(plane.prev_WP_loc, plane.next_WP_loc) ||
         plane.auto_state.wp_distance < MAX(qrtl_radius, plane.quadplane.stopping_distance())) {
        /*
          for a quadplane in RTL mode we switch to QRTL when we
          are within the maximum of the stopping distance and the
          RTL_RADIUS
         */
        plane.set_mode(plane.mode_qrtl, ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL);
        return true;
    }

    return false;
}
