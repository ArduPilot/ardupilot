#include "mode.h"
#include "Plane.h"

bool ModeRTL::_enter()
{
    plane.prev_WP_loc = plane.current_loc;
    plane.do_RTL(plane.get_RTL_altitude_cm());
    stage = STAGE::RETURNING;
#if HAL_QUADPLANE_ENABLED
    plane.vtol_approach_s.approach_stage = Plane::Landing_ApproachStage::RTL;
#endif

    if (plane.current_loc.alt < plane.next_WP_loc.alt) {
        // options to get up to RTL altitude if lower
        if (plane.g2.rtl_climb_min > 0 || plane.g2.rtl_type == RTL_type::CLIMB_BEFORE_TURN) {
            stage = STAGE::LEVEL_CLIMB;

        } else if (plane.g2.rtl_type == RTL_type::LOITER_TO_ALT) {
            // loiter at current location
            plane.next_WP_loc.lat = plane.current_loc.lat;
            plane.next_WP_loc.lng = plane.current_loc.lng;
            stage = STAGE::LOITER_TO_ALT;
        }
    }

    // do not check if we have reached the loiter target if switching from loiter this will trigger as the nav controller has not yet proceeded the new destination
#if HAL_QUADPLANE_ENABLED
    switch_QRTL(false);
#endif

    if ((plane.g.rtl_autoland == 2) && plane.mission.jump_to_landing_sequence()) {
        // Go directly to the landing sequence
        // switch from RTL -> AUTO
        plane.mission.set_force_resume(true);
        plane.set_mode(plane.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND);
    }

    return true;
}

void ModeRTL::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();

    if (stage == STAGE::LEVEL_CLIMB) {
        // Constrain the roll limit as a failsafe, that way if something goes wrong the plane will
        // eventually turn back and go to RTL instead of going perfectly straight. This also leaves
        // some leeway for fighting wind.
        plane.roll_limit_cd = MIN(plane.roll_limit_cd, plane.g.level_roll_limit*100);
        plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -plane.roll_limit_cd, plane.roll_limit_cd);
    }
}

void ModeRTL::navigate()
{
    const bool recent_mode_change = AP_HAL::millis() - plane.last_mode_change_ms < 1000;
#if HAL_QUADPLANE_ENABLED
    if (plane.control_mode->mode_number() != QRTL) {
        // QRTL shares this navigate function with RTL

        if (plane.quadplane.available() && (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::VTOL_APPROACH_QRTL)) {
            // VTOL approach landing
            AP_Mission::Mission_Command cmd;
            cmd.content.location = plane.next_WP_loc;
            plane.verify_landing_vtol_approach(cmd);
            if (plane.vtol_approach_s.approach_stage == Plane::Landing_ApproachStage::VTOL_LANDING) {
                plane.set_mode(plane.mode_qrtl, ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL);
            }
            return;
        }

        if (!recent_mode_change && switch_QRTL()) {
            return;
        }
    }
#endif

    if (!recent_mode_change &&
            stage == STAGE::RETURNING &&
            plane.reached_loiter_target() &&
            labs(plane.altitude_error_cm) < 1000) {
        stage = STAGE::REACHED_HOME;

        if (plane.g.rtl_autoland == 1 && plane.mission.jump_to_landing_sequence()) {
            // we've reached the RTL point, see if we have a landing sequence
            // switch from RTL -> AUTO
            plane.mission.set_force_resume(true);
            plane.set_mode(plane.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND);

        } else if (plane.g.RTL_altitude_cm > 0 && (plane.g2.flight_options & FlightOptions::RTL_NO_DESCENT)) {
            // update target altitude
            plane.do_RTL(plane.g.RTL_altitude_cm + plane.home.alt);
        }
    }

    if ((stage != STAGE::RETURNING) && (stage != STAGE::REACHED_HOME) && (plane.current_loc.alt >= plane.next_WP_loc.alt)) {
        // always return once reached RTL altitude
        stage = STAGE::RETURNING;
        plane.do_RTL(plane.get_RTL_altitude_cm());
    }

    if ((stage == STAGE::LEVEL_CLIMB) &&
            (plane.g2.rtl_type != RTL_type::CLIMB_BEFORE_TURN) &&
            (plane.current_loc.alt > (plane.prev_WP_loc.alt + plane.g2.rtl_climb_min * 100))) {
        // climb min is done
        plane.do_RTL(plane.get_RTL_altitude_cm());
        if (plane.g2.rtl_type == RTL_type::LOITER_TO_ALT) {
            // loiter at current location
            plane.next_WP_loc.lat = plane.current_loc.lat;
            plane.next_WP_loc.lng = plane.current_loc.lng;
            stage = STAGE::LOITER_TO_ALT;
        } else {
            stage = STAGE::RETURNING;
        }
    }

    uint16_t radius = abs(plane.g.rtl_radius);
    if (radius > 0) {
        plane.loiter.direction = (plane.g.rtl_radius < 0) ? -1 : 1;
    }

    plane.update_loiter(radius);
}

#if HAL_QUADPLANE_ENABLED
// Switch to QRTL if enabled and within radius
bool ModeRTL::switch_QRTL(bool check_loiter_target)
{
    if (!plane.quadplane.available() || ((plane.quadplane.rtl_mode != QuadPlane::RTL_MODE::SWITCH_QRTL) && (plane.quadplane.rtl_mode != QuadPlane::RTL_MODE::QRTL_ALWAYS))) {  
        return false;
    }

   // if Q_RTL_MODE is QRTL always, then immediately switch to QRTL mode
   if (plane.quadplane.rtl_mode == QuadPlane::RTL_MODE::QRTL_ALWAYS) {
       plane.set_mode(plane.mode_qrtl, ModeReason::QRTL_INSTEAD_OF_RTL);
       return true;
   }

    uint16_t qrtl_radius = abs(plane.g.rtl_radius);
    if (qrtl_radius == 0) {
        qrtl_radius = abs(plane.aparm.loiter_radius);
    }

    if ( (check_loiter_target && plane.nav_controller->reached_loiter_target()) ||
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

#endif  // HAL_QUADPLANE_ENABLED
