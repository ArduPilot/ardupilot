#include "Plane.h"

#if SOARING_ENABLED == ENABLED

/*
*  ArduSoar support function
*
*  Peter Braswell, Samuel Tabor, Andrey Kolobov, and Iain Guilliard
*/
void Plane::update_soaring() {
    
    // Check if soaring is active. Also sets throttle suppressed
    // status on active state changes.
    plane.g2.soaring_controller.update_active_state();

    if (!g2.soaring_controller.is_active()) {
        return;
    }
    
    g2.soaring_controller.update_vario();

    // Check for throttle suppression change.
    switch (control_mode->mode_number()) {
    case Mode::Number::AUTO:
        g2.soaring_controller.suppress_throttle();
        break;
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
        if (!g2.soaring_controller.suppress_throttle() && aparm.throttle_max > 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: forcing RTL");
            set_mode(mode_rtl, ModeReason::SOARING_FBW_B_WITH_MOTOR_RUNNING);
        }
        break;
    case Mode::Number::LOITER:
        // Never use throttle in LOITER with soaring active.
        g2.soaring_controller.set_throttle_suppressed(true);
        break;
    default:
        // In any other mode allow throttle.
        g2.soaring_controller.set_throttle_suppressed(false);
        break;
    }

    // Nothing to do if we are in powered flight
    if (!g2.soaring_controller.get_throttle_suppressed() && aparm.throttle_max > 0) {
        return;
    }

    switch (control_mode->mode_number()) {
    default:
        // nothing to do
        break;
    case Mode::Number::AUTO:
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
        // Test for switch into thermalling mode
        g2.soaring_controller.update_cruising();

        if (g2.soaring_controller.check_thermal_criteria()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal detected, entering %s", mode_loiter.name());
            set_mode(mode_loiter, ModeReason::SOARING_THERMAL_DETECTED);
        }
        break;

    case Mode::Number::LOITER: {
        // Update thermal estimate and check for switch back to AUTO
        g2.soaring_controller.update_thermalling();  // Update estimate

        // Thermalling is done in a home-relative coordinate system, so we need home to be set.
        Vector3f position;
        if (!ahrs.get_relative_position_NED_home(position)) {
            return;
        }

        // Check distance to home against MAX_RADIUS.
        if (g2.soaring_controller.max_radius >= 0 &&
            sq(position.x)+sq(position.y) > sq(g2.soaring_controller.max_radius) &&
            previous_mode->mode_number()!=Mode::Number::AUTO) {
            // Some other loiter status, and outside of maximum soaring radius, and previous mode wasn't AUTO
            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Outside SOAR_MAX_RADIUS, RTL");
            set_mode(mode_rtl, ModeReason::SOARING_DRIFT_EXCEEDED);
            break;
        }

        // If previous mode was AUTO and there was a previous NAV command, we can use previous and next wps for drift calculation
        // with respect to the desired direction of travel. If these vectors are zero, drift will be calculated from thermal start
        // position only, without taking account of the desired direction of travel.
        Vector2f prev_wp, next_wp;

        if (previous_mode == &mode_auto) {
            AP_Mission::Mission_Command current_nav_cmd = mission.get_current_nav_cmd();
            AP_Mission::Mission_Command prev_nav_cmd;

            if (!(mission.get_next_nav_cmd(mission.get_prev_nav_cmd_with_wp_index(), prev_nav_cmd) &&
                prev_nav_cmd.content.location.get_vector_xy_from_origin_NE(prev_wp) &&
                current_nav_cmd.content.location.get_vector_xy_from_origin_NE(next_wp))) {
                prev_wp.zero();
                next_wp.zero();
            }
        }

        // Get the status of the soaring controller cruise checks.
        const SoaringController::LoiterStatus loiterStatus = g2.soaring_controller.check_cruise_criteria(prev_wp/100, next_wp/100);

        if (loiterStatus == SoaringController::LoiterStatus::GOOD_TO_KEEP_LOITERING) {
            // Reset loiter angle, so that the loiter exit heading criteria
            // only starts expanding when we're ready to exit.
            plane.loiter.sum_cd = 0;
            plane.soaring_mode_timer_ms = AP_HAL::millis();

            //update the wp location
            g2.soaring_controller.get_target(next_WP_loc);

            break;
        }

        // Some other loiter status, we need to think about exiting loiter.
        const uint32_t time_in_loiter_ms = AP_HAL::millis() - plane.soaring_mode_timer_ms;

        if (!soaring_exit_heading_aligned() && loiterStatus!=SoaringController::LoiterStatus::ALT_TOO_LOW && time_in_loiter_ms < 20000) {
            // Heading not lined up, and not timed out or in a condition requiring immediate exit.
            break;
        }

        // Heading lined up and loiter status not good to continue. Need to switch mode.

        // Determine appropriate mode.
        Mode* exit_mode = previous_mode;

        if (loiterStatus == SoaringController::LoiterStatus::ALT_TOO_LOW && 
           ((previous_mode == &mode_cruise) || (previous_mode == &mode_fbwb))) {
            exit_mode = &mode_rtl;
        }

        // Print message and set mode.
        switch (loiterStatus) {
        case SoaringController::LoiterStatus::ALT_TOO_HIGH:
            soaring_restore_mode("Too high", ModeReason::SOARING_ALT_TOO_HIGH, *exit_mode);
            break;
        case SoaringController::LoiterStatus::ALT_TOO_LOW:
            soaring_restore_mode("Too low", ModeReason::SOARING_ALT_TOO_LOW, *exit_mode);
            break;
        default:
        case SoaringController::LoiterStatus::THERMAL_WEAK:
            soaring_restore_mode("Thermal ended", ModeReason::SOARING_THERMAL_ESTIMATE_DETERIORATED, *exit_mode);
            break;
        case SoaringController::LoiterStatus::DRIFT_EXCEEDED:
            soaring_restore_mode("Drifted too far", ModeReason::SOARING_DRIFT_EXCEEDED, *exit_mode);
            break;
        } // switch loiterStatus

        break;
       
    } // case loiter
    } // switch control_mode
}


bool Plane::soaring_exit_heading_aligned() const
{
    // Return true if the current heading is aligned with the next objective.
    // If home is not set, or heading not locked, return true to avoid delaying mode change.
    switch (previous_mode->mode_number()) {
    case Mode::Number::AUTO: {
        //Get the lat/lon of next Nav waypoint after this one:
        AP_Mission::Mission_Command current_nav_cmd = mission.get_current_nav_cmd();;
        return plane.mode_loiter.isHeadingLinedUp(next_WP_loc, current_nav_cmd.content.location);
    }
    case Mode::Number::FLY_BY_WIRE_B:
        return (!AP::ahrs().home_is_set() || plane.mode_loiter.isHeadingLinedUp(next_WP_loc, AP::ahrs().get_home()));
    case Mode::Number::CRUISE:
        return (!cruise_state.locked_heading || plane.mode_loiter.isHeadingLinedUp_cd(cruise_state.locked_heading_cd));
    default:
        break;
    }
    return true;
}

void Plane::soaring_restore_mode(const char *reason, ModeReason modereason, Mode &exit_mode)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Soaring: %s, restoring %s", reason, exit_mode.name());
    set_mode(exit_mode, modereason);
}

#endif // SOARING_ENABLED
