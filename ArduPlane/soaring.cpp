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
        if (!g2.soaring_controller.suppress_throttle() && aparm.throttle_max > 0 && g2.soaring_controller.is_suppress_throttle_mode()) {
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
    if (!g2.soaring_controller.get_throttle_suppressed() && aparm.throttle_max > 0 && g2.soaring_controller.is_suppress_throttle_mode()) {
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

                // Save altitude targets to restore later.
                plane.soaring_restore_target_alt_amsl_cm    = plane.target_altitude.amsl_cm;
                plane.soaring_restore_target_alt_terrain_cm = plane.target_altitude.terrain_alt_cm;

                set_mode(mode_loiter, ModeReason::SOARING_THERMAL_DETECTED);
            }
            break;

        case Mode::Number::LOITER: {
            // Update thermal estimate and check for switch back to AUTO
            g2.soaring_controller.update_thermalling();  // Update estimate

            // Check distance to home.
            Vector3f position;
            if (!ahrs.get_relative_position_NED_home(position)) {
                return;
            } else if (g2.soaring_controller.max_radius >= 0 &&
                       powf(position.x,2)+powf(position.y,2) > powf(g2.soaring_controller.max_radius,2) &&
                    previous_mode->mode_number()!=Mode::Number::AUTO) {
                // Some other loiter status, and outside of maximum soaring radius, and previous mode wasn't AUTO
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Outside SOAR_MAX_RADIUS, RTL");
                set_mode(mode_rtl, ModeReason::SOARING_DRIFT_EXCEEDED);
            }

            // If previous mode was AUTO and there was a previous NAV command, we can use previous and next wps for drift calculation.
            Vector2f prev_wp, next_wp;

            if (previous_mode->mode_number() == Mode::Number::AUTO) {
                AP_Mission::Mission_Command current_nav_cmd = mission.get_current_nav_cmd();
                AP_Mission::Mission_Command prev_nav_cmd;

                if (!(mission.get_next_nav_cmd(mission.get_prev_nav_cmd_with_wp_index(), prev_nav_cmd) &&
                       prev_nav_cmd.content.location.get_vector_xy_from_origin_NE(prev_wp) &&
                    current_nav_cmd.content.location.get_vector_xy_from_origin_NE(next_wp))) {
                    prev_wp.x = 0.0;
                    prev_wp.y = 0.0;
                    next_wp.x = 0.0;
                    next_wp.y = 0.0;
                }
            }

            // Get the status of the soaring controller cruise checks.
            const SoaringController::LoiterStatus loiterStatus = g2.soaring_controller.check_cruise_criteria(prev_wp/100, next_wp/100);

            if (loiterStatus == SoaringController::LoiterStatus::GOOD_TO_KEEP_LOITERING) {
                // Reset loiter angle, so that the loiter exit heading criteria
                // only starts expanding when we're ready to exit.
                plane.loiter.sum_cd = 0;
                plane.soaring_mode_timer = AP_HAL::millis();

                //update the wp location
                g2.soaring_controller.get_target(next_WP_loc);

                break;
            }

            // Some other loiter status, we need to think about exiting loiter.
            uint32_t timer = AP_HAL::millis() - plane.soaring_mode_timer;

            if (!soaring_exit_heading_aligned() && loiterStatus!=SoaringController::LoiterStatus::ALT_TOO_LOW && timer<20e3) {
                // Heading not lined up, and not timed out or in a condition requiring immediate exit.
                break;
            }

            // Heading lined up and loiter status not good to continue. Need to switch mode.

            // Determine appropriate mode.
            Mode* exit_mode = previous_mode;

            if (loiterStatus == SoaringController::LoiterStatus::ALT_TOO_LOW && 
               (g2.soaring_controller.is_suppress_throttle_mode() && ((previous_mode->mode_number() == Mode::Number::CRUISE) || (previous_mode->mode_number() == Mode::Number::FLY_BY_WIRE_B)))) {
                exit_mode = &mode_rtl;
            }

            // Print message and set mode.
            switch (loiterStatus) {
                case SoaringController::LoiterStatus::ALT_TOO_HIGH:
                    gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Too high, restoring %s", exit_mode->name());
                    set_mode(*exit_mode, ModeReason::SOARING_ALT_TOO_HIGH);
                    break;
                case SoaringController::LoiterStatus::ALT_TOO_LOW:
                    gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Too low, restoring %s", exit_mode->name());
                    set_mode(*exit_mode, ModeReason::SOARING_ALT_TOO_LOW);
                    break;
                default:
                case SoaringController::LoiterStatus::THERMAL_WEAK:
                    gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal ended, restoring %s", exit_mode->name());
                    set_mode(*exit_mode, ModeReason::SOARING_THERMAL_ESTIMATE_DETERIORATED);
                    break;
                case SoaringController::LoiterStatus::DRIFT_EXCEEDED:
                    gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Drifted too far, restoring %s", exit_mode->name());
                    set_mode(*exit_mode, ModeReason::SOARING_DRIFT_EXCEEDED);
                    break;
            } // switch loiterStatus

            // Restore target altitude.
            plane.target_altitude.amsl_cm        = plane.soaring_restore_target_alt_amsl_cm;
            plane.target_altitude.terrain_alt_cm = plane.soaring_restore_target_alt_terrain_cm;

            // If operating in mode 2, unsuppress the throttle now rather than waiting for next loop, to avoid target altitude reset.
            if (!g2.soaring_controller.is_suppress_throttle_mode()) {
                g2.soaring_controller.set_throttle_suppressed(false);
            }
            break;
           
        } // case loiter
    } // switch control_mode
}


bool Plane::soaring_exit_heading_aligned()
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

#endif // SOARING_ENABLED