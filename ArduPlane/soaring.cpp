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

        Vector2f prev_wp, next_wp;

        // If previous mode was AUTO and there was a previous NAV command, we can use previous and next wps for drift calculation.
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

        const SoaringController::LoiterStatus loiterStatus = g2.soaring_controller.check_cruise_criteria(prev_wp/100, next_wp/100);

        if (loiterStatus == SoaringController::LoiterStatus::GOOD_TO_KEEP_LOITERING) {
            // Reset loiter angle, so that the loiter exit heading criteria
            // only starts expanding when we're ready to exit.
            plane.loiter.sum_cd = 0;
            plane.soaring_mode_timer = AP_HAL::millis();
            break;
        }

        // Some other loiter status, we need to think about exiting loiter.
        uint32_t timer = AP_HAL::millis() - plane.soaring_mode_timer;

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

        const char* strTooHigh = "Soaring: Too high, restoring ";
        const char* strTooLow =  "Soaring: Too low, restoring ";
        const char* strTooWeak = "Soaring: Thermal ended, restoring ";
        const char* strTooFar  = "Soaring: Drifted too far, restoring ";

        // Exit as soon as thermal state estimate deteriorates and we're lined up to next target
        switch (previous_mode->mode_number()){
        case Mode::Number::FLY_BY_WIRE_B: {
            const bool homeIsSet = AP::ahrs().home_is_set();
            const bool headingLinedupToHome = homeIsSet && plane.mode_loiter.isHeadingLinedUp(next_WP_loc, AP::ahrs().get_home());
            if (homeIsSet && !headingLinedupToHome && loiterStatus!=SoaringController::LoiterStatus::ALT_TOO_LOW && timer<20e3) {
                break;
            }
            switch (loiterStatus) {
                case SoaringController::LoiterStatus::ALT_TOO_HIGH:
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%s", strTooHigh, mode_fbwb.name());
                    set_mode(mode_fbwb, ModeReason::SOARING_ALT_TOO_HIGH);
                    break;
                case SoaringController::LoiterStatus::ALT_TOO_LOW:
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%s", strTooLow, mode_rtl.name());
                    set_mode(mode_rtl, ModeReason::SOARING_ALT_TOO_LOW);
                    break;
                default:
                case SoaringController::LoiterStatus::THERMAL_WEAK:
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%s", strTooWeak, mode_fbwb.name());
                    set_mode(mode_fbwb, ModeReason::SOARING_THERMAL_ESTIMATE_DETERIORATED);
                    break;
                case SoaringController::LoiterStatus::DRIFT_EXCEEDED:
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%s", strTooFar, mode_fbwb.name());
                    set_mode(mode_fbwb, ModeReason::SOARING_DRIFT_EXCEEDED);
                    break;
                } // switch louterStatus
            break;
            }

        case Mode::Number::CRUISE: {
            const bool headingLinedupToCruise = plane.mode_loiter.isHeadingLinedUp_cd(cruise_state.locked_heading_cd);
            if (cruise_state.locked_heading && !headingLinedupToCruise && loiterStatus!=SoaringController::LoiterStatus::ALT_TOO_LOW && timer<20e3) {
                break;
            }
            // return to cruise with old ground course
            const CruiseState cruise = cruise_state;
            switch (loiterStatus) {
                case SoaringController::LoiterStatus::ALT_TOO_HIGH:
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%s", strTooHigh, mode_cruise.name());
                    set_mode(mode_cruise, ModeReason::SOARING_ALT_TOO_HIGH);
                    break;
                case SoaringController::LoiterStatus::ALT_TOO_LOW:
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%s", strTooLow, mode_rtl.name());
                    set_mode(mode_rtl, ModeReason::SOARING_ALT_TOO_LOW);
                    break;
                default:
                case SoaringController::LoiterStatus::THERMAL_WEAK:
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%s", strTooWeak, mode_cruise.name());
                    set_mode(mode_cruise, ModeReason::SOARING_THERMAL_ESTIMATE_DETERIORATED);
                    break;
                case SoaringController::LoiterStatus::DRIFT_EXCEEDED:
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%s", strTooFar, mode_cruise.name());
                    set_mode(mode_cruise, ModeReason::SOARING_DRIFT_EXCEEDED);
                    break;
            } // switch loiterStatus
            cruise_state = cruise;
            set_target_altitude_current();
            break;
            } // case Cruise

        case Mode::Number::AUTO: {
            //Get the lat/lon of next Nav waypoint after this one:
            AP_Mission::Mission_Command current_nav_cmd = mission.get_current_nav_cmd();;
            const bool headingLinedupToWP = plane.mode_loiter.isHeadingLinedUp(next_WP_loc, current_nav_cmd.content.location);
            if (!headingLinedupToWP && loiterStatus!=SoaringController::LoiterStatus::ALT_TOO_LOW && timer<20e3) {
                break;
            }
            switch (loiterStatus) {
                case SoaringController::LoiterStatus::ALT_TOO_HIGH:
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%s", strTooHigh, mode_auto.name());
                    set_mode(mode_auto, ModeReason::SOARING_ALT_TOO_HIGH);
                    break;
                case SoaringController::LoiterStatus::ALT_TOO_LOW:
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%s", strTooLow, mode_auto.name());
                    set_mode(mode_auto, ModeReason::SOARING_ALT_TOO_LOW);
                    break;
                default:
                case SoaringController::LoiterStatus::THERMAL_WEAK:
                        gcs().send_text(MAV_SEVERITY_INFO, "%s%s", strTooWeak, mode_auto.name());
                        set_mode(mode_auto, ModeReason::SOARING_THERMAL_ESTIMATE_DETERIORATED);
                    break;
                case SoaringController::LoiterStatus::DRIFT_EXCEEDED:
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%s", strTooFar, mode_auto.name());
                    set_mode(mode_auto, ModeReason::SOARING_DRIFT_EXCEEDED);
                    break;
                } // switch loiterStatus
            break;
            } // case AUTO
        default: // all other modes
            break;
        } // switch previous_mode
        break;
        } // case loiter
    } // switch control_mode


    if (control_mode == &mode_loiter) {
        // still in thermal - need to update the wp location
        g2.soaring_controller.get_target(next_WP_loc);

    }

}

#endif // SOARING_ENABLED
