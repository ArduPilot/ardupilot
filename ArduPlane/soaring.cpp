#include "Plane.h"

#if SOARING_ENABLED == ENABLED

/*
*  ArduSoar support function
*
*  Peter Braswell, Samuel Tabor, Andrey Kolobov, and Iain Guilliard
*/
void Plane::update_soaring() {
    
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
        if (!g2.soaring_controller.suppress_throttle()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: forcing RTL");
            set_mode(mode_rtl, ModeReason::SOARING_FBW_B_WITH_MOTOR_RUNNING);
        }
        break;
    case Mode::Number::LOITER:
        // Do nothing. We will switch back to auto/rtl before enabling throttle.
        break;
    default:
        // This does not affect the throttle since suppressed is only checked in the above three modes. 
        // It ensures that the soaring always starts with throttle suppressed though.
        g2.soaring_controller.set_throttle_suppressed(true);
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

        const SoaringController::LoiterStatus loiterStatus = g2.soaring_controller.check_cruise_criteria();

        if (loiterStatus == SoaringController::LoiterStatus::THERMAL_GOOD_TO_KEEP_LOITERING ||
            loiterStatus == SoaringController::LoiterStatus::SOARING_DISABLED) {
            break;
        }

        const char* strTooHigh = "Soaring: Too high, restoring ";
        const char* strTooLow =  "Soaring: Too low, restoring ";
        const char* strTooWeak = "Soaring: Thermal ended, restoring ";

        // Exit as soon as thermal state estimate deteriorates and we're lined up to next target
        switch (previous_mode->mode_number()){
        case Mode::Number::FLY_BY_WIRE_B: {
            const bool homeIsSet = AP::ahrs().home_is_set();
            const bool headingLinedupToHome = homeIsSet && plane.mode_loiter.isHeadingLinedUp(next_WP_loc, AP::ahrs().get_home());
            if (homeIsSet && !headingLinedupToHome) {
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
                } // switch louterStatus
            break;
            }

        case Mode::Number::CRUISE: {
            const bool headingLinedupToCruise = plane.mode_loiter.isHeadingLinedUp(cruise_state.locked_heading_cd);
            if (cruise_state.locked_heading && !headingLinedupToCruise) {
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
            } // switch loiterStatus
            cruise_state = cruise;
            set_target_altitude_current();
            break;
            } // case Cruise

        case Mode::Number::AUTO: {
            //Get the lat/lon of next Nav waypoint after this one:
            AP_Mission::Mission_Command next_nav_cmd;
            const bool nextWpisValid = mission.get_next_nav_cmd(mission.get_current_nav_index() + 1, next_nav_cmd);
            const bool headingLinedupToWP = nextWpisValid && plane.mode_loiter.isHeadingLinedUp(next_WP_loc, next_nav_cmd.content.location);
            if (nextWpisValid && !headingLinedupToWP) {
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
