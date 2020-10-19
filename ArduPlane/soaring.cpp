#include "Plane.h"

#if HAL_SOARING_ENABLED

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
    case Mode::Number::FLY_BY_WIRE_B:
    case Mode::Number::CRUISE:
        g2.soaring_controller.suppress_throttle();
        break;
    case Mode::Number::THERMAL:
        // Never use throttle in THERMAL with soaring active.
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
            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal detected, entering %s", mode_thermal.name());
            set_mode(mode_thermal, ModeReason::SOARING_THERMAL_DETECTED);
        }
        break;
    case Mode::Number::THERMAL:
        // Update thermal mode soaring logic.
        mode_thermal.update_soaring();
        break;
    } // switch control_mode
}

#endif // SOARING_ENABLED
