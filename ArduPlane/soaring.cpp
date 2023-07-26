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
    bool override_disable = mission.get_in_landing_sequence_flag();

    plane.g2.soaring_controller.update_active_state(override_disable);

    if (!g2.soaring_controller.is_active()) {
        return;
    }
    
    g2.soaring_controller.update_vario();

    if (control_mode == &mode_thermal) {
        // We are currently thermalling; suppress throttle and update
        // the thermal mode.

        // Never use throttle in THERMAL with soaring active.
        g2.soaring_controller.set_throttle_suppressed(true);

        // Update THERMAL mode soaring logic.
        mode_thermal.update_soaring();
        return;
    }

    if (control_mode->does_automatic_thermal_switch()) {
        // We might decide to start thermalling; if we're not under
        // powered flight then check for thermals and potentially
        // switch modes.

        // Check for throttle suppression change.
        if (g2.soaring_controller.suppress_throttle()) {
            // Throttle is suppressed, perform cruising modes update and check for mode switch.

            // Cruising modes update.
            g2.soaring_controller.update_cruising();

            // Test for switch into THERMAL mode
            if (g2.soaring_controller.check_thermal_criteria()) {
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal detected, entering %s", mode_thermal.name());
                set_mode(mode_thermal, ModeReason::SOARING_THERMAL_DETECTED);
            }
        }
        return;
    }

    //  We are not thermalling and won't start to from this mode. Allow throttle.
    g2.soaring_controller.set_throttle_suppressed(false);
}

#endif // SOARING_ENABLED
