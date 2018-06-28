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
    switch (control_mode){
    case AUTO:
        g2.soaring_controller.suppress_throttle();
        break;
    case FLY_BY_WIRE_B:
    case CRUISE:
        if (!g2.soaring_controller.suppress_throttle()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: forcing RTL");
            set_mode(mode_rtl, MODE_REASON_SOARING_FBW_B_WITH_MOTOR_RUNNING);
        }
        break;
    case LOITER:
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

    switch (control_mode){
    case AUTO:
    case FLY_BY_WIRE_B:
    case CRUISE:
        // Test for switch into thermalling mode
        g2.soaring_controller.update_cruising();

        if (g2.soaring_controller.check_thermal_criteria()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal detected, entering loiter");
            set_mode(mode_loiter, MODE_REASON_SOARING_THERMAL_DETECTED);
        }
        break;

    case LOITER:
        // Update thermal estimate and check for switch back to AUTO
        g2.soaring_controller.update_thermalling();  // Update estimate

        if (g2.soaring_controller.check_cruise_criteria()) {
            // Exit as soon as thermal state estimate deteriorates
            switch (previous_mode) {
            case FLY_BY_WIRE_B:
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal ended, entering RTL");
                set_mode(mode_rtl, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
                break;

            case CRUISE: {
                // return to cruise with old ground course
                CruiseState cruise = cruise_state;
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal ended, restoring CRUISE");
                set_mode(mode_cruise, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
                cruise_state = cruise;
                set_target_altitude_current();
                break;
            }

            case AUTO:
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal ended, restoring AUTO");
                set_mode(mode_auto, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
                break;

            default:
                break;
            }
        } else {
            // still in thermal - need to update the wp location
            g2.soaring_controller.get_target(next_WP_loc);
        }
        break;
    default:
        // nothing to do
        break;
    }
}

#endif // SOARING_ENABLED
