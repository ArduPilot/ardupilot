#include "Plane.h"

/*
*  ArduSoar support function
*
*  Peter Braswell, Samuel Tabor, Andrey Kolobov, and Iain Guilliard
*/
void Plane::update_soaring() {
    
    if (!soaring_controller.is_active()) {
        return;
    }
    
    soaring_controller.update_vario();

    // Check for throttle suppression change.
    switch (control_mode){
    case AUTO:
        soaring_controller.suppress_throttle();
        break;
    case FLY_BY_WIRE_B:
        if (!soaring_controller.suppress_throttle()) {
            set_mode(RTL, MODE_REASON_SOARING_FBW_B_WITH_MOTOR_RUNNING);
        }
        break;
    case LOITER:
        // Do nothing. We will switch back to auto/rtl before enabling throttle.
        break;
    default:
        // This does not affect the throttle since suppressed is only checked in the above three modes. 
        // It ensures that the soaring always starts with throttle suppressed though.
        soaring_controller.set_throttle_suppressed(true);
        break;
    }
    // Nothing to do if we are in powered flight
    if (!soaring_controller.get_throttle_suppressed() && aparm.throttle_max > 0) {
        return;
    }

    switch (control_mode){
    case AUTO:
    case FLY_BY_WIRE_B:
        // Test for switch into thermalling mode
        soaring_controller.update_cruising();

        if (soaring_controller.check_thermal_criteria()) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Thermal detected, entering loiter\n");
            set_mode(LOITER, MODE_REASON_SOARING_THERMAL_DETECTED);
        }
        break;
    case LOITER:
        // Update thermal estimate and check for switch back to AUTO
        soaring_controller.update_thermalling();  // Update estimate

        if (soaring_controller.check_cruise_criteria()) {
            // Exit as soon as thermal state estimate deteriorates
            if (previous_mode == FLY_BY_WIRE_B) {
                set_mode(RTL, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
            } else {
                set_mode(previous_mode, MODE_REASON_SOARING_THERMAL_ESTIMATE_DETERIORATED);
            }
        } else {
            // still in thermal - need to update the wp location
            soaring_controller.get_target(next_WP_loc);
        }
        break;
    default:
        // nothing to do
        break;
    }
}

