#include "AP_Vehicle.h"

#include <AP_Motors/AP_Motors.h>
#include <GCS_MAVLink/GCS.h>

#if AP_MOTORS_ENABLED && HAL_WITH_ESC_TELEM
void AP_Vehicle::motors_takeoff_check(float rpm_min, float rpm_max, bool land_complete)
{
    auto motors = AP::motors();

    // If takeoff check is disabled or vehicle is armed and flying then clear block and return
    if ((rpm_min <= 0) || (motors->armed() && !land_complete)) {
        motors->set_spoolup_block(false);
        return;
    }

    // block takeoff when disarmed but do not display warnings
    if (!motors->armed()) {
        motors->set_spoolup_block(true);
        takeoff_check_state.warning_ms = 0;
        return;
    }

    // if motors have become unblocked return immediately
    // this ensures the motors can only be blocked immediate after arming
    if (!motors->get_spoolup_block()) {
        return;
    }

    // check ESCs are sending RPM at expected level
    uint32_t motor_mask = motors->get_motor_mask();
    const bool telem_active = AP::esc_telem().is_telemetry_active(motor_mask);
    const bool rpm_adequate = AP::esc_telem().are_motors_running(motor_mask, rpm_min, rpm_max);

    // if RPM is at the expected level clear block
    if (telem_active && rpm_adequate) {
        motors->set_spoolup_block(false);
        return;
    }

    // warn user telem inactive or rpm is inadequate every 5 seconds
    uint32_t now_ms = AP_HAL::millis();
    if (takeoff_check_state.warning_ms == 0) {
        takeoff_check_state.warning_ms = now_ms;
    }
    if (now_ms - takeoff_check_state.warning_ms > 5000) {
        takeoff_check_state.warning_ms = now_ms;
        const char* prefix_str = "Takeoff blocked:";
        if (!telem_active) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "%s waiting for ESC RPM", prefix_str);
        } else if (!rpm_adequate) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "%s ESC RPM out of range", prefix_str);
        }
    }
}

#endif  // AP_MOTORS_ENABLED && HAL_WITH_ESC_TELEM
