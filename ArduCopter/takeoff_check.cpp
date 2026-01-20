#include "Copter.h"

//
// pre-takeoff checks
//

// detects if the vehicle should be allowed to takeoff or not and sets the motors.blocked flag
void Copter::takeoff_check()
{
#if HAL_WITH_ESC_TELEM && FRAME_CONFIG != HELI_FRAME
    // If takeoff check is disabled or vehicle is armed and flying then clear block and return
    if ((g2.takeoff_rpm_min <= 0) || (motors->armed() && !ap.land_complete)) {
        motors->set_spoolup_block(false);
        return;
    }

    // Run the common motor checks (called early so it can clear its warning timer when disarmed)
    const bool motor_check_passed = motors_takeoff_check(g2.takeoff_rpm_min, g2.takeoff_rpm_max);

    // block takeoff when disarmed but do not display warnings
    if (!motors->armed()) {
        motors->set_spoolup_block(true);
        takeoff_check_warning_ms = 0;
        return;
    }

    // if motors have become unblocked return immediately
    // this ensures the motors can only be blocked immediate after arming
    if (!motors->get_spoolup_block()) {
        return;
    }

    // Check system load
    float avg_load, peak_load;
    bool load_adequate = true;
    if (hal.util->get_system_load(avg_load, peak_load)) {
        if (avg_load > 95.0f || peak_load > 99.5f) {
            load_adequate = false;
        }
    }

    // Clear block if all checks passed
    if (motor_check_passed && load_adequate) {
        motors->set_spoolup_block(false);
        return;
    }

    // warn about CPU load every 5 seconds
    uint32_t now_ms = AP_HAL::millis();
    if (takeoff_check_warning_ms == 0) {
        takeoff_check_warning_ms = now_ms;
    }
    if (now_ms - takeoff_check_warning_ms > 5000) {
        takeoff_check_warning_ms = now_ms;
        const char* prefix_str = "Takeoff blocked:";
        if (!load_adequate) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "%s CPU overload (%4.1f%%)", prefix_str, avg_load);
        }
    }
#endif
}
