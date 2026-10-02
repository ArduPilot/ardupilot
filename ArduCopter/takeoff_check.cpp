#include "Copter.h"

//
// pre-takeoff checks
//

// detects if the vehicle should be allowed to takeoff or not and sets the motors.blocked flag
void Copter::takeoff_check()
{
#if HAL_WITH_ESC_TELEM && FRAME_CONFIG != HELI_FRAME
    uint32_t now_ms = AP_HAL::millis();

#if AP_CPU_IDLE_STATS_ENABLED
    // window length for the takeoff CPU overload check
    static constexpr uint8_t TAKEOFF_CHECK_CPU_WINDOW_S = 2;

    // the load is measured by the idle threads, which only run the measurement
    // when BRD_IDLE_STATS is enabled, so the check is inactive without it
    const bool cpu_check_enabled = AP_BoardConfig::use_idle_stats() &&
                                   (is_positive(g2.takeoff_cpu_avg_max) || is_positive(g2.takeoff_cpu_peak_max));

    // consume the CPU load window on our own cadence whenever the check is
    // enabled, so a completed window is available when the check runs
    if (cpu_check_enabled && now_ms - takeoff_load_window_ms >= TAKEOFF_CHECK_CPU_WINDOW_S * 1000U) {
        takeoff_load_valid = hal.util->consume_system_load(AP_HAL::Util::LoadReader::TAKEOFF, takeoff_load_avg, takeoff_load_peak);
        takeoff_load_window_ms = now_ms;
    }
#endif

    // if motors have become unblocked return immediately
    // this ensures the motors can only be blocked immediately after arming
    if (!motors->get_spoolup_block()) {
        takeoff_check_warning_ms = now_ms;
        takeoff_check_state.warning_ms = now_ms;
        return;
    }

    // Motors Library has enabled the spool up block.

    // Immediately clear the spool up block if not landed
    if (!ap.land_complete) {
        motors->set_spoolup_block(false);
        return;
    }

    // Run the common motor checks (called early so it can clear its warning timer when disarmed)
    const bool motor_check_passed = motors_takeoff_check(g2.takeoff_rpm_min, g2.takeoff_rpm_max);

    // Check system load against the enabled thresholds. Fail closed when
    // enabled without valid window data: an invalid window also means a CPU
    // so overloaded that the idle threads never ran, which must not pass.
    bool load_adequate = true;
#if AP_CPU_IDLE_STATS_ENABLED
    if (cpu_check_enabled) {
        if (!takeoff_load_valid) {
            load_adequate = false;
        } else if ((is_positive(g2.takeoff_cpu_avg_max) && takeoff_load_avg > g2.takeoff_cpu_avg_max) ||
                   (is_positive(g2.takeoff_cpu_peak_max) && takeoff_load_peak > g2.takeoff_cpu_peak_max)) {
            load_adequate = false;
        }
    }
#endif

    // Clear block if all checks passed
    if (motor_check_passed && load_adequate) {
        motors->set_spoolup_block(false);
        return;
    }

    // warn about CPU load every 2 seconds
    if (now_ms - takeoff_check_warning_ms > 2000) {
        takeoff_check_warning_ms = now_ms;
#if AP_CPU_IDLE_STATS_ENABLED
        const char* prefix_str = "Takeoff blocked:";
        if (!load_adequate) {
            if (takeoff_load_valid) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "%s CPU overload (%.1f/%.1f%%)", prefix_str, takeoff_load_avg, takeoff_load_peak);
            } else {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "%s CPU load unavailable", prefix_str);
            }
        }
#endif
    }
#endif
}
