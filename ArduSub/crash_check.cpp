#include "Sub.h"

// Code to detect a crash
#define CRASH_CHECK_TRIGGER_SEC         2       // 2 seconds inverted indicates a crash
#define CRASH_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees beyond angle max is signal we are inverted
#define CRASH_CHECK_ACCEL_MAX           3.0f    // vehicle must be accelerating less than 3m/s/s to be considered crashed

// crash_check - disarms motors if a crash has been detected
// crashes are detected by the vehicle being more than 20 degrees beyond it's angle limits continuously for more than 1 second
// called at MAIN_LOOP_RATE
void Sub::crash_check()
{
    static uint16_t crash_counter;  // number of iterations vehicle may have been crashed

    // return immediately if disarmed, or crash checking disabled
    if (!motors.armed() || g.fs_crash_check == 0) {
        crash_counter = 0;
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode
    if (control_mode == ACRO) {
        crash_counter = 0;
        return;
    }

    // vehicle not crashed if 1hz filtered acceleration is more than 3m/s (1G on Z-axis has been subtracted)
    if (land_accel_ef_filter.get().length() >= CRASH_CHECK_ACCEL_MAX) {
        crash_counter = 0;
        return;
    }

    // check for angle error over 30 degrees
    const float angle_error = attitude_control.get_att_error_angle_deg();
    if (angle_error <= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        crash_counter = 0;
        return;
    }

    // we may be crashing
    crash_counter++;

    // check if crashing for 2 seconds
    if (crash_counter >= (CRASH_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_CRASH_CHECK, ERROR_CODE_CRASH_CHECK_CRASH);
        // send message to gcs
        gcs_send_text(MAV_SEVERITY_EMERGENCY,"Crash: Disarming");
        // disarm motors
        init_disarm_motors();
    }
}
