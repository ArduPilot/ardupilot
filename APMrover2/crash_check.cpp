#include "Rover.h"

// Code to detect a crash or block
static const uint16_t CRASH_CHECK_TRIGGER_SEC = 2;   // 2 seconds blocked indicates a crash
static const float CRASH_CHECK_THROTTLE_MIN = 5.0f;  // vehicle must have a throttle greater that 5% to be considered crashed
static const float CRASH_CHECK_VEL_MIN = 0.08f;      // vehicle must have a velocity under 0.08 m/s or rad/s to be considered crashed

// crash_check - disarms motors if a crash or block has been detected
// crashes are detected by the vehicle being static (no speed) for more than CRASH_CHECK_TRIGGER_SEC and motor are running
// called at 10Hz
void Rover::crash_check()
{
    static uint16_t crash_counter;  // number of iterations vehicle may have been crashed
    bool crashed = false; //stores crash state

    // return immediately if disarmed, crash checking is disabled or vehicle is Hold, Manual or Acro mode(if it is not a balance bot)
    if (!arming.is_armed() || g.fs_crash_check == FS_CRASH_DISABLE || ((!control_mode->is_autopilot_mode()) && (!is_balancebot()))) {
        crash_counter = 0;
        return;
    }

    // Crashed if pitch/roll > crash_angle
    if ((g2.crash_angle != 0) && ((fabsf(ahrs.pitch) > radians(g2.crash_angle)) || (fabsf(ahrs.roll) > radians(g2.crash_angle)))) {
        crashed = true;
    }

    // TODO : Check if min vel can be calculated
    // min_vel = ( CRASH_CHECK_THROTTLE_MIN * g.speed_cruise) / g.throttle_cruise;

    if (!is_balancebot()) {
        if (!crashed && ((ahrs.groundspeed() >= CRASH_CHECK_VEL_MIN) ||        // Check velocity
            (fabsf(ahrs.get_gyro().z) >= CRASH_CHECK_VEL_MIN) ||  // Check turn speed
            (fabsf(g2.motors.get_throttle()) < CRASH_CHECK_THROTTLE_MIN))) {
            crash_counter = 0;
            return;
        }

        // we may be crashing
        crash_counter++;

        // check if crashing for 2 seconds
        if (crash_counter >= (CRASH_CHECK_TRIGGER_SEC * 10)) {
            crashed = true;
        }
    }

    if (crashed) {
        AP::logger().Write_Error(LogErrorSubsystem::CRASH_CHECK,
                                 LogErrorCode::CRASH_CHECK_CRASH);

        if (is_balancebot()) {
            // send message to gcs
            gcs().send_text(MAV_SEVERITY_EMERGENCY, "Crash: Disarming");
            arming.disarm();
        } else {
            // send message to gcs
            gcs().send_text(MAV_SEVERITY_EMERGENCY, "Crash: Going to HOLD");
            // change mode to hold and disarm
            set_mode(mode_hold, MODE_REASON_CRASH_FAILSAFE);
            if (g.fs_crash_check == FS_CRASH_HOLD_AND_DISARM) {
                arming.disarm();
            }
        }
    }
}
