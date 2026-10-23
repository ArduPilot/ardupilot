#include "Rover.h"

// Code to detect a crash or block

// crash_check - sets hold mode and optionally disarms motors if a crash or block has been detected
// crashes are detected by speed below crash_vel_min or turn rate below crash_turn_rate_min
//     with throttle demand above crash_thr_min for longer than crash_timeout
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
    if ((g2.crash_angle != 0) && ((fabsf(ahrs.get_pitch_rad()) > radians(g2.crash_angle)) || (fabsf(ahrs.get_roll_rad()) > radians(g2.crash_angle)))) {
        crashed = true;
    }

    // TODO : Check if min vel can be calculated
    // min_vel = ( g2.crash_thr_min * g.speed_cruise) / g.throttle_cruise;

    if (!is_balancebot() && g2.crash_thr_min > 0.0f && g2.crash_timeout > 0.0f) {
        if (!crashed && 
            ((g2.crash_vel_min > 0.0f && ahrs.groundspeed() >= g2.crash_vel_min) ||                      // Check velocity
            (g2.crash_turn_rate_min > 0.0f && fabsf(ahrs.get_gyro().z) >= radians(g2.crash_turn_rate_min)) ||  // Check turn rate
            (fabsf(g2.motors.get_throttle()) < g2.crash_thr_min))) {
            crash_counter = 0;
            return;
        }

        // we may be crashing
        crash_counter++;

        // check if crashing for crash_timeout seconds
        if (crash_counter >= (g2.crash_timeout * 10)) {
            crashed = true;
        }
    }

    if (crashed) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::CRASH_CHECK,
                           LogErrorCode::CRASH_CHECK_CRASH);

        if (is_balancebot()) {
            // send message to gcs
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Crash: Disarming");
            arming.disarm(AP_Arming::Method::CRASH);
        } else {
            // send message to gcs
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Crash: Going to HOLD");
            // change mode to hold and disarm
            set_mode(mode_hold, ModeReason::CRASH_FAILSAFE);
            if (g.fs_crash_check == FS_CRASH_HOLD_AND_DISARM) {
                arming.disarm(AP_Arming::Method::CRASH);
            }
        }
    }
}

// override AP_Vehicle::is_crashed() to also check control mode reason
// returns true if control_mode_reason is CRASH_FAILSAFE or
// parent method returns true
bool Rover::is_crashed() const
{
    return (control_mode_reason == ModeReason::CRASH_FAILSAFE) ||
        AP_Vehicle::is_crashed();
}
