#include "Plane.h"

/*
  stall detection algorithm and anti-stall management
 */

/*
  Do we think we are stalled?
  Probabilistic method where a bool is low-passed and considered a probability.
*/
void Plane::stall_detection_update(void)
{
    if (control_mode == &mode_stallrecovery) {
        // we are actively recovering from a stall. Let the mode control all stall values, we'll take over when it's done.
        stall_detection_log();
        return;
    }

    if (!arming.is_armed() ||
        auto_state.last_flying_ms == 0 ||
        !is_flying() ||
        crash_state.is_crashed)
    {
        stall_state.stall_clear();
        stall_state.recovering_clear();
        stall_state.count = 0;
        stall_detection_log();
        return;
    }


    const bool is_stalled_initial = stall_state.is_stalled();

    const bool is_stalled_new = stall_detection_algorithm(true);

    // LPF confidence
    stall_state.confidence = ((float)is_stalled_new * stall_state.coef) + (stall_state.confidence * (1.0f - stall_state.coef));

    if ((is_stalled_initial != stall_state.is_stalled()) && stall_state.is_stalled()) {
        // we just stalled
        stall_state.stall_start();

        gcs().send_text(MAV_SEVERITY_WARNING, "STALL DETECTED!");
        if (g2.stall_self_recovery_enable > 0) {
            set_mode(mode_stallrecovery, ModeReason::STALL_DETECTED);
        }

        if ((g2.stall_self_recovery_enable > 1) && !(plane.stall_state.count % g2.stall_self_recovery_enable)) {
            // bump up the min and cruise airspeeds after every couple stalls

            plane.aparm.airspeed_min = plane.aparm.airspeed_min.get() * 1.02f;
            gcs().send_text(MAV_SEVERITY_WARNING, "STALL set ARSPD_FBW_MIN: %d", (int)plane.aparm.airspeed_min);

            plane.aparm.airspeed_cruise_cm = plane.aparm.airspeed_cruise_cm.get() * 1.02f;
            gcs().send_text(MAV_SEVERITY_WARNING, "STALL set TRIM_ARSPD_CM: %d", (int)plane.aparm.airspeed_cruise_cm);
        }
    }
    stall_detection_log();
}

void Plane::stall_detection_log()
{
//    // log to AP_Logger
//    AP::logger().Write(
//        "STAL",
//        "TimeUS,h,dh,hdem,dhdem,spdem,sp,dsp,ith,iph,th,ph,dspdem,w,f",
//        "smnmnnnn----o--",
//        "F0000000----0--",
//        "QfffffffffffffB",
//        now,
//        (double)_height,
//        (double)_climb_rate,
//        (double)_hgt_dem_adj,
//        (double)_hgt_rate_dem,
//        (double)_TAS_dem_adj,
//        (double)_TAS_state,
//        (double)_vel_dot,
//        (double)_integTHR_state,
//        (double)_integSEB_state,
//        (double)_throttle_dem,
//        (double)_pitch_dem,
//        (double)_TAS_rate_dem,
//        (double)logging.SKE_weighting,
//        _flags_byte);
}

// return true if we think we're stalling
bool Plane::stall_detection_algorithm(bool allow_changing_state)
{
    if (g2.stall_detection_bitmask == STALL_DETECT_NEVER) {
        return false;
    }

    bool is_stalled = false;

    // check some generic things that are true for all aircraft types
    // ----------------

    if (auto_state.sink_rate < -3) {
        // we're going up fast, this is string evidence that we're not stalling
        is_stalled = false;

        if (allow_changing_state) {
            stall_state.stall_clear();
        }
    }



    // check bitmask options
    // ----------------

    // TECS is not able to hold the desired altitude
    if (g2.stall_detection_bitmask & STALL_DETECT_BAD_DESCENT) {
        is_stalled |= SpdHgt_Controller->get_flag_badDescent();
    }

    // we're sinking twice as fast as a controlled sink is allowed to
    if (g2.stall_detection_bitmask & STALL_DETECT_SINKRATE_2X_MAX) {
        is_stalled |= auto_state.sink_rate > (SpdHgt_Controller->get_max_sinkrate() * 2);
    }
    if (g2.stall_detection_bitmask & STALL_DETECT_SINKRATE_4X_MAX) {
        is_stalled |= auto_state.sink_rate > (SpdHgt_Controller->get_max_sinkrate() * 4);
        if (allow_changing_state) {
            stall_state.confidence *= 2;
        }
    }

    const uint32_t roll_error_cd = labs(labs(nav_roll_cd) - labs(ahrs.roll_sensor));
    if (g2.stall_detection_bitmask & STALL_DETECT_BAD_ROLL_30DEG) {
        is_stalled |= roll_error_cd >= 3000;
    }
    if (g2.stall_detection_bitmask & STALL_DETECT_BAD_ROLL_45DEG) {
        is_stalled |= roll_error_cd >= 4500;
    }

    const uint32_t pitch_error_cd = labs(labs(nav_pitch_cd) - labs(ahrs.pitch_sensor));
    if (g2.stall_detection_bitmask & STALL_DETECT_BAD_PITCH_10DEG) {
        is_stalled |= pitch_error_cd >= 1000;
    }
    if (g2.stall_detection_bitmask & STALL_DETECT_BAD_PITCH_20DEG) {
        is_stalled |= pitch_error_cd >= 2000;
    }
    if (g2.stall_detection_bitmask & STALL_DETECT_BAD_PITCH_30DEG) {
        is_stalled |= pitch_error_cd >= 3000;
    }
    if (g2.stall_detection_bitmask & STALL_DETECT_BAD_PITCH_40DEG) {
        is_stalled |= pitch_error_cd >= 4000;
    }

    if (g2.stall_detection_bitmask & STALL_DETECT_BAD_ALT_10m) {
        // posituve plane.altitude_error_cm means too high
        is_stalled |= plane.altitude_error_cm < -1000;
    }
    if (g2.stall_detection_bitmask & STALL_DETECT_BAD_ALT_20m) {
        is_stalled |= plane.altitude_error_cm < -2000;
    }
    if (g2.stall_detection_bitmask & STALL_DETECT_BAD_ALT_40m) {
        is_stalled |= plane.altitude_error_cm < -4000;
    }
    if (g2.stall_detection_bitmask & STALL_DETECT_BAD_ALT_60m) {
        is_stalled |= plane.altitude_error_cm < -6000;
    }

    return is_stalled;
}

