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
    stall_state.raw_algorithm_output = false;

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

    if (stall_state.inhibit_ms != 0) {
        // if someone told us to inihibt, freeze all stall processing for a brief time.
        // This is useful when changing waypoints where we will likely perform a
        // hard turn and roll/pitch error spikes for a moment
        if (AP_HAL::millis() - stall_state.inhibit_ms > 2000) {
            stall_state.inhibit_ms = 0;
        }
        stall_detection_log();
        return;
    }

    const bool is_stalled_initial = stall_state.is_stalled();

    stall_state.raw_algorithm_output = stall_detection_algorithm(true);

    // LPF confidence
    stall_state.confidence = ((float)stall_state.raw_algorithm_output * stall_state.coef) + (stall_state.confidence * (1.0f - stall_state.coef));

    if ((is_stalled_initial != stall_state.is_stalled()) && stall_state.is_stalled()) {
        // we just stalled

        gcs().send_text_rate_limited(MAV_SEVERITY_WARNING, 1000, stall_state.stall_stall_detect_notify_gcs_last_ms, "STALL DETECTED!");

        const bool mode_supports_mode_switch = (control_mode != &mode_manual) && (control_mode != &mode_stabilize);
        if (g2.stall_self_recovery_enable > 0 && mode_supports_mode_switch) {
            // start init stall and incrememnt stall count
            stall_state.stall_start();

            set_mode(mode_stallrecovery, ModeReason::STALL_DETECTED);

            if ((g2.stall_self_recovery_enable > 1) && (plane.stall_state.count % g2.stall_self_recovery_enable) == 0) {
                // bump up the min and cruise airspeeds after every couple stalls

                plane.aparm.airspeed_min = plane.aparm.airspeed_min.get() + 1;
                gcs().send_text(MAV_SEVERITY_WARNING, "STALL set ARSPD_FBW_MIN: %d", (int)plane.aparm.airspeed_min);

                plane.aparm.airspeed_cruise_cm = plane.aparm.airspeed_cruise_cm.get() * 1.02f;
                gcs().send_text(MAV_SEVERITY_WARNING, "STALL set TRIM_ARSPD_CM: %d", (int)plane.aparm.airspeed_cruise_cm);
            }
        }
    }
    stall_detection_log();
}

void Plane::stall_detection_log()
{
    // log to AP_Logger
    AP::logger().Write(
       "STAL",
       "TimeUS,Actv,Raw,All,Conf,Inh,Rec",
       "s------",
       "F------",
       "QBBIfIB",
       AP_HAL::micros64(),
       (uint8_t)stall_state.is_stalled(),
       (uint8_t)stall_state.raw_algorithm_output,
       (uint32_t)stall_state.algorithm_output_if_everything_enabled,
       (double)stall_state.confidence,
       (uint32_t)(stall_state.inhibit_ms != 0) ? AP_HAL::millis() - stall_state.inhibit_ms : 0,
       (uint8_t)stall_state.is_recovering());
}

// return true if we think we're stalling
bool Plane::stall_detection_algorithm(bool allow_changing_state)
{
    //
    // Pre-process nav roll to apply rate limit. Needs to be done before potential early returns.
    //

    // Time since last update.
    const float deltaT = 0.001f * (AP_HAL::millis() - stall_state.last_update_ms);

    // Directly take nav_roll_cd, used if time since last update is too long indicating a reset.
    float limited_nav_roll = 0.01f*nav_roll_cd;

    if (deltaT < 0.5) {
        // Difference between current nav roll and previous limited value.
        float delta_angle      = 0.01f*nav_roll_cd - stall_state.last_limited_nav_roll;

        // New limited nav roll
        limited_nav_roll = stall_state.last_limited_nav_roll + constrain_float(delta_angle, -50.0f*deltaT, 50.0f*deltaT);
    }

    // Save into old values
    stall_state.last_update_ms        = AP_HAL::millis();
    stall_state.last_limited_nav_roll = limited_nav_roll;

    int32_t is_stalled = 0;

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
    const bool tecs_bad_descent = SpdHgt_Controller->get_flag_badDescent();
    is_stalled |= stall_detection_single_check(STALL_DETECT_BAD_DESCENT, tecs_bad_descent);


    // we're sinking faster than a controlled sink is allowed to be
    const float sink_rate = SpdHgt_Controller->get_max_sinkrate();
    is_stalled |= stall_detection_single_check(STALL_DETECT_SINKRATE_2X_MAX, auto_state.sink_rate > (sink_rate * 2));
    is_stalled |= stall_detection_single_check(STALL_DETECT_SINKRATE_4X_MAX, auto_state.sink_rate > (sink_rate * 4));


    // we're rolling faster than a controlled roll is allowed to be
    const uint32_t roll_error_cd = labs(100.0f*limited_nav_roll - ahrs.roll_sensor);
    is_stalled |= stall_detection_single_check(STALL_DETECT_BAD_ROLL_20DEG, roll_error_cd > 2000);
    is_stalled |= stall_detection_single_check(STALL_DETECT_BAD_ROLL_30DEG, roll_error_cd > 3000);


    // we're pitching faster than a controlled pitch is allowed to be
    const uint32_t pitch_error_cd = labs(labs(nav_pitch_cd) - labs(ahrs.pitch_sensor));
    is_stalled |= stall_detection_single_check(STALL_DETECT_BAD_PITCH_10DEG, pitch_error_cd > 1000);
    is_stalled |= stall_detection_single_check(STALL_DETECT_BAD_PITCH_20DEG, pitch_error_cd > 2000);
    is_stalled |= stall_detection_single_check(STALL_DETECT_BAD_PITCH_30DEG, pitch_error_cd > 3000);
    is_stalled |= stall_detection_single_check(STALL_DETECT_BAD_PITCH_40DEG, pitch_error_cd > 4000);


    // We don't use plane.altitude_error because the target component of this is not
    // limited by the maximum climb/sink rates.
    // positive plane.altitude_error_cm means too low
    const float altitude_error = SpdHgt_Controller->get_altitude_error();
    is_stalled |= stall_detection_single_check(STALL_DETECT_BAD_ALT_10m, altitude_error > 10.0f);
    is_stalled |= stall_detection_single_check(STALL_DETECT_BAD_ALT_20m, altitude_error > 20.0f);
    is_stalled |= stall_detection_single_check(STALL_DETECT_BAD_ALT_40m, altitude_error > 40.0f);
    is_stalled |= stall_detection_single_check(STALL_DETECT_BAD_ALT_60m, altitude_error > 60.0f);

    stall_state.algorithm_output_if_everything_enabled = is_stalled;

    return (is_stalled & g2.stall_detection_bitmask) == g2.stall_detection_bitmask;
}

int32_t Plane::stall_detection_single_check(const uint32_t bitmask, const bool check)
{
    if (check) {
        return bitmask;
    }

    return 0;
}

