/*
 * Copyright (C) 2020  Kraus Hamdani Aerospace Inc. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.

 * Author: Tom Pittenger, Samuel Tabor
 */


#include "Plane.h"

/*
  stall detection algorithm and anti-stall management
 */


const AP_Param::GroupInfo Plane::StallState::var_info[] = {
    // @Param: STALL_DETECT
    // @DisplayName: Stall detection type
    // @Description: Stall detection type. Different aircraft stall in different ways.
    // @User: Advanced
    // @Bitmask: 0:CantHoldAltitude,1:SinkRate*2Max,2:SinkRate*4Max,3:RollError20deg,4:RollError30deg,5:RollError45deg,6:PitchError10deg,7:PitchError20deg,8:PitchError20deg,9:PitchError30deg,10:PitchError40deg,11:AltError10m,12:AltError20m,13:AltError40m,14:AltError60m
    AP_GROUPINFO("DETECT", 0, StallState, detection_bitmask, 0),

    AP_GROUPEND
};


/*
  Do we think we are stalled?
  Probabilistic method where a bool is low-passed and considered a probability.
*/
void Plane::StallState::detection_update(void)
{
    if (!plane.arming.is_armed() ||
        !plane.is_flying() ||
        plane.crash_state.is_crashed)
    {
        stall_clear();
        count = 0;
        detection_log();
        return;
    }

    const bool is_stalled_initial = is_stalled();

    last_detection = detect_stall(true);

    // LPF confidence
    confidence = ((float)last_detection * LPFcoef) + (confidence * (1.0f - LPFcoef));

    if ((is_stalled_initial != is_stalled()) && is_stalled()) {
        // we just stalled

        plane.gcs().send_text_rate_limited(MAV_SEVERITY_WARNING, 1000, detect_notify_gcs_last_ms, "STALL DETECTED!");

        stall_start();
        // TODO: trigger stall recovery
    }
    detection_log();
}

void Plane::StallState::detection_log()
{
    // log to AP_Logger
    AP::logger().Write(
       "STAL",
       "TimeUS,Actv,Raw,Any,Conf",
       "s---%",
       "F----",
       "QBBIf",
       AP_HAL::micros64(),
       (uint8_t)is_stalled(),
       (uint8_t)last_detection,
       (uint32_t)detection_bitmask_if_everything_enabled,
       (double)confidence);
}

// return true if we think we're stalling
bool Plane::StallState::detect_stall(bool allow_changing_state)
{
    //
    // Pre-process nav roll to apply rate limit. Needs to be done before potential early returns.
    //
    const uint32_t now_ms = AP_HAL::millis();

    // Time since last update.
    const float deltaT = 0.001f * (now_ms - last_update_ms);

    // Directly take nav_roll_cd, used if time since last update is too long indicating a reset.
    float limited_nav_roll = 0.01f*plane.nav_roll_cd;

    if (deltaT < 0.5) {
        // Difference between current nav roll and previous limited value.
        const float delta_angle      = 0.01f*plane.nav_roll_cd - last_limited_nav_roll;

        // New limited nav roll
        limited_nav_roll = last_limited_nav_roll + constrain_float(delta_angle, -50.0f*deltaT, 50.0f*deltaT);
    }

    // Save into old values
    last_update_ms        = now_ms;
    last_limited_nav_roll = limited_nav_roll;

    bool is_stalled = true;

    // check some generic things that are true for all aircraft types
    // ----------------

    if (plane.auto_state.sink_rate < definitely_not_stalling_sink_rate) {
        // we're going up fast, this is strong evidence that we're not stalling
        is_stalled = false;

        if (allow_changing_state) {
            stall_clear();
        }
    }



    // check bitmask options
    // ----------------
    // TECS is not able to hold the desired altitude
    const bool uncommanded_altitude_loss = plane.SpdHgt_Controller->uncommanded_altitude_loss();
    is_stalled &= detection_single_check(Stall_Detect::BAD_DESCENT, uncommanded_altitude_loss);


    // we're sinking faster than a controlled sink is allowed to be
    const float sink_rate = plane.SpdHgt_Controller->get_max_sinkrate();
    is_stalled &= detection_single_check(Stall_Detect::SINKRATE_2X_MAX, plane.auto_state.sink_rate > (sink_rate * 2));
    is_stalled &= detection_single_check(Stall_Detect::SINKRATE_4X_MAX, plane.auto_state.sink_rate > (sink_rate * 4));


    // we're rolling faster than a controlled roll is allowed to be
    const uint32_t roll_error_cd = fabsf(100.0f*limited_nav_roll - plane.ahrs.roll_sensor);
    is_stalled &= detection_single_check(Stall_Detect::BAD_ROLL_20DEG, roll_error_cd > 2000);
    is_stalled &= detection_single_check(Stall_Detect::BAD_ROLL_30DEG, roll_error_cd > 3000);


    // we're pitching faster than a controlled pitch is allowed to be
    const uint32_t pitch_error_cd = labs(labs(plane.nav_pitch_cd) - labs(plane.ahrs.pitch_sensor));
    is_stalled &= detection_single_check(Stall_Detect::BAD_PITCH_10DEG, pitch_error_cd > 1000);
    is_stalled &= detection_single_check(Stall_Detect::BAD_PITCH_20DEG, pitch_error_cd > 2000);
    is_stalled &= detection_single_check(Stall_Detect::BAD_PITCH_30DEG, pitch_error_cd > 3000);
    is_stalled &= detection_single_check(Stall_Detect::BAD_PITCH_40DEG, pitch_error_cd > 4000);


    // We don't use plane.altitude_error because the target component of this is not
    // limited by the maximum climb/sink rates.
    // positive plane.altitude_error_cm means too low
    const float altitude_error = plane.SpdHgt_Controller->get_altitude_error();
    is_stalled &= detection_single_check(Stall_Detect::BAD_ALT_10m, altitude_error > 10.0f);
    is_stalled &= detection_single_check(Stall_Detect::BAD_ALT_20m, altitude_error > 20.0f);
    is_stalled &= detection_single_check(Stall_Detect::BAD_ALT_40m, altitude_error > 40.0f);
    is_stalled &= detection_single_check(Stall_Detect::BAD_ALT_60m, altitude_error > 60.0f);

    return is_stalled;
}

bool Plane::StallState::detection_single_check(const Stall_Detect _bitmask, const bool check)
{
    const uint32_t bitmask = static_cast<uint32_t>(_bitmask);

    if (check) {
        detection_bitmask_if_everything_enabled |= bitmask;
    } else {
        detection_bitmask_if_everything_enabled &= !bitmask;
    }

    return (detection_bitmask & bitmask) != 0;
}

