#include "mode.h"
#include "Plane.h"

bool ModeStallRecovery::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    start_ms = AP_HAL::millis();

    plane.is_stalled = true;

    gcs().send_text(MAV_SEVERITY_INFO, "%sstart", gcsStrHeader);

    return true;
}

void ModeStallRecovery::_exit()
{
    plane.is_stalled = false;
    gcs().send_text(MAV_SEVERITY_INFO, "%sexit", gcsStrHeader);
}

void ModeStallRecovery::update()
{
    int32_t min_ms, max_ms;
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t duration_ms = now_ms - start_ms;

    if (plane.is_stalled) {
        min_ms = plane.g2.stall_recovery_duration1_min * 1000;
        max_ms = plane.g2.stall_recovery_duration1_max * 1000;
    } else {
        min_ms = plane.g2.stall_recovery_duration2_min * 1000;
        max_ms = plane.g2.stall_recovery_duration2_max * 1000;
    }

    // sanity check params so we never get stuck here
    if (min_ms > max_ms) {
        min_ms = 2000;
        max_ms = 5000;
    } else {
        min_ms = constrain_int32(min_ms, -1, max_ms);
        max_ms = constrain_int32(max_ms, min_ms, 30000);
    }

    // during minimum time, we never timeout and force staying in this mode for a minimum duration
    // when true, minimum duration, if any, has been met
    const bool allow_next_state = (min_ms < 0) || (duration_ms >= (uint32_t)min_ms);

    if (allow_next_state) {

        // when max timeout is reached, we've timed out
        const bool timed_out = (max_ms >= 0) && (duration_ms >= (uint32_t)max_ms);

        if (timed_out || is_recovered_early()) {
            start_ms = now_ms;

            if (timed_out) {
                gcs().send_text(MAV_SEVERITY_INFO, "%stimed out", gcsStrHeader);
            }

            if (plane.is_stalled) {
                // we've successfully recovered from the stall, now lets do some level flight for phase 2
                plane.is_stalled = false;
            } else {
                // Phase 2 complete, we've successfully recovered from the stall finished performing some
                // level flight. Now lets go back to what we were doing before the stall
                resume_previous_mode();
                return;
            }
        }
    }

    set_servo_behavior();
}

void ModeStallRecovery::resume_previous_mode()
{
    const bool mode_success = plane.set_mode(*plane.previous_mode, ModeReason::STALL_RECOVERY_RESUME);
    if (mode_success) {
        gcs().send_text(MAV_SEVERITY_INFO, "%sresuming mode %s", gcsStrHeader, plane.control_mode->name());
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "%sfailed switch to %s", gcsStrHeader, plane.previous_mode->name());
        plane.set_mode(plane.mode_rtl, ModeReason::STALL_RECOVERY_RESUME_FAIL);
    }
}

bool ModeStallRecovery::is_recovered_early()
{
    int32_t is_recovered_bits = 0;
    const int32_t algorithm_bits = (plane.is_stalled) ? plane.g2.stall_recovery_algorithm1 : plane.g2.stall_recovery_algorithm2;

    if (algorithm_bits == 0) {
        return false;
    }

    float airspeed = -1.0f;
    plane.ahrs.airspeed_estimate(airspeed);

    if (plane.is_stalled) {
        if (algorithm_bits & 0x01) {
            if ((airspeed > 0) && (plane.aparm.airspeed_min > 0) && (airspeed >= plane.aparm.airspeed_min)) {
                is_recovered_bits |= 0x01;
                gcs().send_text(MAV_SEVERITY_INFO, "%sarspd %.1f >= %d", gcsStrHeader, (double)airspeed, plane.aparm.airspeed_min.get());
            }
        }
        if (algorithm_bits & 0x02) {
            const float threshold = plane.g2.stall_recovery_spin_rate;
            const float spin_rate = fabsf(degrees(plane.ahrs.get_gyro().z));
            if (!is_zero(threshold) && (spin_rate < threshold)) {
                is_recovered_bits |= 0x02;
                gcs().send_text(MAV_SEVERITY_INFO, "%sspin %.1f < %.1f", gcsStrHeader, (double)spin_rate, (double)threshold);
            }
        }
        if (algorithm_bits & 0x04) {
            const float threshold = plane.g2.stall_recovery_sink_rate;
            if (!is_zero(threshold) && (plane.auto_state.sink_rate < threshold)) {
                is_recovered_bits |= 0x04;
                gcs().send_text(MAV_SEVERITY_INFO, "%ssink %.1f < %.1f", gcsStrHeader, (double)plane.auto_state.sink_rate, (double)threshold);
            }
        }

    } else {
        const float airspeed_cruise = plane.aparm.airspeed_cruise_cm * 0.01f;
        if (algorithm_bits & 0x01) {
            const float airspeed_cruise_thresh = airspeed_cruise * 0.95f;
            if ((airspeed > 0) && (airspeed_cruise_thresh > 0) && (airspeed >= airspeed_cruise_thresh)) {
                is_recovered_bits |= 0x01;
                gcs().send_text(MAV_SEVERITY_INFO, "%sA arspd %.1f >= %.1f", gcsStrHeader, (double)airspeed, (double)airspeed_cruise_thresh);
            }
        }
        if (algorithm_bits & 0x02) {
            const float airspeed_cruise_thresh = airspeed_cruise * 0.95f;
            if ((airspeed > 0) && (airspeed_cruise_thresh > 0) && (airspeed >= airspeed_cruise_thresh)) {
                is_recovered_bits |= 0x02;
                gcs().send_text(MAV_SEVERITY_INFO, "%sB arspd %.1f >= %.1f", gcsStrHeader, (double)airspeed, (double)airspeed_cruise_thresh);
            }
        }
        if (algorithm_bits & 0x04) {
            const float airspeed_cruise_thresh = airspeed_cruise * 0.90f;
            if ((airspeed > 0) && (airspeed_cruise_thresh > 0) && (airspeed >= airspeed_cruise_thresh)) {
                is_recovered_bits |= 0x04;
                gcs().send_text(MAV_SEVERITY_INFO, "%sC arspd %.1f >= %.1f", gcsStrHeader, (double)airspeed, (double)airspeed_cruise_thresh);
            }
        }
    }

    return (is_recovered_bits & algorithm_bits) == algorithm_bits;
}

void ModeStallRecovery::set_servo_behavior()
{
    int16_t throttle = 0;

    if (plane.is_stalled) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);

        const int16_t scaled_elev = constrain_int16(plane.g2.stall_recovery_elevator,-100,100) * 45; // convert +/- percent to +/-4500
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, scaled_elev);

        throttle = plane.g2.stall_recovery_throttle1;
        plane.auto_throttle_mode = false;

    } else {
        // hold wings level
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;

        // assign throttle. Negative means auto-throttle
        throttle = plane.g2.stall_recovery_throttle2;
        if (throttle < 0) {
            plane.auto_throttle_mode = true;
        } else {
            plane.auto_throttle_mode = false;
        }
    }

    // set throttle
    if (plane.auto_state.last_flying_ms == 0) {
        // if we've never flown, don't allow throttle. This inhibits
        // the throttle while on the ground testing the feature
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);

    } else if (plane.auto_throttle_mode) {
        plane.calc_throttle();

    } else {
        throttle = constrain_int16(throttle, 0, 100);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    }


}


