#include "mode.h"
#include "Plane.h"

bool ModeStallRecovery::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    plane.stall_state.state = Plane::STALLED_STATE::RELAX_WINGS_FULL_THR_PITCH_DOWN;
    plane.stall_state.start_ms = AP_HAL::millis();

    gcs().send_text(MAV_SEVERITY_INFO, "%sstart", gcsStrHeader);

    return true;
}

void ModeStallRecovery::_exit()
{
    plane.stall_state.state = Plane::STALLED_STATE::UNSTALLED;
    gcs().send_text(MAV_SEVERITY_INFO, "%sdone", gcsStrHeader);
}

void ModeStallRecovery::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t duration_ms = now_ms - plane.stall_state.start_ms;

    bool stage_complete = false;
    int32_t min_ms, max_ms;


    // determine timeout
    switch (plane.stall_state.state) {
    case Plane::STALLED_STATE::RELAX_WINGS_FULL_THR_PITCH_DOWN:
        min_ms = plane.g2.stall_recovery_duration1_min * 1000;
        max_ms = plane.g2.stall_recovery_duration1_max * 1000;
        break;

    case Plane::STALLED_STATE::LEVEL_WINGS_GAIN_AIRSPEED:
        min_ms = plane.g2.stall_recovery_duration2_min * 1000;
        max_ms = plane.g2.stall_recovery_duration2_max * 1000;
        break;

    default:
    case Plane::STALLED_STATE::UNSTALLED:
        min_ms = -1;
        max_ms = -1;
        break;
    }

    // sanity check params so we never get stuck here
    if (min_ms > max_ms) {
        min_ms = 1000;
        max_ms = 5000;
    } else {
        min_ms = constrain_int32(min_ms, -1, max_ms);
        max_ms = constrain_int32(max_ms, min_ms, 30000);
    }


    // during minimum time, we never timeout - force staying in this mode for a minimum duration
    const bool allow_next_state = (min_ms <= 0) || (duration_ms >= (uint32_t)min_ms);

    // when max timeout is reached, we've timed out
    const bool timed_out = (max_ms >= 0) && (duration_ms >= (uint32_t)max_ms);


    // determine state
    switch (plane.stall_state.state) {
    case Plane::STALLED_STATE::RELAX_WINGS_FULL_THR_PITCH_DOWN:
        stage_complete = state_complete_algorithm();

        if (allow_next_state && (timed_out || stage_complete)) {
            plane.stall_state.start_ms = now_ms;
            plane.stall_state.state = Plane::STALLED_STATE::LEVEL_WINGS_GAIN_AIRSPEED;
        }
        break;

    case Plane::STALLED_STATE::LEVEL_WINGS_GAIN_AIRSPEED:
        stage_complete = state_complete_algorithm();

        if (allow_next_state && (timed_out || stage_complete)) {
            plane.stall_state.start_ms = now_ms;
            plane.stall_state.state = Plane::STALLED_STATE::UNSTALLED;
        }
        break;

    case Plane::STALLED_STATE::UNSTALLED:
        break;
    } // switch _state

    if (timed_out) {
        gcs().send_text(MAV_SEVERITY_INFO, "%stimed out", gcsStrHeader);
    }


    // we've successfully recovered from the stall or have been doing it for too long and timed-out
    if (plane.stall_state.state == Plane::STALLED_STATE::UNSTALLED) {
        const bool mode_success = plane.set_mode(*plane.previous_mode, ModeReason::STALL_RECOVERY_RESUME);
        if (mode_success) {
            gcs().send_text(MAV_SEVERITY_INFO, "%sresuming mode %s", gcsStrHeader, plane.control_mode->name());
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "%sfailed switch to %s", gcsStrHeader, plane.previous_mode->name());
            plane.set_mode(plane.mode_rtl, ModeReason::STALL_RECOVERY_RESUME_FAIL);
        }
        return;
    }
}

bool ModeStallRecovery::has_servo_overrides()
{
    // apply servo behavior
    if (plane.stall_state.is_stalled()) {
        behavior_stalled();
        return true;

    } else if (plane.stall_state.is_recoverying()) {
        behavior_level_wings();
        return true;
    }
    return false;
}


bool ModeStallRecovery::state_complete_algorithm()
{
    bool stage_complete = false;

    float airspeed = -1.0f;
    plane.ahrs.airspeed_estimate(airspeed);

    switch (plane.stall_state.state) {
    case Plane::STALLED_STATE::RELAX_WINGS_FULL_THR_PITCH_DOWN:
        if (plane.g2.stall_recovery_algorithm1 & 0x01) {
            if ((airspeed > 0) && (plane.aparm.airspeed_min > 0) && (airspeed >= plane.aparm.airspeed_min)) {
                stage_complete = true;
                gcs().send_text(MAV_SEVERITY_INFO, "%sA arspd %.1f >= %d", gcsStrHeader, (double)airspeed, plane.aparm.airspeed_min.get());
            }
        }
        if (plane.g2.stall_recovery_algorithm1 & 0x02) {
            const float threshold = plane.g2.stall_recovery_spin_rate;
            const float spin_rate = fabsf(degrees(plane.ahrs.get_gyro().z));
            if ((threshold > 0) && (spin_rate < threshold)) {
                stage_complete = true;
                gcs().send_text(MAV_SEVERITY_INFO, "%sB spin %.1f < %.1f", gcsStrHeader, (double)spin_rate, (double)threshold);
            }
        }
        if (plane.g2.stall_recovery_algorithm1 & 0x04) {
            const float threshold = plane.g2.stall_recovery_sink_rate;
            if ((threshold > 0) && (plane.auto_state.sink_rate < threshold)) {
                stage_complete = true;
                gcs().send_text(MAV_SEVERITY_INFO, "%sC sink %.1f < %.1f", gcsStrHeader, (double)plane.auto_state.sink_rate, (double)threshold);
            }
        }
        break;

    case Plane::STALLED_STATE::LEVEL_WINGS_GAIN_AIRSPEED:
        {
        const float airspeed_cruise = plane.aparm.airspeed_cruise_cm * 0.01f;
        if (plane.g2.stall_recovery_algorithm2 & 0x01) {
            const float airspeed_cruise_thresh = airspeed_cruise * 0.95f;
            if ((airspeed > 0) && (airspeed_cruise_thresh > 0) && (airspeed >= airspeed_cruise_thresh)) {
                stage_complete = true;
                gcs().send_text(MAV_SEVERITY_INFO, "%sA arspd %.1f >= %.1f", gcsStrHeader, (double)airspeed, (double)airspeed_cruise_thresh);
            }
        }
        if (plane.g2.stall_recovery_algorithm2 & 0x02) {
            const float airspeed_cruise_thresh = airspeed_cruise * 0.95f;
            if ((airspeed > 0) && (airspeed_cruise_thresh > 0) && (airspeed >= airspeed_cruise_thresh)) {
                stage_complete = true;
                gcs().send_text(MAV_SEVERITY_INFO, "%sB arspd %.1f >= %.1f", gcsStrHeader, (double)airspeed, (double)airspeed_cruise_thresh);
            }
        }
        if (plane.g2.stall_recovery_algorithm2 & 0x04) {
            const float airspeed_cruise_thresh = airspeed_cruise * 0.90f;
            if ((airspeed > 0) && (airspeed_cruise_thresh > 0) && (airspeed >= airspeed_cruise_thresh)) {
                stage_complete = true;
                gcs().send_text(MAV_SEVERITY_INFO, "%sC arspd %.1f >= %.1f", gcsStrHeader, (double)airspeed, (double)airspeed_cruise_thresh);
            }
        }
        }
        break;

    default:
        stage_complete = true;
        gcs().send_text(MAV_SEVERITY_INFO, "%sC unknown state %d", gcsStrHeader, (int32_t)plane.stall_state.state);
        break;
    }

    return stage_complete;
}

void ModeStallRecovery::behavior_level_wings()
{
    if (!plane.arming.is_armed()) {
        plane.set_servos_throttle_while_disarmed();

    } else if (plane.g2.stall_recovery_throttle2 == -1) {
        // UNTESTED, may not work as expected
        plane.auto_throttle_mode = true;
        plane.update_load_factor();

    } else {
        // Apply throttle
        plane.auto_throttle_mode = false;
        const SRV_Channel::Aux_servo_function_t servo_throttle[] = {SRV_Channel::k_throttle, SRV_Channel::k_throttleLeft, SRV_Channel::k_throttleRight};
        for (uint8_t i=0; i<ARRAY_SIZE(servo_throttle); i++) {
            SRV_Channels::set_output_scaled(servo_throttle[i], plane.g2.stall_recovery_throttle2);
        }
    }

    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
}

void ModeStallRecovery::behavior_stalled()
{
    const SRV_Channel::Aux_servo_function_t servo_throttle[] = {SRV_Channel::k_throttle, SRV_Channel::k_throttleLeft, SRV_Channel::k_throttleRight};

    const SRV_Channel::Aux_servo_function_t servo_elevator[] = {SRV_Channel::k_elevator, SRV_Channel::k_elevator_with_input};

    const SRV_Channel::Aux_servo_function_t servo_rudder[] = {SRV_Channel::k_rudder};

    const SRV_Channel::Aux_servo_function_t servo_100[] = {};

    const SRV_Channel::Aux_servo_function_t servo_50[] = {};

    const SRV_Channel::Aux_servo_function_t servo_0[] = {
            SRV_Channel::k_aileron,
            SRV_Channel::k_airbrake,
            SRV_Channel::k_crow_inner, SRV_Channel::k_crow_outer,
            SRV_Channel::k_dspoilerLeft1, SRV_Channel::k_dspoilerLeft2, SRV_Channel::k_dspoilerRight1, SRV_Channel::k_dspoilerRight2,
            SRV_Channel::k_flap, SRV_Channel::k_flaperon_left, SRV_Channel::k_flaperon_right,SRV_Channel::k_flap_auto,
            };


    // Relax these flight surfaces
    for (uint8_t i=0; i<ARRAY_SIZE(servo_100) && sizeof(servo_100) > 0; i++) {
        SRV_Channels::set_output_scaled(servo_100[i], 100);
    }
    for (uint8_t i=0; i<ARRAY_SIZE(servo_50) && sizeof(servo_50) > 0; i++) {
        SRV_Channels::set_output_scaled(servo_50[i], 50);
    }
    for (uint8_t i=0; i<ARRAY_SIZE(servo_0) && sizeof(servo_0) > 0; i++) {
        SRV_Channels::set_output_scaled(servo_0[i], 0);
    }

    if (!plane.arming.is_armed()) {
        plane.set_servos_throttle_while_disarmed();
    } else {
        // Apply throttle
        for (uint8_t i=0; i<ARRAY_SIZE(servo_throttle); i++) {
            SRV_Channels::set_output_scaled(servo_throttle[i], plane.g2.stall_recovery_throttle1);
        }
    }

    // Set elevator to pitch down a little
    for (uint8_t i=0; i<ARRAY_SIZE(servo_elevator); i++) {
        SRV_Channels::set_output_scaled(servo_elevator[i], plane.g2.stall_recovery_elevator);
    }

    // Set rudder to fight a spin if gyro rate is above a certain threshold
    for (uint8_t i=0; i<ARRAY_SIZE(servo_rudder); i++) {
        SRV_Channels::set_output_scaled(servo_rudder[i], 0);
    }
}


