#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

namespace {

constexpr float THROW_HIGH_SPEED_MS = 5.0f;
constexpr float THROW_VERTICAL_SPEED_MS = 0.5f;
constexpr float THROW_POST_RELEASE_ACCEL_G = 1.0f;
constexpr float THROW_FREEFALL_ACCEL_G = 0.25f;
constexpr float THROW_ATTITUDE_GOOD_COS = 0.866f;
constexpr float THROW_STABILIZE_THROTTLE = 0.5f;
constexpr uint32_t THROW_DETECTION_WINDOW_MS = 500;

}

bool ModeQThrow::_enter()
{
    if (!quadplane.tailsitter.enabled()) {
        gcs().send_text(MAV_SEVERITY_ERROR, "QThrow: tailsitter only");
        return false;
    }

    stage = Stage::Disarmed;
    free_fall_start_ms = 0;
    free_fall_start_vel_u_ms = 0;
    next_mode_attempted = false;

    return true;
}

bool ModeQThrow::_pre_arm_checks(size_t buflen, char *buffer) const
{
    if (!quadplane.tailsitter.enabled()) {
        hal.util->snprintf(buffer, buflen, "tailsitter only");
        return false;
    }

    return Mode::_pre_arm_checks(buflen, buffer);
}

void ModeQThrow::update()
{
    // Q_THROW is tailsitter-only, so use tailsitter stick mapping directly
    const float roll_input = (float)plane.channel_roll->get_control_in() / plane.channel_roll->get_range();
    const float pitch_input = (float)plane.channel_pitch->get_control_in() / plane.channel_pitch->get_range();

    if (quadplane.tailsitter.max_roll_angle > 0) {
        plane.nav_roll_cd = quadplane.tailsitter.max_roll_angle * 100.0f * roll_input;
    } else {
        plane.nav_roll_cd = roll_input * quadplane.attitude_control->lean_angle_max_cd();
    }

    plane.nav_pitch_cd = pitch_input * quadplane.attitude_control->lean_angle_max_cd();
    quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd);
}

void ModeQThrow::run()
{
    if (!plane.arming.is_armed_and_safety_off()) {
        stage = Stage::Disarmed;
        next_mode_attempted = false;
    } else if (stage == Stage::Disarmed) {
        gcs().send_text(MAV_SEVERITY_INFO, "QThrow: waiting for throw");
        stage = Stage::Detecting;
    } else if ((stage == Stage::Detecting) && throw_detected()) {
        gcs().send_text(MAV_SEVERITY_INFO, "QThrow: throw detected");
        stage = Stage::Uprighting;
    } else if ((stage == Stage::Uprighting) && throw_attitude_good() && !next_mode_attempted) {
        next_mode_attempted = true;
        IGNORE_RETURN(switch_to_next_mode());
    }

    switch (stage) {
    case Stage::Disarmed:
    case Stage::Detecting:
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        attitude_control->set_throttle_out(0.0f, true, 0.0f);
        quadplane.relax_attitude_control();
        break;

    case Stage::Uprighting:
        quadplane.hold_stabilize(THROW_STABILIZE_THROTTLE);
        plane.stabilize_roll();
        plane.stabilize_pitch();
        output_rudder_and_steering(0.0f);
        break;
    }
}

bool ModeQThrow::throw_detected()
{
    if (!ahrs.has_status(AP_AHRS::Status::ATTITUDE_VALID) ||
        !ahrs.has_status(AP_AHRS::Status::HORIZ_POS_ABS) ||
        !ahrs.has_status(AP_AHRS::Status::VERT_POS)) {
        return false;
    }

    const bool high_speed = pos_control->get_vel_estimate_NED_ms().length_squared() >
                            (THROW_HIGH_SPEED_MS * THROW_HIGH_SPEED_MS);
    const float vel_u_ms = pos_control->get_vel_estimate_U_ms();
    const bool changing_height = vel_u_ms > THROW_VERTICAL_SPEED_MS;
    const bool free_falling = ahrs.get_accel_ef().z > -THROW_FREEFALL_ACCEL_G * GRAVITY_MSS;
    const bool no_throw_action = plane.ins.get_accel().length() < THROW_POST_RELEASE_ACCEL_G * GRAVITY_MSS;

    const bool possible_throw_detected = (free_falling || high_speed) && changing_height && no_throw_action;
    const uint32_t now = AP_HAL::millis();

    if (possible_throw_detected && ((now - free_fall_start_ms) > THROW_DETECTION_WINDOW_MS)) {
        free_fall_start_ms = now;
        free_fall_start_vel_u_ms = vel_u_ms;
    }

    return ((now - free_fall_start_ms) < THROW_DETECTION_WINDOW_MS) &&
           ((vel_u_ms - free_fall_start_vel_u_ms) < -2.5f);
}

bool ModeQThrow::throw_attitude_good() const
{
    const Matrix3f &rot_mat = ahrs.get_rotation_body_to_ned();
    return rot_mat.c.z > THROW_ATTITUDE_GOOD_COS;
}

bool ModeQThrow::switch_to_next_mode()
{
    if (!plane.set_mode_by_number(Mode::Number::QHOVER, ModeReason::THROW_COMPLETE)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "QThrow: QHOVER transition failed");
        return false;
    }

    const int8_t next_mode_num = quadplane.throw_next_mode.get();
    if (next_mode_num < 0) {
        return true;
    }

    const Mode::Number next_mode = static_cast<Mode::Number>(next_mode_num);
    if (next_mode == Mode::Number::QHOVER) {
        return true;
    }

    Mode *next_mode_ptr = plane.mode_from_mode_num(next_mode);
    if ((next_mode_ptr == nullptr) ||
        (next_mode == Mode::Number::INITIALISING) ||
        (next_mode == Mode::Number::QTHROW)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "QThrow: invalid THROW_NEXT_MODE");
        return false;
    }

    if (!plane.set_mode_by_number(next_mode, ModeReason::THROW_COMPLETE)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "QThrow: next mode transition failed");
        return false;
    }

    return true;
}

#endif
