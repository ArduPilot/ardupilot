#include "Copter.h"

#if MODE_TURTLE_ENABLED == ENABLED

#define CRASH_FLIP_EXPO 35.0f
#define CRASH_FLIP_STICK_MINF 0.15f
#define power3(x) ((x) * (x) * (x))

bool ModeTurtle::init(bool ignore_checks)
{
    // do not enter the mode when already armed or when flying
    if (motors->armed() || SRV_Channels::get_dshot_esc_type() == 0) {
        return false;
    }

    // perform minimal arming checks
    if (!copter.mavlink_motor_control_check(*gcs().chan(0), true, "Turtle Mode")) {
        return false;
    }

    // do not enter the mode if sticks are not centered or throttle is not at zero
    if (!is_zero(channel_pitch->norm_input_dz())
        || !is_zero(channel_roll->norm_input_dz())
        || !is_zero(channel_yaw->norm_input_dz())
        || !is_zero(channel_throttle->norm_input_dz())) {
        return false;
    }

    // turn on notify leds
    AP_Notify::flags.esc_calibration = true;

    return true;
}

void ModeTurtle::arm_motors()
{
    if (hal.util->get_soft_armed()) {
        return;
    }

    // stop the spoolup block activating
    motors->set_spoolup_block(false);

    // reverse the motors
    hal.rcout->disable_channel_mask_updates();
    change_motor_direction(true);

    // disable throttle and gps failsafe
    g.failsafe_throttle.set(FS_THR_DISABLED);
    g.failsafe_gcs.set(FS_GCS_DISABLED);
    g.fs_ekf_action.set(0);

    // arm
    motors->armed(true);
    hal.util->set_soft_armed(true);
}

bool ModeTurtle::allows_arming(AP_Arming::Method method) const
{
    return true;
}

void ModeTurtle::exit()
{
    disarm_motors();

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;
}

void ModeTurtle::disarm_motors()
{
    if (!hal.util->get_soft_armed()) {
        return;
    }

    // disarm
    motors->armed(false);
    hal.util->set_soft_armed(false);

    // un-reverse the motors
    change_motor_direction(false);
    hal.rcout->enable_channel_mask_updates();

    // re-enable failsafes
    g.failsafe_throttle.load();
    g.failsafe_gcs.load();
    g.fs_ekf_action.load();
}

void ModeTurtle::change_motor_direction(bool reverse)
{
    AP_HAL::RCOutput::BLHeliDshotCommand direction = reverse ? AP_HAL::RCOutput::DSHOT_REVERSE : AP_HAL::RCOutput::DSHOT_NORMAL;
    AP_HAL::RCOutput::BLHeliDshotCommand inverse_direction = reverse ? AP_HAL::RCOutput::DSHOT_NORMAL : AP_HAL::RCOutput::DSHOT_REVERSE;

    if (!hal.rcout->get_reversed_mask()) {
        hal.rcout->send_dshot_command(direction, AP_HAL::RCOutput::ALL_CHANNELS, 0, 10, true);
    } else {
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
            if (!motors->is_motor_enabled(i)) {
                continue;
            }

            if ((hal.rcout->get_reversed_mask() & (1U << i)) == 0) {
                hal.rcout->send_dshot_command(direction, i, 0, 10, true);
            } else {
                hal.rcout->send_dshot_command(inverse_direction, i, 0, 10, true);
            }
        }
    }
}

void ModeTurtle::run()
{
    const float flip_power_factor = 1.0f - CRASH_FLIP_EXPO * 0.01f;
    const bool norc = copter.failsafe.radio || !copter.ap.rc_receiver_present;
    const float stick_deflection_pitch = norc ? 0.0f : channel_pitch->norm_input_dz();
    const float stick_deflection_roll = norc ? 0.0f : channel_roll->norm_input_dz();
    const float stick_deflection_yaw = norc ? 0.0f : channel_yaw->norm_input_dz();

    const float stick_deflection_pitch_abs = fabsf(stick_deflection_pitch);
    const float stick_deflection_roll_abs = fabsf(stick_deflection_roll);
    const float stick_deflection_yaw_abs = fabsf(stick_deflection_yaw);

    const float stick_deflection_pitch_expo = flip_power_factor * stick_deflection_pitch_abs + power3(stick_deflection_pitch_abs) * (1 - flip_power_factor);
    const float stick_deflection_roll_expo = flip_power_factor * stick_deflection_roll_abs + power3(stick_deflection_roll_abs) * (1 - flip_power_factor);
    const float stick_deflection_yaw_expo = flip_power_factor * stick_deflection_yaw_abs + power3(stick_deflection_yaw_abs) * (1 - flip_power_factor);

    float sign_pitch = stick_deflection_pitch < 0 ? -1 : 1;
    float sign_roll = stick_deflection_roll < 0 ? 1 : -1;

    float stick_deflection_length = sqrtf(sq(stick_deflection_pitch_abs) + sq(stick_deflection_roll_abs));
    float stick_deflection_expo_length = sqrtf(sq(stick_deflection_pitch_expo) + sq(stick_deflection_roll_expo));

    if (stick_deflection_yaw_abs > MAX(stick_deflection_pitch_abs, stick_deflection_roll_abs)) {
        // If yaw is the dominant, disable pitch and roll
        stick_deflection_length = stick_deflection_yaw_abs;
        stick_deflection_expo_length = stick_deflection_yaw_expo;
        sign_roll = 0;
        sign_pitch = 0;
    }

    const float cos_phi = (stick_deflection_length > 0) ? (stick_deflection_pitch_abs + stick_deflection_roll_abs) / (sqrtf(2.0f) * stick_deflection_length) : 0;
    const float cos_threshold = sqrtf(3.0f) / 2.0f; // cos(PI/6.0f)

    if (cos_phi < cos_threshold) {
        // Enforce either roll or pitch exclusively, if not on diagonal
        if (stick_deflection_roll_abs > stick_deflection_pitch_abs) {
            sign_pitch = 0;
        } else {
            sign_roll = 0;
        }
    }

    // Apply a reasonable amount of stick deadband
    const float crash_flip_stick_min_expo = flip_power_factor * CRASH_FLIP_STICK_MINF + power3(CRASH_FLIP_STICK_MINF) * (1 - flip_power_factor);
    const float flip_stick_range = 1.0f - crash_flip_stick_min_expo;
    const float flip_power = MAX(0.0f, stick_deflection_expo_length - crash_flip_stick_min_expo) / flip_stick_range;

    // at this point we have a power value in the range 0..1

    // normalise the roll and pitch input to match the motors
    Vector2f input{sign_roll, sign_pitch};
    motors_input = input.normalized() * 0.5;
    // we bypass spin min and friends in the deadzone because we only want spin up when the sticks are moved
    motors_output = !is_zero(flip_power) ? motors->thr_lin.thrust_to_actuator(flip_power) : 0.0f;
}

// actually write values to the motors
void ModeTurtle::output_to_motors()
{
    // throttle needs to be raised
    if (is_zero(channel_throttle->norm_input_dz())) {
        const uint32_t now = AP_HAL::millis();
        if (now - last_throttle_warning_output_ms > 5000) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Turtle: raise throttle to arm");
            last_throttle_warning_output_ms = now;
        }

        disarm_motors();
        return;
    }

    arm_motors();

    // check if motor are allowed to spin
    const bool allow_output = motors->armed() && motors->get_interlock();

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (!motors->is_motor_enabled(i)) {
            continue;
        }

        const Vector2f output{motors->get_roll_factor(i), motors->get_pitch_factor(i)};
        // if output aligns with input then use this motor
        if (!allow_output || (motors_input - output).length() > 0.5) {
            motors->rc_write(i, motors->get_pwm_output_min());
            continue;
        }

        int16_t pwm = motors->get_pwm_output_min() + (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * motors_output;

        motors->rc_write(i, pwm);
    }
}

#endif
