#include "Copter.h"

#if MODE_TURTLE_ENABLED == ENABLED

#define CRASH_FLIP_EXPO 35.0f
#define CRASH_FLIP_STICK_MINF 0.15f
#define power3(x) ((x)*(x)*(x))

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

    // do not enter the mode if sticks are not centered
    if (!is_zero(channel_pitch->norm_input_dz())
        || !is_zero(channel_roll->norm_input_dz())
        || !is_zero(channel_yaw->norm_input_dz())) {
            return false;
    }
    // reverse the motors
    hal.rcout->disable_channel_mask_updates();
    change_motor_direction(true);

    // disable throttle and gps failsafe
    g.failsafe_throttle = FS_THR_DISABLED;
    g.failsafe_gcs = FS_GCS_DISABLED;
    g.fs_ekf_action = 0;

    // arm
    motors->armed(true);
    hal.util->set_soft_armed(true);

    return true;
}

bool  ModeTurtle::allows_arming(AP_Arming::Method method) const
{
    return true;
}

void ModeTurtle::exit()
{
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

            if ((hal.rcout->get_reversed_mask() & (1U<<i)) == 0) {
                hal.rcout->send_dshot_command(direction, i, 0, 10, true);
            } else {
                hal.rcout->send_dshot_command(inverse_direction, i, 0, 10, true);
            }
        }
    }
}

void ModeTurtle::run()
{
    const float flipPowerFactor = 1.0f - CRASH_FLIP_EXPO / 100.0f;
    const bool norc = copter.failsafe.radio || !copter.ap.rc_receiver_present;
    const float stickDeflectionPitch =  norc ? 0.0f : channel_pitch->norm_input_dz();
    const float stickDeflectionRoll = norc ? 0.0f : channel_roll->norm_input_dz();
    const float stickDeflectionYaw = norc ? 0.0f : channel_yaw->norm_input_dz();

    const float stickDeflectionPitchAbs =  fabsf(stickDeflectionPitch);
    const float stickDeflectionRollAbs = fabsf(stickDeflectionRoll);
    const float stickDeflectionYawAbs = fabsf(stickDeflectionYaw);

    const float stickDeflectionPitchExpo = flipPowerFactor * stickDeflectionPitchAbs + power3(stickDeflectionPitchAbs) * (1 - flipPowerFactor);
    const float stickDeflectionRollExpo = flipPowerFactor * stickDeflectionRollAbs + power3(stickDeflectionRollAbs) * (1 - flipPowerFactor);
    const float stickDeflectionYawExpo = flipPowerFactor * stickDeflectionYawAbs + power3(stickDeflectionYawAbs) * (1 - flipPowerFactor);

    float signPitch = stickDeflectionPitch < 0 ? -1 : 1;
    float signRoll = stickDeflectionRoll < 0 ? 1 : -1;
    //float signYaw = stickDeflectionYaw < 0 ? -1 : 1;

    float stickDeflectionLength = sqrtf(sq(stickDeflectionPitchAbs) + sq(stickDeflectionRollAbs));
    float stickDeflectionExpoLength = sqrtf(sq(stickDeflectionPitchExpo) + sq(stickDeflectionRollExpo));

    if (stickDeflectionYawAbs > MAX(stickDeflectionPitchAbs, stickDeflectionRollAbs)) {
        // If yaw is the dominant, disable pitch and roll
        stickDeflectionLength = stickDeflectionYawAbs;
        stickDeflectionExpoLength = stickDeflectionYawExpo;
        signRoll = 0;
        signPitch = 0;
    }

    const float cosPhi = (stickDeflectionLength > 0) ? (stickDeflectionPitchAbs + stickDeflectionRollAbs) / (sqrtf(2.0f) * stickDeflectionLength) : 0;
    const float cosThreshold = sqrtf(3.0f)/2.0f; // cos(PI/6.0f)

    if (cosPhi < cosThreshold) {
        // Enforce either roll or pitch exclusively, if not on diagonal
        if (stickDeflectionRollAbs > stickDeflectionPitchAbs) {
            signPitch = 0;
        } else {
            signRoll = 0;
        }
    }

    // Apply a reasonable amount of stick deadband
    const float crashFlipStickMinExpo = flipPowerFactor * CRASH_FLIP_STICK_MINF + power3(CRASH_FLIP_STICK_MINF) * (1 - flipPowerFactor);
    const float flipStickRange = 1.0f - crashFlipStickMinExpo;
    const float flipPower = MAX(0.0f, stickDeflectionExpoLength - crashFlipStickMinExpo) / flipStickRange;

    // at this point we have a power value in the range 0..1

    // notmalise the roll and pitch input to match the motors
    Vector2f input {signRoll, signPitch};
    motors_input = input.normalized() * 0.5;
    // we bypass spin min and friends in the deadzone because we only want spin up when the sticks are moved
    motors_output = !is_zero(flipPower) ? motors->thrust_to_actuator(flipPower) : 0.0f;
}

// actually write values to the motors
void ModeTurtle::output_to_motors()
{
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (!motors->is_motor_enabled(i)) {
            continue;
        }

        const Vector2f output {motors->get_roll_factor(i), motors->get_pitch_factor(i)};
        // if output aligns with input then use this motor
        if ((motors_input - output).length() > 0.5) {
            motors->rc_write(i, motors->get_pwm_output_min());
            continue;
        }

        int16_t pwm = motors->get_pwm_output_min()
            + (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * motors_output;

        motors->rc_write(i, pwm);
    }
}

#endif
