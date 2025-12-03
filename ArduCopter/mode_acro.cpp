#include "Copter.h"

#include "mode.h"

#if MODE_ACRO_ENABLED

/*
 * Init and run calls for acro flight mode
 */
void ModeAcro::run()
{
    // convert the input to the desired body frame rate
    float target_roll_rads, target_pitch_rads, target_yaw_rads;
    get_pilot_desired_rates_rads(target_roll_rads, target_pitch_rads, target_yaw_rads);

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block

        // Attempting to Land or motors not yet spinning
        // if airmode is enabled only an actual landing will spool down the motors
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_target_and_rate(true);
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // run attitude controller
    if (g2.acro_options.get() & uint8_t(AcroOptions::RATE_LOOP_ONLY)) {
        // scale I by the value of angle P to mimic betaflight tunes
        attitude_control->scale_I_to_angle_P();

        // send rate commands to attitude controller (RATE_LOOP_ONLY bypasses full attitude stabilization)
        attitude_control->input_rate_bf_roll_pitch_yaw_2_rads(target_roll_rads, target_pitch_rads, target_yaw_rads);
    } else {
        // send rate commands to attitude controller with attitude stabilization
        attitude_control->input_rate_bf_roll_pitch_yaw_rads(target_roll_rads, target_pitch_rads, target_yaw_rads);
    }

    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(pilot_desired_throttle, false, copter.g.throttle_filt);
}

bool ModeAcro::init(bool ignore_checks)
{
    if (g2.acro_options.get() & uint8_t(AcroOptions::AIR_MODE)) {
        disable_air_mode_reset = false;
        copter.air_mode = AirMode::AIRMODE_ENABLED;
    }

    return true;
}

void ModeAcro::exit()
{
    if (!disable_air_mode_reset && (g2.acro_options.get() & uint8_t(AcroOptions::AIR_MODE))) {
        copter.air_mode = AirMode::AIRMODE_DISABLED;
    }
    disable_air_mode_reset = false;
}

void ModeAcro::air_mode_aux_changed()
{
    disable_air_mode_reset = true;
}

float ModeAcro::throttle_hover() const
{
    if (is_positive(g2.acro_thr_mid)) {
        return g2.acro_thr_mid;
    }
    return Mode::throttle_hover();
}

// return desired angular rates (radians/second) created from pilot inputs
void ModeAcro::get_pilot_desired_rates_rads(float &roll_out_rads, float &pitch_out_rads, float &yaw_out_rads)
{
    float rate_delta_max_rads;
    Vector3f rate_ef_level_rads, rate_bf_level_rads, rate_bf_request_rads;

    float roll_in_norm = channel_roll->norm_input_dz();
    float pitch_in_norm = channel_pitch->norm_input_dz();
    const float yaw_in_norm = channel_yaw->norm_input_dz();

    // apply circular limit to pitch and roll inputs
    float norm_in_length = norm(pitch_in_norm, roll_in_norm);

    if (norm_in_length > 1.0) {
        float ratio = 1.0 / norm_in_length;
        roll_in_norm *= ratio;
        pitch_in_norm *= ratio;
    }

    // calculate roll, pitch rate requests

    // roll rate request with input expo applied
    rate_bf_request_rads.x = radians(g2.command_model_acro_rp.get_rate()) * input_expo(roll_in_norm, g2.command_model_acro_rp.get_expo());

    // pitch rate request with input expo applied
    rate_bf_request_rads.y = radians(g2.command_model_acro_rp.get_rate()) * input_expo(pitch_in_norm, g2.command_model_acro_rp.get_expo());

    // yaw rate request with input expo applied
    rate_bf_request_rads.z = radians(g2.command_model_acro_y.get_rate()) * input_expo(yaw_in_norm, g2.command_model_acro_y.get_expo());

    // calculate earth frame rate corrections to pull the copter back to level while in ACRO mode

    if (g.acro_trainer != (uint8_t)Trainer::OFF) {

        // get attitude targets
        const Vector3f att_target_euler_rad = attitude_control->get_att_target_euler_rad();

        // Calculate trainer mode earth frame rate command for roll
        float roll_angle_rad = wrap_PI(att_target_euler_rad.x);
        rate_ef_level_rads.x = -constrain_float(roll_angle_rad, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD) * g.acro_balance_roll;

        // Calculate trainer mode earth frame rate command for pitch
        float pitch_angle_rad = wrap_PI(att_target_euler_rad.y);
        rate_ef_level_rads.y = -constrain_float(pitch_angle_rad, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD) * g.acro_balance_pitch;

        // Calculate trainer mode earth frame rate command for yaw
        rate_ef_level_rads.z = 0;

        // Calculate angle limiting earth frame rate commands
        if (g.acro_trainer == (uint8_t)Trainer::LIMITED) {
            const float angle_max_rad = attitude_control->lean_angle_max_rad();
            if (roll_angle_rad > angle_max_rad) {
                rate_ef_level_rads.x += sqrt_controller(angle_max_rad - roll_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_roll_max_radss(), G_Dt);
            } else if (roll_angle_rad < -angle_max_rad) {
                rate_ef_level_rads.x += sqrt_controller(-angle_max_rad - roll_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_roll_max_radss(), G_Dt);
            }

            if (pitch_angle_rad > angle_max_rad) {
                rate_ef_level_rads.y += sqrt_controller(angle_max_rad - pitch_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_pitch_max_radss(), G_Dt);
            } else if (pitch_angle_rad < -angle_max_rad) {
                rate_ef_level_rads.y += sqrt_controller(-angle_max_rad - pitch_angle_rad, radians(g2.command_model_acro_rp.get_rate()) / ACRO_LEVEL_MAX_OVERSHOOT_RAD, attitude_control->get_accel_pitch_max_radss(), G_Dt);
            }
        }

        // convert earth-frame level rates to body-frame level rates
        attitude_control->euler_rate_to_ang_vel(attitude_control->get_attitude_target_quat(), rate_ef_level_rads, rate_bf_level_rads);

        // combine earth frame rate corrections with rate requests
        if (g.acro_trainer == (uint8_t)Trainer::LIMITED) {
            rate_bf_request_rads.x += rate_bf_level_rads.x;
            rate_bf_request_rads.y += rate_bf_level_rads.y;
            rate_bf_request_rads.z += rate_bf_level_rads.z;
        } else {
            float acro_level_mix = constrain_float(1-float(MAX(MAX(abs(roll_in_norm), abs(pitch_in_norm)), abs(yaw_in_norm))), 0, 1) * ahrs.cos_pitch();

            // Scale levelling rates by stick input
            rate_bf_level_rads = rate_bf_level_rads * acro_level_mix;

            // Calculate the maximum allowed change in rate to prevent reversal through inverted
            rate_delta_max_rads = fabsf(fabsf(rate_bf_request_rads.x)-fabsf(rate_bf_level_rads.x));
            rate_bf_request_rads.x += rate_bf_level_rads.x;
            rate_bf_request_rads.x = constrain_float(rate_bf_request_rads.x, -rate_delta_max_rads, rate_delta_max_rads);

            // Calculate the maximum allowed change in rate to prevent reversal through inverted
            rate_delta_max_rads = fabsf(fabsf(rate_bf_request_rads.y)-fabsf(rate_bf_level_rads.y));
            rate_bf_request_rads.y += rate_bf_level_rads.y;
            rate_bf_request_rads.y = constrain_float(rate_bf_request_rads.y, -rate_delta_max_rads, rate_delta_max_rads);

            // Calculate the maximum allowed change in rate to prevent reversal through inverted
            rate_delta_max_rads = fabsf(fabsf(rate_bf_request_rads.z)-fabsf(rate_bf_level_rads.z));
            rate_bf_request_rads.z += rate_bf_level_rads.z;
            rate_bf_request_rads.z = constrain_float(rate_bf_request_rads.z, -rate_delta_max_rads, rate_delta_max_rads);
        }
    }

    // hand back rate request
    roll_out_rads = rate_bf_request_rads.x;
    pitch_out_rads = rate_bf_request_rads.y;
    yaw_out_rads = rate_bf_request_rads.z;
}
#endif
