#include "Copter.h"

#include "mode.h"

#if MODE_ACRO_ENABLED == ENABLED

/*
 * Init and run calls for acro flight mode
 */
void ModeAcro::run()
{
    // convert the input to the desired body frame rate
    float target_roll, target_pitch, target_yaw;
    get_pilot_desired_angle_rates(channel_roll->norm_input_dz(), channel_pitch->norm_input_dz(), channel_yaw->norm_input_dz(), target_roll, target_pitch, target_yaw);

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
        attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, target_yaw);
    } else {
        attitude_control->input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
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
    if (g2.acro_thr_mid > 0) {
        return g2.acro_thr_mid;
    }
    return Mode::throttle_hover();
}

// get_pilot_desired_angle_rates - transform pilot's normalised roll pitch and yaw input into a desired lean angle rates
// inputs are -1 to 1 and the function returns desired angle rates in centi-degrees-per-second
void ModeAcro::get_pilot_desired_angle_rates(float roll_in, float pitch_in, float yaw_in, float &roll_out, float &pitch_out, float &yaw_out)
{
    float rate_limit;
    Vector3f rate_ef_level_cd, rate_bf_level_cd, rate_bf_request_cd;

    // apply circular limit to pitch and roll inputs
    float total_in = norm(pitch_in, roll_in);

    if (total_in > 1.0) {
        float ratio = 1.0 / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // calculate roll, pitch rate requests
    
    // roll expo
    rate_bf_request_cd.x = g2.command_model_acro_rp.get_rate() * 100.0 * input_expo(roll_in, g2.command_model_acro_rp.get_expo());

    // pitch expo
    rate_bf_request_cd.y = g2.command_model_acro_rp.get_rate() * 100.0 * input_expo(pitch_in, g2.command_model_acro_rp.get_expo());

    // yaw expo
    rate_bf_request_cd.z = g2.command_model_acro_y.get_rate() * 100.0 * input_expo(yaw_in, g2.command_model_acro_y.get_expo());

    // calculate earth frame rate corrections to pull the copter back to level while in ACRO mode

    if (g.acro_trainer != (uint8_t)Trainer::OFF) {

        // get attitude targets
        const Vector3f att_target = attitude_control->get_att_target_euler_cd();

        // Calculate trainer mode earth frame rate command for roll
        int32_t roll_angle = wrap_180_cd(att_target.x);
        rate_ef_level_cd.x = -constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_roll;

        // Calculate trainer mode earth frame rate command for pitch
        int32_t pitch_angle = wrap_180_cd(att_target.y);
        rate_ef_level_cd.y = -constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_pitch;

        // Calculate trainer mode earth frame rate command for yaw
        rate_ef_level_cd.z = 0;

        // Calculate angle limiting earth frame rate commands
        if (g.acro_trainer == (uint8_t)Trainer::LIMITED) {
            const float angle_max = copter.aparm.angle_max;
            if (roll_angle > angle_max){
                rate_ef_level_cd.x += sqrt_controller(angle_max - roll_angle, g2.command_model_acro_rp.get_rate() * 100.0 / ACRO_LEVEL_MAX_OVERSHOOT, attitude_control->get_accel_roll_max_cdss(), G_Dt);
            }else if (roll_angle < -angle_max) {
                rate_ef_level_cd.x += sqrt_controller(-angle_max - roll_angle, g2.command_model_acro_rp.get_rate() * 100.0 / ACRO_LEVEL_MAX_OVERSHOOT, attitude_control->get_accel_roll_max_cdss(), G_Dt);
            }

            if (pitch_angle > angle_max){
                rate_ef_level_cd.y += sqrt_controller(angle_max - pitch_angle, g2.command_model_acro_rp.get_rate() * 100.0 / ACRO_LEVEL_MAX_OVERSHOOT, attitude_control->get_accel_pitch_max_cdss(), G_Dt);
            }else if (pitch_angle < -angle_max) {
                rate_ef_level_cd.y += sqrt_controller(-angle_max - pitch_angle, g2.command_model_acro_rp.get_rate() * 100.0 / ACRO_LEVEL_MAX_OVERSHOOT, attitude_control->get_accel_pitch_max_cdss(), G_Dt);
            }
        }

        // convert earth-frame level rates to body-frame level rates
        attitude_control->euler_rate_to_ang_vel(attitude_control->get_att_target_euler_cd() * radians(0.01f), rate_ef_level_cd, rate_bf_level_cd);

        // combine earth frame rate corrections with rate requests
        if (g.acro_trainer == (uint8_t)Trainer::LIMITED) {
            rate_bf_request_cd.x += rate_bf_level_cd.x;
            rate_bf_request_cd.y += rate_bf_level_cd.y;
            rate_bf_request_cd.z += rate_bf_level_cd.z;
        }else{
            float acro_level_mix = constrain_float(1-float(MAX(MAX(abs(roll_in), abs(pitch_in)), abs(yaw_in))/4500.0), 0, 1)*ahrs.cos_pitch();

            // Scale levelling rates by stick input
            rate_bf_level_cd = rate_bf_level_cd * acro_level_mix;

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request_cd.x)-fabsf(rate_bf_level_cd.x));
            rate_bf_request_cd.x += rate_bf_level_cd.x;
            rate_bf_request_cd.x = constrain_float(rate_bf_request_cd.x, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request_cd.y)-fabsf(rate_bf_level_cd.y));
            rate_bf_request_cd.y += rate_bf_level_cd.y;
            rate_bf_request_cd.y = constrain_float(rate_bf_request_cd.y, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request_cd.z)-fabsf(rate_bf_level_cd.z));
            rate_bf_request_cd.z += rate_bf_level_cd.z;
            rate_bf_request_cd.z = constrain_float(rate_bf_request_cd.z, -rate_limit, rate_limit);
        }
    }

    // hand back rate request
    roll_out = rate_bf_request_cd.x;
    pitch_out = rate_bf_request_cd.y;
    yaw_out = rate_bf_request_cd.z;
}
#endif
