#include "Copter.h"

#include "mode.h"

#if MODE_ACRO_ENABLED == ENABLED

/*
 * Init and run calls for acro flight mode
 */

bool Copter::ModeAcro::init(bool ignore_checks)
{
   // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
   if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
           (get_pilot_desired_throttle(channel_throttle->get_control_in(), copter.g2.acro_thr_mid) > copter.get_non_takeoff_throttle())) {
       return false;
   }

   return true;
}

void Copter::ModeAcro::run()
{
    float target_roll, target_pitch, target_yaw;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // convert the input to the desired body frame rate
    get_pilot_desired_angle_rates(channel_roll->get_control_in(), channel_pitch->get_control_in(), channel_yaw->get_control_in(), target_roll, target_pitch, target_yaw);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in(), g2.acro_thr_mid);

    // run attitude controller
    attitude_control->input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);

    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, copter.g.throttle_filt);
}


// get_pilot_desired_angle_rates - transform pilot's roll pitch and yaw input into a desired lean angle rates
// returns desired angle rates in centi-degrees-per-second
void Copter::ModeAcro::get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out)
{
    float rate_limit;
    Vector3f rate_ef_level, rate_bf_level, rate_bf_request;

    // apply circular limit to pitch and roll inputs
    float total_in = norm(pitch_in, roll_in);

    if (total_in > ROLL_PITCH_YAW_INPUT_MAX) {
        float ratio = (float)ROLL_PITCH_YAW_INPUT_MAX / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }
    
    // calculate roll, pitch rate requests
    if (g.acro_rp_expo <= 0) {
        rate_bf_request.x = roll_in * g.acro_rp_p;
        rate_bf_request.y = pitch_in * g.acro_rp_p;
    } else {
        // expo variables
        float rp_in, rp_in3, rp_out;

        // range check expo
        if (g.acro_rp_expo > 1.0f) {
            g.acro_rp_expo = 1.0f;
        }

        // roll expo
        rp_in = float(roll_in)/ROLL_PITCH_YAW_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_rp_expo * rp_in3) + ((1.0f - g.acro_rp_expo) * rp_in);
        rate_bf_request.x = ROLL_PITCH_YAW_INPUT_MAX * rp_out * g.acro_rp_p;

        // pitch expo
        rp_in = float(pitch_in)/ROLL_PITCH_YAW_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_rp_expo * rp_in3) + ((1.0f - g.acro_rp_expo) * rp_in);
        rate_bf_request.y = ROLL_PITCH_YAW_INPUT_MAX * rp_out * g.acro_rp_p;
    }

    // calculate yaw rate request
    rate_bf_request.z = get_pilot_desired_yaw_rate(yaw_in);

    // calculate earth frame rate corrections to pull the copter back to level while in ACRO mode

    if (g.acro_trainer != ACRO_TRAINER_DISABLED) {

        // get attitude targets
        const Vector3f att_target = attitude_control->get_att_target_euler_cd();

        // Calculate trainer mode earth frame rate command for roll
        int32_t roll_angle = wrap_180_cd(att_target.x);
        rate_ef_level.x = -constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_roll;

        // Calculate trainer mode earth frame rate command for pitch
        int32_t pitch_angle = wrap_180_cd(att_target.y);
        rate_ef_level.y = -constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_pitch;

        // Calculate trainer mode earth frame rate command for yaw
        rate_ef_level.z = 0;

        // Calculate angle limiting earth frame rate commands
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            const float angle_max = copter.aparm.angle_max;
            if (roll_angle > angle_max){
                rate_ef_level.x -=  g.acro_balance_roll*(roll_angle-angle_max);
            }else if (roll_angle < -angle_max) {
                rate_ef_level.x -=  g.acro_balance_roll*(roll_angle+angle_max);
            }

            if (pitch_angle > angle_max){
                rate_ef_level.y -=  g.acro_balance_pitch*(pitch_angle-angle_max);
            }else if (pitch_angle < -angle_max) {
                rate_ef_level.y -=  g.acro_balance_pitch*(pitch_angle+angle_max);
            }
        }

        // convert earth-frame level rates to body-frame level rates
        attitude_control->euler_rate_to_ang_vel(attitude_control->get_att_target_euler_cd()*radians(0.01f), rate_ef_level, rate_bf_level);

        // combine earth frame rate corrections with rate requests
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            rate_bf_request.x += rate_bf_level.x;
            rate_bf_request.y += rate_bf_level.y;
            rate_bf_request.z += rate_bf_level.z;
        }else{
            float acro_level_mix = constrain_float(float(1-MAX(MAX(abs(roll_in), abs(pitch_in)), abs(yaw_in)))/4500.0, 0, 1)*ahrs.cos_pitch();

            // Scale leveling rates by stick input
            rate_bf_level = rate_bf_level*acro_level_mix;

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.x)-fabsf(rate_bf_level.x));
            rate_bf_request.x += rate_bf_level.x;
            rate_bf_request.x = constrain_float(rate_bf_request.x, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.y)-fabsf(rate_bf_level.y));
            rate_bf_request.y += rate_bf_level.y;
            rate_bf_request.y = constrain_float(rate_bf_request.y, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request.z)-fabsf(rate_bf_level.z));
            rate_bf_request.z += rate_bf_level.z;
            rate_bf_request.z = constrain_float(rate_bf_request.z, -rate_limit, rate_limit);
        }
    }

    // hand back rate request
    roll_out = rate_bf_request.x;
    pitch_out = rate_bf_request.y;
    yaw_out = rate_bf_request.z;
}
#endif
