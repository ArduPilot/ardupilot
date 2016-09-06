/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Copter.h"


/*
 * Init and run calls for acro flight mode
 */

// acro_init - initialise acro controller
bool Copter::acro_init(bool ignore_checks)
{
   // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
   if (motors.armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) && (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
       return false;
   }
   // set target altitude to zero for reporting
   pos_control.set_alt_target(0);

   return true;
}

// acro_run - runs the acro controller
// should be called at 100hz or more
void Copter::acro_run()
{
    float target_roll_rate_rads, target_pitch_rate_rads, target_yaw_rate_rads;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() || ap.throttle_zero || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // convert the input to the desired body frame rate
    get_pilot_desired_angle_rates_rad(radians(channel_roll->get_control_in()*0.01f), radians(channel_pitch->get_control_in()*0.01f), radians(channel_yaw->get_control_in()*0.01f), target_roll_rate_rads, target_pitch_rate_rads, target_yaw_rate_rads);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // run attitude controller
    attitude_control.input_rate_bf_roll_pitch_yaw_rad(target_roll_rate_rads, target_pitch_rate_rads, target_yaw_rate_rads);

    // output pilot's throttle without angle boost
    attitude_control.set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}


// get_pilot_desired_angle_rates - transform pilot's roll pitch and yaw input into a desired lean angle rates
// returns desired angle rates in radians-per-second
void Copter::get_pilot_desired_angle_rates_rad(float roll_in_rad, float pitch_in_rad, float yaw_in_rad, float &roll_out_rads, float &pitch_out_rads, float &yaw_out_rads)
{
    float rate_limit_rads;
    Vector3f rate_ef_level_rads, rate_bf_level_rads, rate_bf_request_rads;

    // apply circular limit to pitch and roll inputs
    float total_in_rad = norm(pitch_in_rad, roll_in_rad);

    if (total_in_rad > ROLL_PITCH_INPUT_MAX_RAD) {
        float ratio = ROLL_PITCH_INPUT_MAX_RAD / total_in_rad;
        roll_in_rad *= ratio;
        pitch_in_rad *= ratio;
    }
    
    // calculate roll, pitch rate requests
    if (g.acro_expo <= 0) {
        rate_bf_request_rads.x = roll_in_rad * g.acro_rp_p;
        rate_bf_request_rads.y = pitch_in_rad * g.acro_rp_p;
    } else {
        // expo variables
        float rp_in, rp_in3, rp_out;

        // range check expo
        if (g.acro_expo > 1.0f) {
            g.acro_expo = 1.0f;
        }

        // roll expo
        rp_in = roll_in_rad/ROLL_PITCH_INPUT_MAX_RAD;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request_rads.x = ROLL_PITCH_INPUT_MAX_RAD * rp_out * g.acro_rp_p;

        // pitch expo
        rp_in = pitch_in_rad/ROLL_PITCH_INPUT_MAX_RAD;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request_rads.y = ROLL_PITCH_INPUT_MAX_RAD * rp_out * g.acro_rp_p;
    }

    // calculate yaw rate request
    rate_bf_request_rads.z = yaw_in_rad * g.acro_yaw_p;

    // calculate earth frame rate corrections to pull the copter back to level while in ACRO mode

    if (g.acro_trainer != ACRO_TRAINER_DISABLED) {
        // Calculate trainer mode earth frame rate command for roll
        float roll_angle_rad = wrap_PI(ahrs.roll);
        rate_ef_level_rads.x = -constrain_float(roll_angle_rad, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD) * g.acro_balance_roll;

        // Calculate trainer mode earth frame rate command for pitch
        float pitch_angle_rad = wrap_PI(ahrs.pitch);
        rate_ef_level_rads.y = -constrain_float(pitch_angle_rad, -ACRO_LEVEL_MAX_ANGLE_RAD, ACRO_LEVEL_MAX_ANGLE_RAD) * g.acro_balance_pitch;

        // Calculate trainer mode earth frame rate command for yaw
        rate_ef_level_rads.z = 0.0f;
        float angle_max_rad = radians(aparm.angle_max*0.01f);
        // Calculate angle limiting earth frame rate commands
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            if (roll_angle_rad > angle_max_rad){
                rate_ef_level_rads.x -=  g.acro_balance_roll*(roll_angle_rad-angle_max_rad);
            }else if (roll_angle_rad < -angle_max_rad) {
                rate_ef_level_rads.x -=  g.acro_balance_roll*(roll_angle_rad+angle_max_rad);
            }

            if (pitch_angle_rad > angle_max_rad){
                rate_ef_level_rads.y -=  g.acro_balance_pitch*(pitch_angle_rad-angle_max_rad);
            }else if (pitch_angle_rad < -angle_max_rad) {
                rate_ef_level_rads.y -=  g.acro_balance_pitch*(pitch_angle_rad+angle_max_rad);
            }
        }

        // convert earth-frame level rates to body-frame level rates
        attitude_control.euler_rate_to_ang_vel_rad(attitude_control.get_att_target_euler_rad(), rate_ef_level_rads, rate_bf_level_rads);

        // combine earth frame rate corrections with rate requests
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            rate_bf_request_rads.x += rate_bf_level_rads.x;
            rate_bf_request_rads.y += rate_bf_level_rads.y;
            rate_bf_request_rads.z += rate_bf_level_rads.z;
        }else{
            float acro_level_mix = constrain_float(1-MAX(MAX(abs(roll_in_rad), abs(pitch_in_rad)), abs(yaw_in_rad))/(DEG_TO_RAD*45.0f), 0, 1)*ahrs.cos_pitch();

            // Scale leveling rates by stick input
            rate_bf_level_rads = rate_bf_level_rads*acro_level_mix;

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit_rads = fabsf(fabsf(rate_bf_request_rads.x)-fabsf(rate_bf_level_rads.x));
            rate_bf_request_rads.x += rate_bf_level_rads.x;
            rate_bf_request_rads.x = constrain_float(rate_bf_request_rads.x, -rate_limit_rads, rate_limit_rads);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit_rads = fabsf(fabsf(rate_bf_request_rads.y)-fabsf(rate_bf_level_rads.y));
            rate_bf_request_rads.y += rate_bf_level_rads.y;
            rate_bf_request_rads.y = constrain_float(rate_bf_request_rads.y, -rate_limit_rads, rate_limit_rads);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit_rads = fabsf(fabsf(rate_bf_request_rads.z)-fabsf(rate_bf_level_rads.z));
            rate_bf_request_rads.z += rate_bf_level_rads.z;
            rate_bf_request_rads.z = constrain_float(rate_bf_request_rads.z, -rate_limit_rads, rate_limit_rads);
        }
    }

    // hand back rate request
    roll_out_rads = rate_bf_request_rads.x;
    pitch_out_rads = rate_bf_request_rads.y;
    yaw_out_rads = rate_bf_request_rads.z;
}
