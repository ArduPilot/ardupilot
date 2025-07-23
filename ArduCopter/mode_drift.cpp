#include "Copter.h"

#if MODE_DRIFT_ENABLED

/*
 * Init and run calls for drift flight mode
 */

#ifndef DRIFT_SPEEDGAIN
 # define DRIFT_SPEEDGAIN 8.0f
#endif
#ifndef DRIFT_SPEEDLIMIT
 # define DRIFT_SPEEDLIMIT 560.0f
#endif
#ifndef DRIFT_VEL_FORWARD_MIN
 # define DRIFT_VEL_FORWARD_MIN 2000.0f
#endif

#ifndef DRIFT_THR_ASSIST_GAIN
 # define DRIFT_THR_ASSIST_GAIN 0.0018f    // gain controlling amount of throttle assistance
#endif

#ifndef DRIFT_THR_ASSIST_MAX
 # define DRIFT_THR_ASSIST_MAX  0.3f    // maximum assistance throttle assist will provide
#endif

#ifndef DRIFT_THR_MIN
 # define DRIFT_THR_MIN         0.213f  // throttle assist will be active when pilot's throttle is above this value
#endif
#ifndef DRIFT_THR_MAX
 # define DRIFT_THR_MAX         0.787f  // throttle assist will be active when pilot's throttle is below this value
#endif

// drift_init - initialise drift controller
bool ModeDrift::init(bool ignore_checks)
{
    return true;
}

// drift_run - runs the drift controller
// should be called at 100hz or more
void ModeDrift::run()
{
    static float braker = 0.0f;
    static float roll_input_cd = 0.0f;

    // convert pilot input to lean angles
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());
    float target_roll_cd = rad_to_cd(target_roll_rad);
    float target_pitch_cd = rad_to_cd(target_pitch_rad);

    // Grab inertial velocity
    const Vector3f& vel_NEU_cms = pos_control->get_vel_estimate_NEU_cms();

    // rotate roll, pitch input from north facing to vehicle's perspective
    float vel_right_cms =  vel_NEU_cms.y * ahrs.cos_yaw() - vel_NEU_cms.x * ahrs.sin_yaw(); // body roll vel_NEU_cms
    float vel_forward_cms = vel_NEU_cms.y * ahrs.sin_yaw() + vel_NEU_cms.x * ahrs.cos_yaw(); // body pitch vel_NEU_cms

    // gain scheduling for yaw
    float vel_forward_2_cms = MIN(fabsf(vel_forward_cms), DRIFT_VEL_FORWARD_MIN);
    float target_yaw_rate_cds = target_roll_cd * (1.0f - (vel_forward_2_cms / 5000.0f)) * g2.command_model_acro_y.get_rate() / 45.0;

    vel_right_cms = constrain_float(vel_right_cms, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);
    vel_forward_cms = constrain_float(vel_forward_cms, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);

    roll_input_cd = roll_input_cd * 0.96f + (float)channel_yaw->get_control_in() * 0.04f;

    // convert user input into desired roll velocity
    float roll_vel_error = vel_right_cms - (roll_input_cd / DRIFT_SPEEDGAIN);

    // roll velocity is feed into roll acceleration to minimize slip
    target_roll_cd = roll_vel_error * -DRIFT_SPEEDGAIN;
    target_roll_cd = constrain_float(target_roll_cd, -4500.0f, 4500.0f);

    // If we let go of sticks, bring us to a stop
    if (is_zero(target_pitch_cd)) {
        // 0.14/ (0.03 * 100) = 4.6 seconds till full braking
        braker += 0.03f;
        braker = MIN(braker, DRIFT_SPEEDGAIN);
        target_pitch_cd = vel_forward_cms * braker;
    } else {
        braker = 0.0f;
    }

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
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

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll_cd, target_pitch_cd, target_yaw_rate_cds);

    // output pilot's throttle with angle boost
    const float assisted_throttle = get_throttle_assist(vel_NEU_cms.z, get_pilot_desired_throttle());
    attitude_control->set_throttle_out(assisted_throttle, true, g.throttle_filt);
}

// get_throttle_assist - return throttle output (range 0 ~ 1) based on pilot input and z-axis velocity
float ModeDrift::get_throttle_assist(float velz, float pilot_throttle_scaled)
{
    // throttle assist - adjusts throttle to slow the vehicle's vertical velocity
    //      Only active when pilot's throttle is between 213 ~ 787
    //      Assistance is strongest when throttle is at mid, drops linearly to no assistance at 213 and 787
    float thr_assist = 0.0f;
    if (pilot_throttle_scaled > DRIFT_THR_MIN && pilot_throttle_scaled < DRIFT_THR_MAX) {
        // calculate throttle assist gain
        thr_assist = 1.2f - ((float)fabsf(pilot_throttle_scaled - 0.5f) / 0.24f);
        thr_assist = constrain_float(thr_assist, 0.0f, 1.0f) * -DRIFT_THR_ASSIST_GAIN * velz;

        // ensure throttle assist never adjusts the throttle by more than 300 pwm
        thr_assist = constrain_float(thr_assist, -DRIFT_THR_ASSIST_MAX, DRIFT_THR_ASSIST_MAX);
    }
    
    return constrain_float(pilot_throttle_scaled + thr_assist, 0.0f, 1.0f);
}
#endif
