#include "Copter.h"

#if MODE_DRIFT_ENABLED

/*
 * Drift flight mode — meters/second and radians version
 */
//   Converted: 8 [cd/(cm/s)] * (π/18000 rad/cd) * (100 cm/m) = 0.13962634 rad/(m/s)
#ifndef DRIFT_SPEEDGAIN_RAD
 # define DRIFT_SPEEDGAIN_RAD 0.13962634f
#endif
#ifdef DRIFT_SPEEDGAIN
    #error please convert to radians and use DRIFT_SPEEDGAIN_RAD
#endif

#ifndef DRIFT_SPEEDLIMIT_MS
 # define DRIFT_SPEEDLIMIT_MS 5.60f
#endif
#ifdef DRIFT_SPEEDLIMIT
    #error please convert to meters per second and use DRIFT_SPEEDLIMIT_MS
#endif

#ifndef DRIFT_VEL_FORWARD_MIN_MS
 # define DRIFT_VEL_FORWARD_MIN_MS 20.0f
#endif
#ifdef DRIFT_VEL_FORWARD_MIN
    #error please convert to meters per second and use DRIFT_VEL_FORWARD_MIN_MS
#endif

#ifndef DRIFT_THR_ASSIST_GAIN_MS
 # define DRIFT_THR_ASSIST_GAIN_MS 0.18f
#endif
#ifdef DRIFT_THR_ASSIST_GAIN
    #error please convert to meters per second and use DRIFT_THR_ASSIST_GAIN_MS
#endif

#ifndef DRIFT_THR_ASSIST_MAX
 # define DRIFT_THR_ASSIST_MAX  0.3f      // maximum assistance throttle assist will provide
#endif

#ifndef DRIFT_THR_MIN
 # define DRIFT_THR_MIN         0.213f
#endif
#ifndef DRIFT_THR_MAX
 # define DRIFT_THR_MAX         0.787f
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
    static float roll_input_rad = 0.0f;

    // convert pilot input to lean angles (already radians)
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // Grab inertial velocity (already m/s)
    const Vector3f& vel_ned_ms = pos_control->get_vel_estimate_NED_ms();

    // rotate roll, pitch input from north facing to vehicle's perspective
    // body-frame components in m/s
    float vel_right_ms   =  vel_ned_ms.y * ahrs.cos_yaw() - vel_ned_ms.x * ahrs.sin_yaw(); // body roll axis
    float vel_forward_ms =  vel_ned_ms.y * ahrs.sin_yaw() + vel_ned_ms.x * ahrs.cos_yaw(); // body pitch axis

    // gain scheduling for yaw
    const float vel_forward_2_ms = MIN(fabsf(vel_forward_ms), DRIFT_VEL_FORWARD_MIN_MS);

    // yaw-rate schedule:
    const float yaw_rate_max_rads = radians(g2.command_model_acro_y.get_rate());
    const float target_yaw_rate_rads = (target_roll_rad / radians(45.0f)) * yaw_rate_max_rads * (1.0f - (vel_forward_2_ms / 50.0f));

    // Constrain body velocities
    vel_right_ms   = constrain_float(vel_right_ms,   -DRIFT_SPEEDLIMIT_MS, DRIFT_SPEEDLIMIT_MS);
    vel_forward_ms = constrain_float(vel_forward_ms, -DRIFT_SPEEDLIMIT_MS, DRIFT_SPEEDLIMIT_MS);

    // roll_input from yaw stick (convert centidegrees -> radians before LP)
    // (channel_yaw->get_control_in() returns centidegrees)
    const float yaw_stick_rad = cd_to_rad((float)channel_yaw->get_control_in());
    roll_input_rad = roll_input_rad * 0.96f + yaw_stick_rad * 0.04f;

    // convert user input into desired roll velocity term (m/s equivalent)
    float roll_vel_error_ms = vel_right_ms - (roll_input_rad / DRIFT_SPEEDGAIN_RAD);

    // roll velocity is fed into roll angle to minimize slip
    target_roll_rad = roll_vel_error_ms * -DRIFT_SPEEDGAIN_RAD;

    // constrain to ±45 deg
    target_roll_rad = constrain_float(target_roll_rad, -radians(45.0f), radians(45.0f));

    // If we let go of sticks, bring us to a stop
    if (is_zero(target_pitch_rad)) {
        // Clamp to the same coupling constant, now in rad/(m/s)
        braker += 0.03f;
        braker = MIN(braker, DRIFT_SPEEDGAIN_RAD);
        target_pitch_rad = vel_forward_ms * braker;
    } else {
        braker = 0.0f;
    }

    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        attitude_control->reset_yaw_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        break;
    }

    // call attitude controller (already expects radians)
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

    // output pilot's throttle with angle boost (velz now m/s)
    const float assisted_throttle = get_throttle_assist(vel_ned_ms.z, get_pilot_desired_throttle());
    attitude_control->set_throttle_out(assisted_throttle, true, g.throttle_filt);
}

// get_throttle_assist - return throttle output (range 0 ~ 1) based on pilot input and D-axis velocity (positive down)
float ModeDrift::get_throttle_assist(float vel_d_ms, float pilot_throttle_scaled)
{
    // throttle assist - adjusts throttle to slow the vehicle's vertical velocity
    //      Only active when pilot's throttle is between 0.213 ~ 0.787
    //      Assistance is strongest at mid, drops linearly to no assistance at 0.213 and 0.787
    float thr_assist = 0.0f;
    if (pilot_throttle_scaled > DRIFT_THR_MIN && pilot_throttle_scaled < DRIFT_THR_MAX) {
        // calculate throttle assist gain
        thr_assist = 1.2f - ((float)fabsf(pilot_throttle_scaled - 0.5f) / 0.24f);
        thr_assist = constrain_float(thr_assist, 0.0f, 1.0f) * DRIFT_THR_ASSIST_GAIN_MS * vel_d_ms;

        // ensure throttle assist never adjusts the throttle by more than 0.3 (≈300 pwm)
        thr_assist = constrain_float(thr_assist, -DRIFT_THR_ASSIST_MAX, DRIFT_THR_ASSIST_MAX);
    }

    return constrain_float(pilot_throttle_scaled + thr_assist, 0.0f, 1.0f);
}
#endif
