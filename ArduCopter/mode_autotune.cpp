#include "Copter.h"

#if AUTOTUNE_ENABLED == ENABLED

/*
 * Init and run calls for autotune flight mode
 *
 * Instructions:
 *      1) Set up one flight mode switch position to be AltHold.
 *      2) Set the Ch7 Opt or Ch8 Opt to AutoTune to allow you to turn the auto tuning on/off with the ch7 or ch8 switch.
 *      3) Ensure the ch7 or ch8 switch is in the LOW position.
 *      4) Wait for a calm day and go to a large open area.
 *      5) Take off and put the vehicle into AltHold mode at a comfortable altitude.
 *      6) Set the ch7/ch8 switch to the HIGH position to engage auto tuning:
 *          a) You will see it twitch about 20 degrees left and right for a few minutes, then it will repeat forward and back.
 *          b) Use the roll and pitch stick at any time to reposition the copter if it drifts away (it will use the original PID gains during repositioning and between tests).
 *             When you release the sticks it will continue auto tuning where it left off.
 *          c) Move the ch7/ch8 switch into the LOW position at any time to abandon the autotuning and return to the origin PIDs.
 *          d) Make sure that you do not have any trim set on your transmitter or the autotune may not get the signal that the sticks are centered.
 *      7) When the tune completes the vehicle will change back to the original PID gains.
 *      8) Put the ch7/ch8 switch into the LOW position then back to the HIGH position to test the tuned PID gains.
 *      9) Put the ch7/ch8 switch into the LOW position to fly using the original PID gains.
 *      10) If you are happy with the autotuned PID gains, leave the ch7/ch8 switch in the HIGH position, land and disarm to save the PIDs permanently.
 *          If you DO NOT like the new PIDS, switch ch7/ch8 LOW to return to the original PIDs. The gains will not be saved when you disarm
 *
 * What it's doing during each "twitch":
 *      a) invokes 90 deg/sec rate request
 *      b) records maximum "forward" roll rate and bounce back rate
 *      c) when copter reaches 20 degrees or 1 second has passed, it commands level
 *      d) tries to keep max rotation rate between 80% ~ 100% of requested rate (90deg/sec) by adjusting rate P
 *      e) increases rate D until the bounce back becomes greater than 10% of requested rate (90deg/sec)
 *      f) decreases rate D until the bounce back becomes less than 10% of requested rate (90deg/sec)
 *      g) increases rate P until the max rotate rate becomes greater than the request rate (90deg/sec)
 *      h) invokes a 20deg angle request on roll or pitch
 *      i) increases stab P until the maximum angle becomes greater than 110% of the requested angle (20deg)
 *      j) decreases stab P by 25%
 *
 */

#define AUTOTUNE_AXIS_BITMASK_ROLL            1
#define AUTOTUNE_AXIS_BITMASK_PITCH           2
#define AUTOTUNE_AXIS_BITMASK_YAW             4

#define AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS  500     // restart tuning if pilot has left sticks in middle for 2 seconds
#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS   1000     // timeout for tuning mode's testing step
#define AUTOTUNE_LEVEL_ANGLE_CD             500     // angle which qualifies as level
#define AUTOTUNE_LEVEL_RATE_RP_CD          1000     // rate which qualifies as level for roll and pitch
#define AUTOTUNE_LEVEL_RATE_Y_CD            750     // rate which qualifies as level for yaw
#define AUTOTUNE_REQUIRED_LEVEL_TIME_MS     500     // time we require the copter to be level
#define AUTOTUNE_RD_STEP                  0.05f     // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                  0.05f     // minimum increment when increasing/decreasing Rate P term
#define AUTOTUNE_SP_STEP                  0.05f     // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1f     // I is set 10x smaller than P during testing
#define AUTOTUNE_PI_RATIO_FINAL            1.0f     // I is set 1x P after testing
#define AUTOTUNE_YAW_PI_RATIO_FINAL        0.1f     // I is set 1x P after testing
#define AUTOTUNE_RD_MAX                  0.200f     // maximum Rate D value
#define AUTOTUNE_RLPF_MIN                  1.0f     // minimum Rate Yaw filter value
#define AUTOTUNE_RLPF_MAX                  5.0f     // maximum Rate Yaw filter value
#define AUTOTUNE_RP_MIN                   0.01f     // minimum Rate P value
#define AUTOTUNE_RP_MAX                    2.0f     // maximum Rate P value
#define AUTOTUNE_SP_MAX                   20.0f     // maximum Stab P value
#define AUTOTUNE_SP_MIN                    0.5f     // maximum Stab P value
#define AUTOTUNE_RP_ACCEL_MIN           4000.0f     // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_Y_ACCEL_MIN            1000.0f     // Minimum acceleration for Yaw
#define AUTOTUNE_Y_FILT_FREQ              10.0f     // Autotune filter frequency when testing Yaw
#define AUTOTUNE_SUCCESS_COUNT                4     // The number of successful iterations we need to freeze at current gains
#define AUTOTUNE_D_UP_DOWN_MARGIN          0.2f     // The margin below the target that we tune D in
#define AUTOTUNE_RD_BACKOFF                1.0f     // Rate D gains are reduced to 50% of their maximum value discovered during tuning
#define AUTOTUNE_RP_BACKOFF                1.0f     // Rate P gains are reduced to 97.5% of their maximum value discovered during tuning
#define AUTOTUNE_SP_BACKOFF                0.9f     // Stab P gains are reduced to 90% of their maximum value discovered during tuning
#define AUTOTUNE_ACCEL_RP_BACKOFF          1.0f     // back off from maximum acceleration
#define AUTOTUNE_ACCEL_Y_BACKOFF           1.0f     // back off from maximum acceleration

// roll and pitch axes
#define AUTOTUNE_TARGET_ANGLE_RLLPIT_CD     2000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_RATE_RLLPIT_CDS     9000    // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD 1000    // minimum target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS 4500    // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step

// yaw axis
#define AUTOTUNE_TARGET_ANGLE_YAW_CD        3000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_RATE_YAW_CDS        9000    // target yaw rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_ANGLE_YAW_CD     500    // minimum target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_MIN_RATE_YAW_CDS    1500    // minimum target yaw rate during AUTOTUNE_STEP_TWITCHING step

// Auto Tune message ids for ground station
#define AUTOTUNE_MESSAGE_STARTED 0
#define AUTOTUNE_MESSAGE_STOPPED 1
#define AUTOTUNE_MESSAGE_SUCCESS 2
#define AUTOTUNE_MESSAGE_FAILED 3
#define AUTOTUNE_MESSAGE_SAVED_GAINS 4

#define AUTOTUNE_ANNOUNCE_INTERVAL_MS 2000

// autotune_init - should be called when autotune mode is selected
bool Copter::ModeAutoTune::init(bool ignore_checks)
{
    bool success = true;

    switch (mode) {
        case FAILED:
            // autotune has been run but failed so reset state to uninitialized
            mode = UNINITIALISED;
            // fall through to restart the tuning
            FALLTHROUGH;

        case UNINITIALISED:
            // autotune has never been run
            success = start(false);
            if (success) {
                // so store current gains as original gains
                backup_gains_and_initialise();
                // advance mode to tuning
                mode = TUNING;
                // send message to ground station that we've started tuning
                update_gcs(AUTOTUNE_MESSAGE_STARTED);
            }
            break;

        case TUNING:
            // we are restarting tuning after the user must have switched ch7/ch8 off so we restart tuning where we left off
            success = start(false);
            if (success) {
                // reset gains to tuning-start gains (i.e. low I term)
                load_intra_test_gains();
                // write dataflash log even and send message to ground station
                Log_Write_Event(DATA_AUTOTUNE_RESTART);
                update_gcs(AUTOTUNE_MESSAGE_STARTED);
            }
            break;

        case SUCCESS:
            // we have completed a tune and the pilot wishes to test the new gains in the current flight mode
            // so simply apply tuning gains (i.e. do not change flight mode)
            load_tuned_gains();
            Log_Write_Event(DATA_AUTOTUNE_PILOT_TESTING);
            break;
    }

    // only do position hold if starting autotune from LOITER or POSHOLD
    use_poshold = (_copter.control_mode == LOITER || _copter.control_mode == POSHOLD);
    have_position = false;

    return success;
}

// stop - should be called when the ch7/ch8 switch is switched OFF
void Copter::ModeAutoTune::stop()
{
    // set gains to their original values
    load_orig_gains();

    // re-enable angle-to-rate request limits
    attitude_control->use_ff_and_input_shaping(true);

    // log off event and send message to ground station
    update_gcs(AUTOTUNE_MESSAGE_STOPPED);
    Log_Write_Event(DATA_AUTOTUNE_OFF);

    // Note: we leave the mode as it was so that we know how the autotune ended
    // we expect the caller will change the flight mode back to the flight mode indicated by the flight mode switch
}

// start - Initialize autotune flight mode
bool Copter::ModeAutoTune::start(bool ignore_checks)
{
    // only allow flip from Stabilize, AltHold,  PosHold or Loiter modes
    if (_copter.control_mode != STABILIZE && _copter.control_mode != ALT_HOLD &&
        _copter.control_mode != LOITER && _copter.control_mode != POSHOLD) {
        return false;
    }

    // ensure throttle is above zero
    if (ap.throttle_zero) {
        return false;
    }

    // ensure we are flying
    if (!motors->armed() || !ap.auto_armed || ap.land_complete) {
        return false;
    }

    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    return true;
}

const char *Copter::ModeAutoTune::level_issue_string() const
{
    switch (level_problem.issue) {
    case LEVEL_ISSUE_NONE:
        return "None";
    case LEVEL_ISSUE_ANGLE_ROLL:
        return "Angle(R)";
    case LEVEL_ISSUE_ANGLE_PITCH:
        return "Angle(P)";
    case LEVEL_ISSUE_ANGLE_YAW:
        return "Angle(Y)";
    case LEVEL_ISSUE_RATE_ROLL:
        return "Rate(R)";
    case LEVEL_ISSUE_RATE_PITCH:
        return "Rate(P)";
    case LEVEL_ISSUE_RATE_YAW:
        return "Rate(Y)";
    }
    return "Bug";
}

void Copter::ModeAutoTune::send_step_string()
{
    if (pilot_override) {
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Paused: Pilot Override Active");
        return;
    }
    switch (step) {
    case WAITING_FOR_LEVEL:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: WFL (%s) (%f > %f)", level_issue_string(), (double)(level_problem.current*0.01f), (double)(level_problem.maximum*0.01f));
        return;
    case UPDATE_GAINS:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: UPDATING_GAINS");
        return;
    case TWITCHING:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: TWITCHING");
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: unknown step");
}

const char *Copter::ModeAutoTune::type_string() const
{
    switch (tune_type) {
    case RD_UP:
        return "Rate D Up";
    case RD_DOWN:
        return "Rate D Down";
    case RP_UP:
        return "Rate P Up";
    case SP_DOWN:
        return "Angle P Down";
    case SP_UP:
        return "Angle P Up";
    }
    return "Bug";
}

void Copter::ModeAutoTune::do_gcs_announcements()
{
    const uint32_t now = millis();
    if (now - announce_time < AUTOTUNE_ANNOUNCE_INTERVAL_MS) {
        return;
    }
    float tune_rp = 0.0f;
    float tune_rd = 0.0f;
    float tune_sp = 0.0f;
    float tune_accel = 0.0f;
    char axis_char = '?';
    switch (axis) {
    case ROLL:
        tune_rp = tune_roll_rp;
        tune_rd = tune_roll_rd;
        tune_sp = tune_roll_sp;
        tune_accel = tune_roll_accel;
        axis_char = 'R';
        break;
    case PITCH:
        tune_rp = tune_pitch_rp;
        tune_rd = tune_pitch_rd;
        tune_sp = tune_pitch_sp;
        tune_accel = tune_pitch_accel;
        axis_char = 'P';
        break;
    case YAW:
        tune_rp = tune_yaw_rp;
        tune_rd = tune_yaw_rLPF;
        tune_sp = tune_yaw_sp;
        tune_accel = tune_yaw_accel;
        axis_char = 'Y';
        break;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: (%c) %s", axis_char, type_string());
    send_step_string();
    if (!is_zero(lean_angle)) {
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: lean=%f target=%f", (double)lean_angle, (double)target_angle);
    }
    if (!is_zero(rotation_rate)) {
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: rotation=%f target=%f", (double)(rotation_rate*0.01f), (double)(target_rate*0.01f));
    }
    switch (tune_type) {
    case RD_UP:
    case RD_DOWN:
    case RP_UP:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: p=%f d=%f", (double)tune_rp, (double)tune_rd);
        break;
    case SP_DOWN:
    case SP_UP:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: p=%f accel=%f", (double)tune_sp, (double)tune_accel);
        break;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: success %u/%u", counter, AUTOTUNE_SUCCESS_COUNT);

    announce_time = now;
}

// run - runs the autotune flight mode
// should be called at 100hz or more
void Copter::ModeAutoTune::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t target_climb_rate;

    // tell the user what's going on
    do_gcs_announcements();

    // initialize smoothing gain
    attitude_control->set_smoothing_gain(get_smoothing_gain());

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    // this should not actually be possible because of the init() checks
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control->relax_alt_hold_controllers(0.0f);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, _copter.aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    // check for pilot requested take-off - this should not actually be possible because of init() checks
    if (ap.land_complete && target_climb_rate > 0) {
        // indicate we are taking off
        set_land_complete(false);
        // clear i term when we're taking off
        set_throttle_takeoff();
    }

    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        pos_control->relax_alt_hold_controllers(0.0f);
        pos_control->update_z_controller();
    }else{
        // check if pilot is overriding the controls
        bool zero_rp_input = is_zero(target_roll) && is_zero(target_pitch);
        if (!zero_rp_input || !is_zero(target_yaw_rate) || target_climb_rate != 0) {
            if (!pilot_override) {
                pilot_override = true;
                // set gains to their original values
                load_orig_gains();
                attitude_control->use_ff_and_input_shaping(true);
            }
            // reset pilot override time
            override_time = millis();
            if (!zero_rp_input) {
                // only reset position on roll or pitch input
                have_position = false;
            }
        }else if (pilot_override) {
            // check if we should resume tuning after pilot's override
            if (millis() - override_time > AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS) {
                pilot_override = false;             // turn off pilot override
                // set gains to their intra-test values (which are very close to the original gains)
                // load_intra_test_gains(); //I think we should be keeping the originals here to let the I term settle quickly
                step = WAITING_FOR_LEVEL; // set tuning step back from beginning
                desired_yaw = ahrs.yaw_sensor;
            }
        }

        if (zero_rp_input) {
            // pilot input on throttle and yaw will still use position hold if enabled
            get_poshold_attitude(target_roll, target_pitch, desired_yaw);
        }

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // if pilot override call attitude controller
        if (pilot_override || mode != TUNING) {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        }else{
            // somehow get attitude requests from autotuning
            autotune_attitude_control();
        }

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
    }
}

bool Copter::ModeAutoTune::check_level(const LEVEL_ISSUE issue, const float current, const float maximum)
{
    if (current > maximum) {
        level_problem.current = current;
        level_problem.maximum = maximum;
        level_problem.issue = issue;
        return false;
    }
    return true;
}

bool Copter::ModeAutoTune::currently_level()
{
    if (!check_level(LEVEL_ISSUE_ANGLE_ROLL,
                     labs(ahrs.roll_sensor - roll_cd),
                     AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }

    if (!check_level(LEVEL_ISSUE_ANGLE_PITCH,
                     labs(ahrs.pitch_sensor - pitch_cd),
                     AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }
    if (!check_level(LEVEL_ISSUE_ANGLE_YAW,
                     labs(wrap_180_cd(ahrs.yaw_sensor-(int32_t)desired_yaw)),
                     AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }
    if (!check_level(LEVEL_ISSUE_RATE_ROLL,
                     (ToDeg(ahrs.get_gyro().x) * 100.0f),
                     AUTOTUNE_LEVEL_RATE_RP_CD)) {
        return false;
    }
    if (!check_level(LEVEL_ISSUE_RATE_PITCH,
                     (ToDeg(ahrs.get_gyro().y) * 100.0f),
                     AUTOTUNE_LEVEL_RATE_RP_CD)) {
        return false;
    }
    if (!check_level(LEVEL_ISSUE_RATE_YAW,
                     (ToDeg(ahrs.get_gyro().z) * 100.0f),
                     AUTOTUNE_LEVEL_RATE_Y_CD)) {
        return false;
    }
    return true;
}

// attitude_controller - sets attitude control targets during tuning
void Copter::ModeAutoTune::autotune_attitude_control()
{
    rotation_rate = 0.0f;        // rotation rate in radians/second
    lean_angle = 0.0f;
    const float direction_sign = positive_direction ? 1.0f : -1.0f;

    // check tuning step
    switch (step) {

    case WAITING_FOR_LEVEL:
        // Note: we should be using intra-test gains (which are very close to the original gains but have lower I)
        // re-enable rate limits
        attitude_control->use_ff_and_input_shaping(true);

        get_poshold_attitude(roll_cd, pitch_cd, desired_yaw);
        
        // hold level attitude
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_cd, pitch_cd, desired_yaw, true);

        // hold the copter level for 0.5 seconds before we begin a twitch
        // reset counter if we are no longer level
        if (!currently_level()) {
            step_start_time = millis();
        }

        // if we have been level for a sufficient amount of time (0.5 seconds) move onto tuning step
        if (millis() - step_start_time >= AUTOTUNE_REQUIRED_LEVEL_TIME_MS) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Twitch");
            // initiate variables for next step
            step = TWITCHING;
            step_start_time = millis();
            step_stop_time = step_start_time + AUTOTUNE_TESTING_STEP_TIMEOUT_MS;
            twitch_first_iter = true;
            test_max = 0.0f;
            test_min = 0.0f;
            rotation_rate_filt.reset(0.0f);
            rate_max = 0.0f;
            // set gains to their to-be-tested values
            load_twitch_gains();
        }

        switch (axis) {
        case ROLL:
            target_rate = constrain_float(ToDeg(attitude_control->max_rate_step_bf_roll())*100.0f, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
            target_angle = constrain_float(ToDeg(attitude_control->max_angle_step_bf_roll())*100.0f, AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD, AUTOTUNE_TARGET_ANGLE_RLLPIT_CD);
            start_rate = ToDeg(ahrs.get_gyro().x) * 100.0f;
            start_angle = ahrs.roll_sensor;
            rotation_rate_filt.set_cutoff_frequency(attitude_control->get_rate_roll_pid().filt_hz()*2.0f);
            if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
                rotation_rate_filt.reset(start_rate);
            } else {
                rotation_rate_filt.reset(0);
            }
        break;
        case PITCH:
            target_rate = constrain_float(ToDeg(attitude_control->max_rate_step_bf_pitch())*100.0f, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
            target_angle = constrain_float(ToDeg(attitude_control->max_angle_step_bf_pitch())*100.0f, AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD, AUTOTUNE_TARGET_ANGLE_RLLPIT_CD);
            start_rate = ToDeg(ahrs.get_gyro().y) * 100.0f;
            start_angle = ahrs.pitch_sensor;
            rotation_rate_filt.set_cutoff_frequency(attitude_control->get_rate_pitch_pid().filt_hz()*2.0f);
            if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
                rotation_rate_filt.reset(start_rate);
            } else {
                rotation_rate_filt.reset(0);
            }
            break;
        case YAW:
            target_rate = constrain_float(ToDeg(attitude_control->max_rate_step_bf_yaw()*0.75f)*100.0f, AUTOTUNE_TARGET_MIN_RATE_YAW_CDS, AUTOTUNE_TARGET_RATE_YAW_CDS);
            target_angle = constrain_float(ToDeg(attitude_control->max_angle_step_bf_yaw()*0.75f)*100.0f, AUTOTUNE_TARGET_MIN_ANGLE_YAW_CD, AUTOTUNE_TARGET_ANGLE_YAW_CD);
            start_rate = ToDeg(ahrs.get_gyro().z) * 100.0f;
            start_angle = ahrs.yaw_sensor;
            rotation_rate_filt.set_cutoff_frequency(AUTOTUNE_Y_FILT_FREQ);
            if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
                rotation_rate_filt.reset(start_rate);
            } else {
                rotation_rate_filt.reset(0);
            }
            break;
        }
        break;

    case TWITCHING:
        // Run the twitching step
        // Note: we should be using intra-test gains (which are very close to the original gains but have lower I)

        // disable rate limits
        attitude_control->use_ff_and_input_shaping(false);
        // hold current attitude
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);

        if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
            // step angle targets on first iteration
            if (twitch_first_iter) {
                twitch_first_iter = false;
                // Testing increasing stabilize P gain so will set lean angle target
                switch (axis) {
                case ROLL:
                    // request roll to 20deg
                    attitude_control->input_angle_step_bf_roll_pitch_yaw(direction_sign * target_angle, 0.0f, 0.0f);
                    break;
                case PITCH:
                    // request pitch to 20deg
                    attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f, direction_sign * target_angle, 0.0f);
                    break;
                case YAW:
                    // request pitch to 20deg
                    attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f, 0.0f, direction_sign * target_angle);
                    break;
                }
            }
        } else {
            // Testing rate P and D gains so will set body-frame rate targets.
            // Rate controller will use existing body-frame rates and convert to motor outputs
            // for all axes except the one we override here.
            switch (axis) {
            case ROLL:
                // override body-frame roll rate
                attitude_control->rate_bf_roll_target(direction_sign * target_rate + start_rate);
                break;
            case PITCH:
                // override body-frame pitch rate
                attitude_control->rate_bf_pitch_target(direction_sign * target_rate + start_rate);
                break;
            case YAW:
                // override body-frame yaw rate
                attitude_control->rate_bf_yaw_target(direction_sign * target_rate + start_rate);
                break;
            }
        }

        // capture this iterations rotation rate and lean angle
        // Add filter to measurements
        switch (axis) {
        case ROLL:
            if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().x) * 100.0f), _copter.scheduler.get_loop_period_s());
            } else {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().x) * 100.0f - start_rate), _copter.scheduler.get_loop_period_s());
            }
            lean_angle = direction_sign * (ahrs.roll_sensor - (int32_t)start_angle);
            break;
        case PITCH:
            if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().y) * 100.0f), _copter.scheduler.get_loop_period_s());
            } else {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().y) * 100.0f - start_rate), _copter.scheduler.get_loop_period_s());
            }
            lean_angle = direction_sign * (ahrs.pitch_sensor - (int32_t)start_angle);
            break;
        case YAW:
            if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().z) * 100.0f), _copter.scheduler.get_loop_period_s());
            } else {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().z) * 100.0f - start_rate), _copter.scheduler.get_loop_period_s());
            }
            lean_angle = direction_sign * wrap_180_cd(ahrs.yaw_sensor-(int32_t)start_angle);
            break;
        }

        switch (tune_type) {
        case RD_UP:
        case RD_DOWN:
            twitching_test(rotation_rate, target_rate, test_min, test_max);
            twitching_measure_acceleration(test_accel_max, rotation_rate, rate_max);
            if (lean_angle >= target_angle) {
                step = UPDATE_GAINS;
            }
            break;
        case RP_UP:
            twitching_test(rotation_rate, target_rate*(1+0.5f*g.autotune_aggressiveness), test_min, test_max);
            twitching_measure_acceleration(test_accel_max, rotation_rate, rate_max);
            if (lean_angle >= target_angle) {
                step = UPDATE_GAINS;
            }
            break;
        case SP_DOWN:
        case SP_UP:
            twitching_test(lean_angle, target_angle*(1+0.5f*g.autotune_aggressiveness), test_min, test_max);
            twitching_measure_acceleration(test_accel_max, rotation_rate - direction_sign * start_rate, rate_max);
            break;
        }

        // log this iterations lean angle and rotation rate
        Log_Write_AutoTuneDetails(lean_angle, rotation_rate);
        _copter.DataFlash.Log_Write_Rate(ahrs, *motors, *attitude_control, *pos_control);
        break;

    case UPDATE_GAINS:

        // re-enable rate limits
        attitude_control->use_ff_and_input_shaping(true);

        // log the latest gains
        if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
            switch (axis) {
            case ROLL:
                Log_Write_AutoTune(axis, tune_type, target_angle, test_min, test_max, tune_roll_rp, tune_roll_rd, tune_roll_sp, test_accel_max);
                break;
            case PITCH:
                Log_Write_AutoTune(axis, tune_type, target_angle, test_min, test_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, test_accel_max);
                break;
            case YAW:
                Log_Write_AutoTune(axis, tune_type, target_angle, test_min, test_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, test_accel_max);
                break;
            }
        } else {
            switch (axis) {
            case ROLL:
                Log_Write_AutoTune(axis, tune_type, target_rate, test_min, test_max, tune_roll_rp, tune_roll_rd, tune_roll_sp, test_accel_max);
                break;
            case PITCH:
                Log_Write_AutoTune(axis, tune_type, target_rate, test_min, test_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, test_accel_max);
                break;
            case YAW:
                Log_Write_AutoTune(axis, tune_type, target_rate, test_min, test_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, test_accel_max);
                break;
            }
        }

        // Check results after mini-step to increase rate D gain
        switch (tune_type) {
        case RD_UP:
            switch (axis) {
            case ROLL:
                updating_d_up(tune_roll_rd, g.autotune_min_d, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_min, test_max);
                break;
            case PITCH:
                updating_d_up(tune_pitch_rd, g.autotune_min_d, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_min, test_max);
                break;
            case YAW:
                updating_d_up(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RLPF_MAX, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_min, test_max);
                break;
            }
            break;
        // Check results after mini-step to decrease rate D gain
        case RD_DOWN:
            switch (axis) {
            case ROLL:
                updating_d_down(tune_roll_rd, g.autotune_min_d, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_min, test_max);
                break;
            case PITCH:
                updating_d_down(tune_pitch_rd, g.autotune_min_d, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_min, test_max);
                break;
            case YAW:
                updating_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_min, test_max);
                break;
            }
            break;
        // Check results after mini-step to increase rate P gain
        case RP_UP:
            switch (axis) {
            case ROLL:
                updating_p_up_d_down(tune_roll_rd, g.autotune_min_d, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_min, test_max);
                break;
            case PITCH:
                updating_p_up_d_down(tune_pitch_rd, g.autotune_min_d, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_min, test_max);
                break;
            case YAW:
                updating_p_up_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_min, test_max);
                break;
            }
            break;
        // Check results after mini-step to increase stabilize P gain
        case SP_DOWN:
            switch (axis) {
            case ROLL:
                updating_p_down(tune_roll_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, target_angle, test_max);
                break;
            case PITCH:
                updating_p_down(tune_pitch_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, target_angle, test_max);
                break;
            case YAW:
                updating_p_down(tune_yaw_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, target_angle, test_max);
                break;
            }
            break;
        // Check results after mini-step to increase stabilize P gain
        case SP_UP:
            switch (axis) {
            case ROLL:
                updating_p_up(tune_roll_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, target_angle, test_max);
                break;
            case PITCH:
                updating_p_up(tune_pitch_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, target_angle, test_max);
                break;
            case YAW:
                updating_p_up(tune_yaw_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, target_angle, test_max);
                break;
            }
            break;
        }

        // we've complete this step, finalize pids and move to next step
        if (counter >= AUTOTUNE_SUCCESS_COUNT) {

            // reset counter
            counter = 0;

            // move to the next tuning type
            switch (tune_type) {
            case RD_UP:
                tune_type = TuneType(tune_type + 1);
                break;
            case RD_DOWN:
                tune_type = TuneType(tune_type + 1);
                switch (axis) {
                case ROLL:
                    tune_roll_rd = MAX(g.autotune_min_d, tune_roll_rd * AUTOTUNE_RD_BACKOFF);
                    tune_roll_rp = MAX(AUTOTUNE_RP_MIN, tune_roll_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                case PITCH:
                    tune_pitch_rd = MAX(g.autotune_min_d, tune_pitch_rd * AUTOTUNE_RD_BACKOFF);
                    tune_pitch_rp = MAX(AUTOTUNE_RP_MIN, tune_pitch_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                case YAW:
                    tune_yaw_rLPF = MAX(AUTOTUNE_RLPF_MIN, tune_yaw_rLPF * AUTOTUNE_RD_BACKOFF);
                    tune_yaw_rp = MAX(AUTOTUNE_RP_MIN, tune_yaw_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                }
                break;
            case RP_UP:
                tune_type = TuneType(tune_type + 1);
                switch (axis) {
                case ROLL:
                    tune_roll_rp = MAX(AUTOTUNE_RP_MIN, tune_roll_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                case PITCH:
                    tune_pitch_rp = MAX(AUTOTUNE_RP_MIN, tune_pitch_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                case YAW:
                    tune_yaw_rp = MAX(AUTOTUNE_RP_MIN, tune_yaw_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                }
                break;
            case SP_DOWN:
                tune_type = TuneType(tune_type + 1);
                break;
            case SP_UP:
                // we've reached the end of a D-up-down PI-up-down tune type cycle
                tune_type = RD_UP;

                // advance to the next axis
                bool complete = false;
                switch (axis) {
                case ROLL:
                    tune_roll_sp = MAX(AUTOTUNE_SP_MIN, tune_roll_sp * AUTOTUNE_SP_BACKOFF);
                    tune_roll_accel = MAX(AUTOTUNE_RP_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    if (pitch_enabled()) {
                        axis = PITCH;
                    } else if (yaw_enabled()) {
                        axis = YAW;
                    } else {
                        complete = true;
                    }
                    break;
                case PITCH:
                    tune_pitch_sp = MAX(AUTOTUNE_SP_MIN, tune_pitch_sp * AUTOTUNE_SP_BACKOFF);
                    tune_pitch_accel = MAX(AUTOTUNE_RP_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    if (yaw_enabled()) {
                        axis = YAW;
                    } else {
                        complete = true;
                    }
                    break;
                case YAW:
                    tune_yaw_sp = MAX(AUTOTUNE_SP_MIN, tune_yaw_sp * AUTOTUNE_SP_BACKOFF);
                    tune_yaw_accel = MAX(AUTOTUNE_Y_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_Y_BACKOFF);
                    complete = true;
                    break;
                }

                // if we've just completed all axes we have successfully completed the autotune
                    // change to TESTING mode to allow user to fly with new gains
                if (complete) {
                    mode = SUCCESS;
                    update_gcs(AUTOTUNE_MESSAGE_SUCCESS);
                    Log_Write_Event(DATA_AUTOTUNE_SUCCESS);
                    AP_Notify::events.autotune_complete = 1;
                } else {
                    AP_Notify::events.autotune_next_axis = 1;
                }
                break;
            }
        }

        // reverse direction
        positive_direction = !positive_direction;

        if (axis == YAW) {
            attitude_control->input_euler_angle_roll_pitch_yaw(0.0f, 0.0f, ahrs.yaw_sensor, false);
        }

        // set gains to their intra-test values (which are very close to the original gains)
        load_intra_test_gains();

        // reset testing step
        step = WAITING_FOR_LEVEL;
        step_start_time = millis();
        break;
    }
}

// backup_gains_and_initialise - store current gains as originals
//  called before tuning starts to backup original gains
void Copter::ModeAutoTune::backup_gains_and_initialise()
{
    // initialise state because this is our first time
    if (roll_enabled()) {
        axis = ROLL;
    } else if (pitch_enabled()) {
        axis = PITCH;
    } else if (yaw_enabled()) {
        axis = YAW;
    }
    positive_direction = false;
    step = WAITING_FOR_LEVEL;
    step_start_time = millis();
    tune_type = RD_UP;

    desired_yaw = ahrs.yaw_sensor;

    g.autotune_aggressiveness = constrain_float(g.autotune_aggressiveness, 0.05f, 0.2f);

    orig_bf_feedforward = attitude_control->get_bf_feedforward();

    // backup original pids and initialise tuned pid values
    orig_roll_rp = attitude_control->get_rate_roll_pid().kP();
    orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
    orig_roll_rd = attitude_control->get_rate_roll_pid().kD();
    orig_roll_sp = attitude_control->get_angle_roll_p().kP();
    orig_roll_accel = attitude_control->get_accel_roll_max();
    tune_roll_rp = attitude_control->get_rate_roll_pid().kP();
    tune_roll_rd = attitude_control->get_rate_roll_pid().kD();
    tune_roll_sp = attitude_control->get_angle_roll_p().kP();
    tune_roll_accel = attitude_control->get_accel_roll_max();

    orig_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
    orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
    orig_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
    orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
    orig_pitch_accel = attitude_control->get_accel_pitch_max();
    tune_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
    tune_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
    tune_pitch_sp = attitude_control->get_angle_pitch_p().kP();
    tune_pitch_accel = attitude_control->get_accel_pitch_max();

    orig_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
    orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
    orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
    orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_hz();
    orig_yaw_accel = attitude_control->get_accel_yaw_max();
    orig_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    tune_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
    tune_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_hz();
    tune_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    tune_yaw_accel = attitude_control->get_accel_yaw_max();

    Log_Write_Event(DATA_AUTOTUNE_INITIALISED);
}

// load_orig_gains - set gains to their original values
//  called by stop and failed functions
void Copter::ModeAutoTune::load_orig_gains()
{
    attitude_control->bf_feedforward(orig_bf_feedforward);
    if (roll_enabled()) {
        if (!is_zero(orig_roll_rp)) {
            attitude_control->get_rate_roll_pid().kP(orig_roll_rp);
            attitude_control->get_rate_roll_pid().kI(orig_roll_ri);
            attitude_control->get_rate_roll_pid().kD(orig_roll_rd);
            attitude_control->get_angle_roll_p().kP(orig_roll_sp);
            attitude_control->set_accel_roll_max(orig_roll_accel);
        }
    }
    if (pitch_enabled()) {
        if (!is_zero(orig_pitch_rp)) {
            attitude_control->get_rate_pitch_pid().kP(orig_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(orig_pitch_ri);
            attitude_control->get_rate_pitch_pid().kD(orig_pitch_rd);
            attitude_control->get_angle_pitch_p().kP(orig_pitch_sp);
            attitude_control->set_accel_pitch_max(orig_pitch_accel);
        }
    }
    if (yaw_enabled()) {
        if (!is_zero(orig_yaw_rp)) {
            attitude_control->get_rate_yaw_pid().kP(orig_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(orig_yaw_ri);
            attitude_control->get_rate_yaw_pid().kD(orig_yaw_rd);
            attitude_control->get_rate_yaw_pid().filt_hz(orig_yaw_rLPF);
            attitude_control->get_angle_yaw_p().kP(orig_yaw_sp);
            attitude_control->set_accel_yaw_max(orig_yaw_accel);
        }
    }
}

// load_tuned_gains - load tuned gains
void Copter::ModeAutoTune::load_tuned_gains()
{
    if (!attitude_control->get_bf_feedforward()) {
        attitude_control->bf_feedforward(true);
        attitude_control->set_accel_roll_max(0.0f);
        attitude_control->set_accel_pitch_max(0.0f);
    }
    if (roll_enabled()) {
        if (!is_zero(tune_roll_rp)) {
            attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
            attitude_control->get_rate_roll_pid().kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
            attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
            attitude_control->get_angle_roll_p().kP(tune_roll_sp);
            attitude_control->set_accel_roll_max(tune_roll_accel);
        }
    }
    if (pitch_enabled()) {
        if (!is_zero(tune_pitch_rp)) {
            attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
            attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
            attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
            attitude_control->set_accel_pitch_max(tune_pitch_accel);
        }
    }
    if (yaw_enabled()) {
        if (!is_zero(tune_yaw_rp)) {
            attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
            attitude_control->get_rate_yaw_pid().kD(0.0f);
            attitude_control->get_rate_yaw_pid().filt_hz(tune_yaw_rLPF);
            attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
            attitude_control->set_accel_yaw_max(tune_yaw_accel);
        }
    }
}

// load_intra_test_gains - gains used between tests
//  called during testing mode's update-gains step to set gains ahead of return-to-level step
void Copter::ModeAutoTune::load_intra_test_gains()
{
    // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
    // sanity check the gains
    attitude_control->bf_feedforward(true);
    if (roll_enabled()) {
        attitude_control->get_rate_roll_pid().kP(orig_roll_rp);
        attitude_control->get_rate_roll_pid().kI(orig_roll_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_roll_pid().kD(orig_roll_rd);
        attitude_control->get_angle_roll_p().kP(orig_roll_sp);
    }
    if (pitch_enabled()) {
        attitude_control->get_rate_pitch_pid().kP(orig_pitch_rp);
        attitude_control->get_rate_pitch_pid().kI(orig_pitch_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_pitch_pid().kD(orig_pitch_rd);
        attitude_control->get_angle_pitch_p().kP(orig_pitch_sp);
    }
    if (yaw_enabled()) {
        attitude_control->get_rate_yaw_pid().kP(orig_yaw_rp);
        attitude_control->get_rate_yaw_pid().kI(orig_yaw_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_yaw_pid().kD(orig_yaw_rd);
        attitude_control->get_rate_yaw_pid().filt_hz(orig_yaw_rLPF);
        attitude_control->get_angle_yaw_p().kP(orig_yaw_sp);
    }
}

// load_twitch_gains - load the to-be-tested gains for a single axis
// called by attitude_control() just before it beings testing a gain (i.e. just before it twitches)
void Copter::ModeAutoTune::load_twitch_gains()
{
    switch (axis) {
        case ROLL:
            attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
            attitude_control->get_rate_roll_pid().kI(tune_roll_rp*0.01f);
            attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
            attitude_control->get_angle_roll_p().kP(tune_roll_sp);
            break;
        case PITCH:
            attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*0.01f);
            attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
            attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
            break;
        case YAW:
            attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*0.01f);
            attitude_control->get_rate_yaw_pid().kD(0.0f);
            attitude_control->get_rate_yaw_pid().filt_hz(tune_yaw_rLPF);
            attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
            break;
    }
}

// save_tuning_gains - save the final tuned gains for each axis
// save discovered gains to eeprom if autotuner is enabled (i.e. switch is in the high position)
void Copter::ModeAutoTune::save_tuning_gains()
{
    // if we successfully completed tuning
    if (mode == SUCCESS) {

        if (!attitude_control->get_bf_feedforward()) {
            attitude_control->bf_feedforward_save(true);
            attitude_control->save_accel_roll_max(0.0f);
            attitude_control->save_accel_pitch_max(0.0f);
        }

        // sanity check the rate P values
        if (roll_enabled() && !is_zero(tune_roll_rp)) {
            // rate roll gains
            attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
            attitude_control->get_rate_roll_pid().kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
            attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
            attitude_control->get_rate_roll_pid().save_gains();

            // stabilize roll
            attitude_control->get_angle_roll_p().kP(tune_roll_sp);
            attitude_control->get_angle_roll_p().save_gains();

            // acceleration roll
            attitude_control->save_accel_roll_max(tune_roll_accel);

            // resave pids to originals in case the autotune is run again
            orig_roll_rp = attitude_control->get_rate_roll_pid().kP();
            orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
            orig_roll_rd = attitude_control->get_rate_roll_pid().kD();
            orig_roll_sp = attitude_control->get_angle_roll_p().kP();
            orig_roll_accel = attitude_control->get_accel_roll_max();
        }

        if (pitch_enabled() && !is_zero(tune_pitch_rp)) {
            // rate pitch gains
            attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
            attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
            attitude_control->get_rate_pitch_pid().save_gains();

            // stabilize pitch
            attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
            attitude_control->get_angle_pitch_p().save_gains();

            // acceleration pitch
            attitude_control->save_accel_pitch_max(tune_pitch_accel);

            // resave pids to originals in case the autotune is run again
            orig_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
            orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
            orig_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
            orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
            orig_pitch_accel = attitude_control->get_accel_pitch_max();
        }

        if (yaw_enabled() && !is_zero(tune_yaw_rp)) {
            // rate yaw gains
            attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
            attitude_control->get_rate_yaw_pid().kD(0.0f);
            attitude_control->get_rate_yaw_pid().filt_hz(tune_yaw_rLPF);
            attitude_control->get_rate_yaw_pid().save_gains();

            // stabilize yaw
            attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
            attitude_control->get_angle_yaw_p().save_gains();

            // acceleration yaw
            attitude_control->save_accel_yaw_max(tune_yaw_accel);

            // resave pids to originals in case the autotune is run again
            orig_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
            orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
            orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
            orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_hz();
            orig_yaw_sp = attitude_control->get_angle_yaw_p().kP();
            orig_yaw_accel = attitude_control->get_accel_pitch_max();
        }
        // update GCS and log save gains event
        update_gcs(AUTOTUNE_MESSAGE_SAVED_GAINS);
        Log_Write_Event(DATA_AUTOTUNE_SAVEDGAINS);
        // reset Autotune so that gains are not saved again and autotune can be run again.
        mode = UNINITIALISED;
    }
}

// update_gcs - send message to ground station
void Copter::ModeAutoTune::update_gcs(uint8_t message_id)
{
    switch (message_id) {
        case AUTOTUNE_MESSAGE_STARTED:
            gcs().send_text(MAV_SEVERITY_INFO,"AutoTune: Started");
            break;
        case AUTOTUNE_MESSAGE_STOPPED:
            gcs().send_text(MAV_SEVERITY_INFO,"AutoTune: Stopped");
            break;
        case AUTOTUNE_MESSAGE_SUCCESS:
            gcs().send_text(MAV_SEVERITY_INFO,"AutoTune: Success");
            break;
        case AUTOTUNE_MESSAGE_FAILED:
            gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: Failed");
            break;
        case AUTOTUNE_MESSAGE_SAVED_GAINS:
            gcs().send_text(MAV_SEVERITY_INFO,"AutoTune: Saved gains");
            break;
    }
}

// axis helper functions
inline bool Copter::ModeAutoTune::roll_enabled() {
    return g.autotune_axis_bitmask & AUTOTUNE_AXIS_BITMASK_ROLL;
}

inline bool Copter::ModeAutoTune::pitch_enabled() {
    return g.autotune_axis_bitmask & AUTOTUNE_AXIS_BITMASK_PITCH;
}

inline bool Copter::ModeAutoTune::yaw_enabled() {
    return g.autotune_axis_bitmask & AUTOTUNE_AXIS_BITMASK_YAW;
}

// twitching_test - twitching tests
// update min and max and test for end conditions
void Copter::ModeAutoTune::twitching_test(float measurement, float target, float &measurement_min, float &measurement_max)
{
    // capture maximum measurement
    if (measurement > measurement_max) {
        // the measurement is continuing to increase without stopping
        measurement_max = measurement;
        measurement_min = measurement;
    }

    // capture minimum measurement after the measurement has peaked (aka "bounce back")
    if ((measurement < measurement_min) && (measurement_max > target * 0.5f)) {
        // the measurement is bouncing back
        measurement_min = measurement;
    }

    // calculate early stopping time based on the time it takes to get to 90%
    if (measurement_max < target * 0.75f) {
        // the measurement not reached the 90% threshold yet
        step_stop_time = step_start_time + (millis() - step_start_time) * 3.0f;
        step_stop_time = MIN(step_stop_time, step_start_time + AUTOTUNE_TESTING_STEP_TIMEOUT_MS);
    }

    if (measurement_max > target) {
        // the measurement has passed the target
        step = UPDATE_GAINS;
    }

    if (measurement_max-measurement_min > measurement_max*g.autotune_aggressiveness) {
        // the measurement has passed 50% of the target and bounce back is larger than the threshold
        step = UPDATE_GAINS;
    }

    if (millis() >= step_stop_time) {
        // we have passed the maximum stop time
        step = UPDATE_GAINS;
    }
}

// updating_d_up - increase D and adjust P to optimize the D term for a little bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void Copter::ModeAutoTune::updating_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max)
{
    if (measurement_max > target) {
        // if maximum measurement was higher than target
        // reduce P gain (which should reduce maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        if (tune_p < tune_p_min) {
            // P gain is at minimum so start reducing D
            tune_p = tune_p_min;
            tune_d -= tune_d*tune_d_step_ratio;
            if (tune_d <= tune_d_min) {
                // We have reached minimum D gain so stop tuning
                tune_d = tune_d_min;
                counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }else if ((measurement_max < target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
    }else{
        // we have a good measurement of bounce back
        if (measurement_max-measurement_min > measurement_max*g.autotune_aggressiveness) {
            // ignore the next result unless it is the same as this one
            ignore_next = true;
            // bounce back is bigger than our threshold so increment the success counter
            counter++;
        }else{
            if (ignore_next == false) {
                // bounce back is smaller than our threshold so decrement the success counter
                if (counter > 0 ) {
                    counter--;
                }
                // increase D gain (which should increase bounce back)
                tune_d += tune_d*tune_d_step_ratio*2.0f;
                // stop tuning if we hit maximum D
                if (tune_d >= tune_d_max) {
                    tune_d = tune_d_max;
                    counter = AUTOTUNE_SUCCESS_COUNT;
                    Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                }
            } else {
                ignore_next = false;
            }
        }
    }
}

// updating_d_down - decrease D and adjust P to optimize the D term for no bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void Copter::ModeAutoTune::updating_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max)
{
    if (measurement_max > target) {
        // if maximum measurement was higher than target
        // reduce P gain (which should reduce maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        if (tune_p < tune_p_min) {
            // P gain is at minimum so start reducing D gain
            tune_p = tune_p_min;
            tune_d -= tune_d*tune_d_step_ratio;
            if (tune_d <= tune_d_min) {
                // We have reached minimum D so stop tuning
                tune_d = tune_d_min;
                counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }else if ((measurement_max < target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
    }else{
        // we have a good measurement of bounce back
        if (measurement_max-measurement_min < measurement_max*g.autotune_aggressiveness) {
            if (ignore_next == false) {
                // bounce back is less than our threshold so increment the success counter
                counter++;
            } else {
                ignore_next = false;
            }
        }else{
            // ignore the next result unless it is the same as this one
            ignore_next = true;
            // bounce back is larger than our threshold so decrement the success counter
            if (counter > 0 ) {
                counter--;
            }
            // decrease D gain (which should decrease bounce back)
            tune_d -= tune_d*tune_d_step_ratio;
            // stop tuning if we hit minimum D
            if (tune_d <= tune_d_min) {
                tune_d = tune_d_min;
                counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }
}

// updating_p_down - decrease P until we don't reach the target before time out
// P is decreased to ensure we are not overshooting the target
void Copter::ModeAutoTune::updating_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float target, float measurement_max)
{
    if (measurement_max < target*(1+0.5f*g.autotune_aggressiveness)) {
        if (ignore_next == false) {
            // if maximum measurement was lower than target so increment the success counter
            counter++;
        } else {
            ignore_next = false;
        }
    }else{
        // ignore the next result unless it is the same as this one
        ignore_next = true;
        // if maximum measurement was higher than target so decrement the success counter
        if (counter > 0 ) {
            counter--;
        }
        // decrease P gain (which should decrease the maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        // stop tuning if we hit maximum P
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            counter = AUTOTUNE_SUCCESS_COUNT;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
    }
}

// updating_p_up - increase P to ensure the target is reached
// P is increased until we achieve our target within a reasonable time
void Copter::ModeAutoTune::updating_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float target, float measurement_max)
{
    if (measurement_max > target*(1+0.5f*g.autotune_aggressiveness)) {
        // ignore the next result unless it is the same as this one
        ignore_next = 1;
        // if maximum measurement was greater than target so increment the success counter
        counter++;
    }else{
        if (ignore_next == false) {
            // if maximum measurement was lower than target so decrement the success counter
            if (counter > 0 ) {
                counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p*tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            ignore_next = false;
        }
    }
}

// updating_p_up - increase P to ensure the target is reached while checking bounce back isn't increasing
// P is increased until we achieve our target within a reasonable time while reducing D if bounce back increases above the threshold
void Copter::ModeAutoTune::updating_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max)
{
    if (measurement_max > target*(1+0.5f*g.autotune_aggressiveness)) {
        // ignore the next result unless it is the same as this one
        ignore_next = true;
        // if maximum measurement was greater than target so increment the success counter
        counter++;
    } else if ((measurement_max < target) && (measurement_max > target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (measurement_max-measurement_min > measurement_max*g.autotune_aggressiveness) && (tune_d > tune_d_min)) {
        // if bounce back was larger than the threshold so decrement the success counter
        if (counter > 0 ) {
            counter--;
        }
        // decrease D gain (which should decrease bounce back)
        tune_d -= tune_d*tune_d_step_ratio;
        // stop tuning if we hit minimum D
        if (tune_d <= tune_d_min) {
            tune_d = tune_d_min;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
        // decrease P gain to match D gain reduction
        tune_p -= tune_p*tune_p_step_ratio;
        // stop tuning if we hit minimum P
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
        // cancel change in direction
        positive_direction = !positive_direction;
    }else{
        if (ignore_next == false) {
            // if maximum measurement was lower than target so decrement the success counter
            if (counter > 0 ) {
                counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p*tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            ignore_next = false;
        }
    }
}

// twitching_measure_acceleration - measure rate of change of measurement
void Copter::ModeAutoTune::twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max)
{
    if (rate_measurement_max < rate_measurement) {
        rate_measurement_max = rate_measurement;
        rate_of_change = (1000.0f*rate_measurement_max)/(millis() - step_start_time);
    }
}

// get attitude for slow position hold in autotune mode
void Copter::ModeAutoTune::get_poshold_attitude(float &roll_cd_out, float &pitch_cd_out, float &yaw_cd_out)
{
    roll_cd_out = pitch_cd_out = 0;

    if (!use_poshold) {
        // we are not trying to hold position
        return;
    }

    // do we know where we are?
    if (!_copter.position_ok()) {
        return;
    }

    if (!have_position) {
        have_position = true;
        start_position = inertial_nav.get_position();
    }

    // don't go past 10 degrees, as autotune result would deteriorate too much
    const float angle_max_cd = 1000;

    // hit the 10 degree limit at 20 meters position error
    const float dist_limit_cm = 2000;

    // we only start adjusting yaw if we are more than 5m from the
    // target position. That corresponds to a lean angle of 2.5 degrees
    const float yaw_dist_limit_cm = 500;
    
    Vector3f pdiff = inertial_nav.get_position() - start_position;
    pdiff.z = 0;
    float dist_cm = pdiff.length();
    if (dist_cm < 10) {
        // don't do anything within 10cm
        return;
    }

    /*
      very simple linear controller
     */
    float scaling = constrain_float(angle_max_cd * dist_cm / dist_limit_cm, 0, angle_max_cd);
    Vector2f angle_ne(pdiff.x, pdiff.y);
    angle_ne *= scaling / dist_cm;

    // rotate into body frame
    pitch_cd_out = angle_ne.x * ahrs.cos_yaw() + angle_ne.y * ahrs.sin_yaw();
    roll_cd_out  = angle_ne.x * ahrs.sin_yaw() - angle_ne.y * ahrs.cos_yaw();

    if (dist_cm < yaw_dist_limit_cm) {
        // no yaw adjustment
        return;
    }

    /*
      also point so that twitching occurs perpendicular to the wind,
      if we have drifted more than yaw_dist_limit_cm from the desired
      position. This ensures that autotune doesn't have to deal with
      more than 2.5 degrees of attitude on the axis it is tuning
     */
    float target_yaw_cd = degrees(atan2f(pdiff.y, pdiff.x)) * 100;
    if (axis == PITCH) {
        // for roll and yaw tuning we point along the wind, for pitch
        // we point across the wind
        target_yaw_cd += 9000;
    }
    // go to the nearest 180 degree mark, with 5 degree slop to prevent oscillation
    if (fabsf(yaw_cd_out - target_yaw_cd) > 9500) {
        target_yaw_cd += 18000;
    }

    yaw_cd_out = target_yaw_cd;
}

#endif  // AUTOTUNE_ENABLED == ENABLED
