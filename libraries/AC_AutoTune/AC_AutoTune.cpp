#include "AC_AutoTune.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Scheduler/AP_Scheduler.h>

/*
 * autotune support for multicopters
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

#define AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS  500     // restart tuning if pilot has left sticks in middle for 2 seconds
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 # define AUTOTUNE_LEVEL_ANGLE_CD           500     // angle which qualifies as level (Plane uses more relaxed 5deg)
 # define AUTOTUNE_LEVEL_RATE_RP_CD        1000     // rate which qualifies as level for roll and pitch (Plane uses more relaxed 10deg/sec)
#else
 # define AUTOTUNE_LEVEL_ANGLE_CD           250     // angle which qualifies as level
 # define AUTOTUNE_LEVEL_RATE_RP_CD         500     // rate which qualifies as level for roll and pitch
#endif
#define AUTOTUNE_LEVEL_RATE_Y_CD            750     // rate which qualifies as level for yaw
#define AUTOTUNE_REQUIRED_LEVEL_TIME_MS     500     // time we require the aircraft to be level
#define AUTOTUNE_LEVEL_TIMEOUT_MS          2000     // time out for level
#define AUTOTUNE_LEVEL_WARNING_INTERVAL_MS 5000     // level failure warning messages sent at this interval to users
#define AUTOTUNE_RD_BACKOFF                1.0f     // Rate D gains are reduced to 50% of their maximum value discovered during tuning
#define AUTOTUNE_RP_BACKOFF                1.0f     // Rate P gains are reduced to 97.5% of their maximum value discovered during tuning
#define AUTOTUNE_ACCEL_RP_BACKOFF          1.0f     // back off from maximum acceleration
#define AUTOTUNE_ACCEL_Y_BACKOFF           1.0f     // back off from maximum acceleration


// ifdef is not working.  Modified multi values to reflect heli requirements
#if APM_BUILD_TYPE(APM_BUILD_Heli)
 // heli defines
 #define AUTOTUNE_RP_ACCEL_MIN           20000.0f     // Minimum acceleration for Roll and Pitch
 #define AUTOTUNE_Y_ACCEL_MIN            10000.0f     // Minimum acceleration for Yaw
 #define AUTOTUNE_SP_BACKOFF                 1.0f     // Stab P gains are reduced to 90% of their maximum value discovered during tuning
#else
 // Frame specific defaults
 #define AUTOTUNE_RP_ACCEL_MIN            4000.0f     // Minimum acceleration for Roll and Pitch
 #define AUTOTUNE_Y_ACCEL_MIN             1000.0f     // Minimum acceleration for Yaw
 #define AUTOTUNE_SP_BACKOFF                 0.9f     // Stab P gains are reduced to 90% of their maximum value discovered during tuning
#endif // HELI_BUILD

// second table of user settable parameters for quadplanes, this
// allows us to go beyond the 64 parameter limit

AC_AutoTune::AC_AutoTune()
{
}

// autotune_init - should be called when autotune mode is selected
bool AC_AutoTune::init_internals(bool _use_poshold,
                                 AC_AttitudeControl *_attitude_control,
                                 AC_PosControl *_pos_control,
                                 AP_AHRS_View *_ahrs_view,
                                 AP_InertialNav *_inertial_nav)
{
    use_poshold = _use_poshold;
    attitude_control = _attitude_control;
    pos_control = _pos_control;
    ahrs_view = _ahrs_view;
    inertial_nav = _inertial_nav;
    motors = AP_Motors::get_singleton();

    // exit immediately if motor are not armed
    if ((motors == nullptr) || !motors->armed()) {
        return false;
    }

    // initialise position controller
    init_position_controller();

    switch (mode) {
    case FAILED:
        // fall through to restart the tuning
        FALLTHROUGH;

    case UNINITIALISED:
        // initializes dwell test sequence for rate_p_up and rate_d_up tests for tradheli
        freq_cnt = 0;
        start_freq = 0.0f;
        stop_freq = 0.0f;
        ff_up_first_iter = true;

        // autotune has never been run
        // so store current gains as original gains
        backup_gains_and_initialise();
        // advance mode to tuning
        mode = TUNING;
        // send message to ground station that we've started tuning
        update_gcs(AUTOTUNE_MESSAGE_STARTED);
        break;

    case TUNING:
        // we are restarting tuning so restart where we left off
        // reset test variables to continue where we left off
        // reset dwell test variables if sweep was interrupted in order to restart sweep
        if (!is_equal(start_freq, stop_freq)) {
            freq_cnt = 0;
            start_freq = 0.0f;
            stop_freq = 0.0f;
        }
        step = WAITING_FOR_LEVEL;
        step_start_time_ms = AP_HAL::millis();
        level_start_time_ms = step_start_time_ms;
        // reset gains to tuning-start gains (i.e. low I term)
        load_gains(GAIN_INTRA_TEST);
        AP::logger().Write_Event(LogEvent::AUTOTUNE_RESTART);
        update_gcs(AUTOTUNE_MESSAGE_STARTED);
        break;

    case SUCCESS:
        // we have completed a tune and the pilot wishes to test the new gains
        load_gains(GAIN_TUNED);
        update_gcs(AUTOTUNE_MESSAGE_TESTING);
        AP::logger().Write_Event(LogEvent::AUTOTUNE_PILOT_TESTING);
        break;
    }

    have_position = false;

    return true;
}

// stop - should be called when the ch7/ch8 switch is switched OFF
void AC_AutoTune::stop()
{
    // set gains to their original values
    load_gains(GAIN_ORIGINAL);

    // re-enable angle-to-rate request limits
    attitude_control->use_sqrt_controller(true);

    update_gcs(AUTOTUNE_MESSAGE_STOPPED);
    AP::logger().Write_Event(LogEvent::AUTOTUNE_OFF);

    // Note: we leave the mode as it was so that we know how the autotune ended
    // we expect the caller will change the flight mode back to the flight mode indicated by the flight mode switch
}

// initialise position controller
bool AC_AutoTune::init_position_controller(void)
{
    // initialize vertical maximum speeds and acceleration
    init_z_limits();

    // initialise the vertical position controller
    pos_control->init_z_controller();

    return true;
}

const char *AC_AutoTune::level_issue_string() const
{
    switch (level_problem.issue) {
    case LevelIssue::NONE:
        return "None";
    case LevelIssue::ANGLE_ROLL:
        return "Angle(R)";
    case LevelIssue::ANGLE_PITCH:
        return "Angle(P)";
    case LevelIssue::ANGLE_YAW:
        return "Angle(Y)";
    case LevelIssue::RATE_ROLL:
        return "Rate(R)";
    case LevelIssue::RATE_PITCH:
        return "Rate(P)";
    case LevelIssue::RATE_YAW:
        return "Rate(Y)";
    }
    return "Bug";
}

void AC_AutoTune::send_step_string()
{
    if (pilot_override) {
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Paused: Pilot Override Active");
        return;
    }
    switch (step) {
    case WAITING_FOR_LEVEL:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Leveling (%s %4.1f > %4.1f)", level_issue_string(), (double)(level_problem.current*0.01f), (double)(level_problem.maximum*0.01f));
        return;
    case UPDATE_GAINS:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Updating Gains");
        return;
    case TESTING:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Testing");
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: unknown step");
}

const char *AC_AutoTune::type_string() const
{
    switch (tune_type) {
    case RD_UP:
        return "Rate D Up";
    case RD_DOWN:
        return "Rate D Down";
    case RP_UP:
        return "Rate P Up";
    case RP_DOWN:
        return "Rate P Down";
    case RFF_UP:
        return "Rate FF Up";
    case RFF_DOWN:
        return "Rate FF Down";
    case SP_UP:
        return "Angle P Up";
    case SP_DOWN:
        return "Angle P Down";
    case MAX_GAINS:
        return "Find Max Gains";
    case TUNE_COMPLETE:
        return "Tune Complete";
    }
    return "Bug";
}

// run - runs the autotune flight mode
// should be called at 100hz or more
void AC_AutoTune::run()
{
    // initialize vertical speeds and acceleration
    init_z_limits();

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    // this should not actually be possible because of the init() checks
    if (!motors->armed() || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0.0f, true, 0.0f);
        pos_control->relax_z_controller(0.0f);
        return;
    }

    float target_roll_cd, target_pitch_cd, target_yaw_rate_cds;
    get_pilot_desired_rp_yrate_cd(target_roll_cd, target_pitch_cd, target_yaw_rate_cds);

    // get pilot desired climb rate
    const float target_climb_rate_cms = get_pilot_desired_climb_rate_cms();

    const bool zero_rp_input = is_zero(target_roll_cd) && is_zero(target_pitch_cd);

    const uint32_t now = AP_HAL::millis();
    if (!zero_rp_input || !is_zero(target_yaw_rate_cds) || !is_zero(target_climb_rate_cms)) {
        if (!pilot_override) {
            pilot_override = true;
            // set gains to their original values
            load_gains(GAIN_ORIGINAL);
            attitude_control->use_sqrt_controller(true);
        }
        // reset pilot override time
        override_time = now;
        if (!zero_rp_input) {
            // only reset position on roll or pitch input
            have_position = false;
        }
    } else if (pilot_override) {
        // check if we should resume tuning after pilot's override
        if (now - override_time > AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS) {
            pilot_override = false;             // turn off pilot override
            // set gains to their intra-test values (which are very close to the original gains)
            // load_gains(GAIN_INTRA_TEST); //I think we should be keeping the originals here to let the I term settle quickly
            step = WAITING_FOR_LEVEL; // set tuning step back from beginning
            step_start_time_ms = now;
            level_start_time_ms = now;
            desired_yaw_cd = ahrs_view->yaw_sensor;
        }
    }
    if (pilot_override) {
        if (now - last_pilot_override_warning > 1000) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: pilot overrides active");
            last_pilot_override_warning = now;
        }
    }
    if (zero_rp_input) {
        // pilot input on throttle and yaw will still use position hold if enabled
        get_poshold_attitude(target_roll_cd, target_pitch_cd, desired_yaw_cd);
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // if pilot override call attitude controller
    if (pilot_override || mode != TUNING) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll_cd, target_pitch_cd, target_yaw_rate_cds);
    } else {
        // somehow get attitude requests from autotuning
        control_attitude();
        // tell the user what's going on
        do_gcs_announcements();
    }

    // call position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cms);
    pos_control->update_z_controller();

}

// check if current is greater than maximum and update level_problem structure
bool AC_AutoTune::check_level(const LevelIssue issue, const float current, const float maximum)
{
    if (current > maximum) {
        level_problem.current = current;
        level_problem.maximum = maximum;
        level_problem.issue = issue;
        return false;
    }
    return true;
}

// return true if vehicle is close to level
bool AC_AutoTune::currently_level()
{
    float threshold_mul = 1.0;

    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - level_start_time_ms > AUTOTUNE_LEVEL_TIMEOUT_MS) {
        // after a long wait we use looser threshold, to allow tuning
        // with poor initial gains
        threshold_mul *= 2;
    }

    // display warning if vehicle fails to level
    if ((now_ms - level_start_time_ms > AUTOTUNE_LEVEL_WARNING_INTERVAL_MS) &&
        (now_ms - level_fail_warning_time_ms > AUTOTUNE_LEVEL_WARNING_INTERVAL_MS)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "AutoTune: failing to level, please tune manually");
        level_fail_warning_time_ms = now_ms;
    }

    if (!check_level(LevelIssue::ANGLE_ROLL,
                     fabsf(ahrs_view->roll_sensor - roll_cd),
                     threshold_mul*AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }

    if (!check_level(LevelIssue::ANGLE_PITCH,
                     fabsf(ahrs_view->pitch_sensor - pitch_cd),
                     threshold_mul*AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }
    if (!check_level(LevelIssue::ANGLE_YAW,
                     fabsf(wrap_180_cd(ahrs_view->yaw_sensor - desired_yaw_cd)),
                     threshold_mul*AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }
    if (!check_level(LevelIssue::RATE_ROLL,
                     (ToDeg(ahrs_view->get_gyro().x) * 100.0f),
                     threshold_mul*AUTOTUNE_LEVEL_RATE_RP_CD)) {
        return false;
    }
    if (!check_level(LevelIssue::RATE_PITCH,
                     (ToDeg(ahrs_view->get_gyro().y) * 100.0f),
                     threshold_mul*AUTOTUNE_LEVEL_RATE_RP_CD)) {
        return false;
    }
    if (!check_level(LevelIssue::RATE_YAW,
                     (ToDeg(ahrs_view->get_gyro().z) * 100.0f),
                     threshold_mul*AUTOTUNE_LEVEL_RATE_Y_CD)) {
        return false;
    }
    return true;
}

// main state machine to level vehicle, perform a test and update gains
// directly updates attitude controller with targets
void AC_AutoTune::control_attitude()
{
    rotation_rate = 0.0f;        // rotation rate in radians/second
    lean_angle = 0.0f;
    const float direction_sign = positive_direction ? 1.0f : -1.0f;
    const uint32_t now = AP_HAL::millis();

    // check tuning step
    switch (step) {

    case WAITING_FOR_LEVEL: {

        // Note: we should be using intra-test gains (which are very close to the original gains but have lower I)
        // re-enable rate limits
        attitude_control->use_sqrt_controller(true);

        get_poshold_attitude(roll_cd, pitch_cd, desired_yaw_cd);

        // hold level attitude
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_cd, pitch_cd, desired_yaw_cd, true);

        // hold the copter level for 0.5 seconds before we begin a twitch
        // reset counter if we are no longer level
        if (!currently_level()) {
            step_start_time_ms = now;
        }

        // if we have been level for a sufficient amount of time (0.5 seconds) move onto tuning step
        if (now - step_start_time_ms > AUTOTUNE_REQUIRED_LEVEL_TIME_MS) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Start Test");
            // initiate variables for next step
            step = TESTING;
            step_start_time_ms = now;
            step_time_limit_ms = AUTOTUNE_TESTING_STEP_TIMEOUT_MS;
            // set gains to their to-be-tested values
            twitch_first_iter = true;
            test_rate_max = 0.0f;
            test_rate_min = 0.0f;
            test_angle_max = 0.0f;
            test_angle_min = 0.0f;
            rotation_rate_filt.reset(0.0f);
            rate_max = 0.0f;
            load_gains(GAIN_TEST);
        } else {
            // when waiting for level we use the intra-test gains
            load_gains(GAIN_INTRA_TEST);
        }

        // Initialize test-specific variables
        switch (axis) {
        case ROLL:
            abort_angle = AUTOTUNE_TARGET_ANGLE_RLLPIT_CD;
            start_rate = ToDeg(ahrs_view->get_gyro().x) * 100.0f;
            start_angle = ahrs_view->roll_sensor;
            break;
        case PITCH:
            abort_angle = AUTOTUNE_TARGET_ANGLE_RLLPIT_CD;
            start_rate = ToDeg(ahrs_view->get_gyro().y) * 100.0f;
            start_angle = ahrs_view->pitch_sensor;
            break;
        case YAW:
            abort_angle = AUTOTUNE_TARGET_ANGLE_YAW_CD;
            start_rate = ToDeg(ahrs_view->get_gyro().z) * 100.0f;
            start_angle = ahrs_view->yaw_sensor;
            break;
        }

        // tests must be initialized last as some rely on variables above
        test_init();

        break;
    }

    case TESTING: {
        // Run the twitching step
        load_gains(GAIN_TEST);

        // run the test
        test_run(axis, direction_sign);

        // Check for failure causing reverse response
        if (lean_angle <= -AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD) {
            step = WAITING_FOR_LEVEL;
        }

        // protect from roll over
        if (ahrs_view->roll_sensor > 3000.0f || ahrs_view->roll_sensor < -3000.0f ||ahrs_view->pitch_sensor > 3000.0f || ahrs_view->pitch_sensor < -3000.0f) {
            step = WAITING_FOR_LEVEL;
        }

        // log this iterations lean angle and rotation rate
        Log_AutoTuneDetails();
        ahrs_view->Write_Rate(*motors, *attitude_control, *pos_control);
        log_pids();
        break;
    }

    case UPDATE_GAINS:

        // re-enable rate limits
        attitude_control->use_sqrt_controller(true);

        // log the latest gains
        Log_AutoTune();

        switch (tune_type) {
        // Check results after mini-step to increase rate D gain
        case RD_UP:
            updating_rate_d_up_all(axis);
            break;
        // Check results after mini-step to decrease rate D gain
        case RD_DOWN:
            updating_rate_d_down_all(axis);
            break;
        // Check results after mini-step to increase rate P gain
        case RP_UP:
            updating_rate_p_up_all(axis);
            break;
        // Check results after mini-step to increase stabilize P gain
        case SP_DOWN:
            updating_angle_p_down_all(axis);
            break;
        // Check results after mini-step to increase stabilize P gain
        case SP_UP:
            updating_angle_p_up_all(axis);
            break;
        case RFF_UP:
            updating_rate_ff_up_all(axis);
            break;
        case MAX_GAINS:
            updating_max_gains_all(axis);
            break;
        case RP_DOWN:
        case RFF_DOWN:
        case TUNE_COMPLETE:
            break;
        }

        // we've complete this step, finalize pids and move to next step
        if (counter >= AUTOTUNE_SUCCESS_COUNT) {

            // reset counter
            counter = 0;

            // reset scaling factor
            step_scaler = 1.0f;


            // move to the next tuning type
            switch (tune_type) {
            case RD_UP:
                break;
            case RD_DOWN:
                switch (axis) {
                case ROLL:
                    tune_roll_rd = MAX(min_d, tune_roll_rd * AUTOTUNE_RD_BACKOFF);
                    tune_roll_rp = MAX(get_rp_min(), tune_roll_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                case PITCH:
                    tune_pitch_rd = MAX(min_d, tune_pitch_rd * AUTOTUNE_RD_BACKOFF);
                    tune_pitch_rp = MAX(get_rp_min(), tune_pitch_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                case YAW:
                    tune_yaw_rLPF = MAX(get_yaw_rate_filt_min(), tune_yaw_rLPF * AUTOTUNE_RD_BACKOFF);
                    tune_yaw_rp = MAX(get_rp_min(), tune_yaw_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                }
                break;
            case RP_UP:
                switch (axis) {
                case ROLL:
                    tune_roll_rp = MAX(get_rp_min(), tune_roll_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                case PITCH:
                    tune_pitch_rp = MAX(get_rp_min(), tune_pitch_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                case YAW:
                    tune_yaw_rp = MAX(get_rp_min(), tune_yaw_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                }
                break;
            case SP_DOWN:
                break;
            case SP_UP:
                switch (axis) {
                case ROLL:
                    tune_roll_sp = MAX(get_sp_min(), tune_roll_sp * AUTOTUNE_SP_BACKOFF);
                    // trad heli uses original parameter value rather than max demostrated through test
                    if (set_accel_to_max_test_value()) {
                        tune_roll_accel = MAX(AUTOTUNE_RP_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    }
                    break;
                case PITCH:
                    tune_pitch_sp = MAX(get_sp_min(), tune_pitch_sp * AUTOTUNE_SP_BACKOFF);
                    // trad heli uses original parameter value rather than max demostrated through test
                    if (set_accel_to_max_test_value()) {
                        tune_pitch_accel = MAX(AUTOTUNE_RP_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    }
                    break;
                case YAW:
                    tune_yaw_sp = MAX(get_sp_min(), tune_yaw_sp * AUTOTUNE_SP_BACKOFF);
                    // trad heli uses original parameter value rather than max demostrated through test
                    if (set_accel_to_max_test_value()) {
                        tune_yaw_accel = MAX(AUTOTUNE_Y_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_Y_BACKOFF);
                    }
                    break;
                }
                break;
            case RP_DOWN:
            case RFF_UP:
            case RFF_DOWN:
            case MAX_GAINS:
            case TUNE_COMPLETE:
                break;
            }

            // increment the tune type to the next one in tune sequence
            tune_seq_curr++;
            tune_type = tune_seq[tune_seq_curr];

            if (tune_type == TUNE_COMPLETE) {
                // we've reached the end of a D-up-down PI-up-down tune type cycle
                tune_seq_curr = 0;
                tune_type = tune_seq[tune_seq_curr];

                // advance to the next axis
                bool complete = false;
                switch (axis) {
                case ROLL:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_ROLL;
                    if (pitch_enabled()) {
                        axis = PITCH;
                    } else if (yaw_enabled()) {
                        axis = YAW;
                    } else {
                        complete = true;
                    }
                    break;
                case PITCH:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_PITCH;
                    if (yaw_enabled()) {
                        axis = YAW;
                    } else {
                        complete = true;
                    }
                    break;
                case YAW:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_YAW;
                    complete = true;
                    break;
                }

                // if we've just completed all axes we have successfully completed the autotune
                // change to TESTING mode to allow user to fly with new gains
                if (complete) {
                    mode = SUCCESS;
                    update_gcs(AUTOTUNE_MESSAGE_SUCCESS);
                    AP::logger().Write_Event(LogEvent::AUTOTUNE_SUCCESS);
                    AP_Notify::events.autotune_complete = true;
                } else {
                    AP_Notify::events.autotune_next_axis = true;
                }
            }
        }

        // reverse direction for multicopter twitch test
        positive_direction = twitch_reverse_direction();

        if (axis == YAW) {
            attitude_control->input_euler_angle_roll_pitch_yaw(0.0f, 0.0f, ahrs_view->yaw_sensor, false);
        }

        // set gains to their intra-test values (which are very close to the original gains)
        load_gains(GAIN_INTRA_TEST);

        // reset testing step
        step = WAITING_FOR_LEVEL;
        step_start_time_ms = now;
        level_start_time_ms = step_start_time_ms;
        step_time_limit_ms = AUTOTUNE_REQUIRED_LEVEL_TIME_MS;
        break;
    }
}

// backup_gains_and_initialise - store current gains as originals
//  called before tuning starts to backup original gains
void AC_AutoTune::backup_gains_and_initialise()
{
    // initialise state because this is our first time
    if (roll_enabled()) {
        axis = ROLL;
    } else if (pitch_enabled()) {
        axis = PITCH;
    } else if (yaw_enabled()) {
        axis = YAW;
    }
    // no axes are complete
    axes_completed = 0;

    // set the tune sequence
    set_tune_sequence();

    // start at the beginning of tune sequence
    tune_seq_curr = 0;
    tune_type = tune_seq[tune_seq_curr];

    positive_direction = false;
    step = WAITING_FOR_LEVEL;
    step_start_time_ms = AP_HAL::millis();
    level_start_time_ms = step_start_time_ms;
    step_scaler = 1.0f;

    desired_yaw_cd = ahrs_view->yaw_sensor;

    aggressiveness = constrain_float(aggressiveness, 0.05f, 0.2f);

    orig_bf_feedforward = attitude_control->get_bf_feedforward();

    // backup original pids and initialise tuned pid values
    orig_roll_rp = attitude_control->get_rate_roll_pid().kP();
    orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
    orig_roll_rd = attitude_control->get_rate_roll_pid().kD();
    orig_roll_rff = attitude_control->get_rate_roll_pid().ff();
    orig_roll_fltt = attitude_control->get_rate_roll_pid().filt_T_hz();
    orig_roll_smax = attitude_control->get_rate_roll_pid().slew_limit();
    orig_roll_sp = attitude_control->get_angle_roll_p().kP();
    orig_roll_accel = attitude_control->get_accel_roll_max_cdss();
    tune_roll_rp = attitude_control->get_rate_roll_pid().kP();
    tune_roll_rd = attitude_control->get_rate_roll_pid().kD();
    tune_roll_rff = attitude_control->get_rate_roll_pid().ff();
    tune_roll_sp = attitude_control->get_angle_roll_p().kP();
    tune_roll_accel = attitude_control->get_accel_roll_max_cdss();

    orig_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
    orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
    orig_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
    orig_pitch_rff = attitude_control->get_rate_pitch_pid().ff();
    orig_pitch_fltt = attitude_control->get_rate_pitch_pid().filt_T_hz();
    orig_pitch_smax = attitude_control->get_rate_pitch_pid().slew_limit();
    orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
    orig_pitch_accel = attitude_control->get_accel_pitch_max_cdss();
    tune_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
    tune_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
    tune_pitch_rff = attitude_control->get_rate_pitch_pid().ff();
    tune_pitch_sp = attitude_control->get_angle_pitch_p().kP();
    tune_pitch_accel = attitude_control->get_accel_pitch_max_cdss();

    orig_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
    orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
    orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
    orig_yaw_rff = attitude_control->get_rate_yaw_pid().ff();
    orig_yaw_fltt = attitude_control->get_rate_yaw_pid().filt_T_hz();
    orig_yaw_smax = attitude_control->get_rate_yaw_pid().slew_limit();
    orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_E_hz();
    orig_yaw_accel = attitude_control->get_accel_yaw_max_cdss();
    orig_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    tune_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
    tune_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
    tune_yaw_rff = attitude_control->get_rate_yaw_pid().ff();
    tune_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_E_hz();
    tune_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    tune_yaw_accel = attitude_control->get_accel_yaw_max_cdss();

    AP::logger().Write_Event(LogEvent::AUTOTUNE_INITIALISED);
}

// load_orig_gains - set gains to their original values
//  called by stop and failed functions
void AC_AutoTune::load_orig_gains()
{
    attitude_control->bf_feedforward(orig_bf_feedforward);
    if (roll_enabled()) {
        if (!is_zero(orig_roll_rp) || allow_zero_rate_p()) {
            attitude_control->get_rate_roll_pid().kP(orig_roll_rp);
            attitude_control->get_rate_roll_pid().kI(orig_roll_ri);
            attitude_control->get_rate_roll_pid().kD(orig_roll_rd);
            attitude_control->get_rate_roll_pid().ff(orig_roll_rff);
            attitude_control->get_rate_roll_pid().filt_T_hz(orig_roll_fltt);
            attitude_control->get_rate_roll_pid().slew_limit(orig_roll_smax);
            attitude_control->get_angle_roll_p().kP(orig_roll_sp);
            attitude_control->set_accel_roll_max_cdss(orig_roll_accel);
        }
    }
    if (pitch_enabled()) {
        if (!is_zero(orig_pitch_rp) || allow_zero_rate_p()) {
            attitude_control->get_rate_pitch_pid().kP(orig_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(orig_pitch_ri);
            attitude_control->get_rate_pitch_pid().kD(orig_pitch_rd);
            attitude_control->get_rate_pitch_pid().ff(orig_pitch_rff);
            attitude_control->get_rate_pitch_pid().filt_T_hz(orig_pitch_fltt);
            attitude_control->get_rate_pitch_pid().slew_limit(orig_pitch_smax);
            attitude_control->get_angle_pitch_p().kP(orig_pitch_sp);
            attitude_control->set_accel_pitch_max_cdss(orig_pitch_accel);
        }
    }
    if (yaw_enabled()) {
        if (!is_zero(orig_yaw_rp)) {
            attitude_control->get_rate_yaw_pid().kP(orig_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(orig_yaw_ri);
            attitude_control->get_rate_yaw_pid().kD(orig_yaw_rd);
            attitude_control->get_rate_yaw_pid().ff(orig_yaw_rff);
            attitude_control->get_rate_yaw_pid().filt_E_hz(orig_yaw_rLPF);
            attitude_control->get_rate_yaw_pid().filt_T_hz(orig_yaw_fltt);
            attitude_control->get_rate_yaw_pid().slew_limit(orig_yaw_smax);
            attitude_control->get_angle_yaw_p().kP(orig_yaw_sp);
            attitude_control->set_accel_yaw_max_cdss(orig_yaw_accel);
        }
    }
}

// load_tuned_gains - load tuned gains
void AC_AutoTune::load_tuned_gains()
{
    if (!attitude_control->get_bf_feedforward()) {
        attitude_control->bf_feedforward(true);
        attitude_control->set_accel_roll_max_cdss(0.0f);
        attitude_control->set_accel_pitch_max_cdss(0.0f);
    }
    if (roll_enabled()) {
        if (!is_zero(tune_roll_rp) || allow_zero_rate_p()) {
            attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
            attitude_control->get_rate_roll_pid().kI(get_tuned_ri(axis));
            attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
            attitude_control->get_rate_roll_pid().ff(tune_roll_rff);
            attitude_control->get_angle_roll_p().kP(tune_roll_sp);
            attitude_control->set_accel_roll_max_cdss(tune_roll_accel);
        }
    }
    if (pitch_enabled()) {
        if (!is_zero(tune_pitch_rp) || allow_zero_rate_p()) {
            attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(get_tuned_ri(axis));
            attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
            attitude_control->get_rate_pitch_pid().ff(tune_pitch_rff);
            attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
            attitude_control->set_accel_pitch_max_cdss(tune_pitch_accel);
        }
    }
    if (yaw_enabled()) {
        if (!is_zero(tune_yaw_rp)) {
            attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(get_tuned_ri(axis));
            attitude_control->get_rate_yaw_pid().kD(get_tuned_yaw_rd());
            attitude_control->get_rate_yaw_pid().ff(tune_yaw_rff);
            attitude_control->get_rate_yaw_pid().filt_E_hz(tune_yaw_rLPF);
            attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
            attitude_control->set_accel_yaw_max_cdss(tune_yaw_accel);
        }
    }
}

// load_intra_test_gains - gains used between tests
//  called during testing mode's update-gains step to set gains ahead of return-to-level step
void AC_AutoTune::load_intra_test_gains()
{
    // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
    // sanity check the gains
    attitude_control->bf_feedforward(true);
    if (roll_enabled()) {
        attitude_control->get_rate_roll_pid().kP(orig_roll_rp);
        attitude_control->get_rate_roll_pid().kI(get_intra_test_ri(axis));
        attitude_control->get_rate_roll_pid().kD(orig_roll_rd);
        attitude_control->get_rate_roll_pid().ff(orig_roll_rff);
        attitude_control->get_rate_roll_pid().filt_T_hz(orig_roll_fltt);
        attitude_control->get_rate_roll_pid().slew_limit(orig_roll_smax);
        attitude_control->get_angle_roll_p().kP(orig_roll_sp);
        attitude_control->set_accel_roll_max_cdss(orig_roll_accel);
    }
    if (pitch_enabled()) {
        attitude_control->get_rate_pitch_pid().kP(orig_pitch_rp);
        attitude_control->get_rate_pitch_pid().kI(get_intra_test_ri(axis));
        attitude_control->get_rate_pitch_pid().kD(orig_pitch_rd);
        attitude_control->get_rate_pitch_pid().ff(orig_pitch_rff);
        attitude_control->get_rate_pitch_pid().filt_T_hz(orig_pitch_fltt);
        attitude_control->get_rate_pitch_pid().slew_limit(orig_pitch_smax);
        attitude_control->get_angle_pitch_p().kP(orig_pitch_sp);
        attitude_control->set_accel_pitch_max_cdss(orig_pitch_accel);
    }
    if (yaw_enabled()) {
        attitude_control->get_rate_yaw_pid().kP(orig_yaw_rp);
        attitude_control->get_rate_yaw_pid().kI(get_intra_test_ri(axis));
        attitude_control->get_rate_yaw_pid().kD(orig_yaw_rd);
        attitude_control->get_rate_yaw_pid().ff(orig_yaw_rff);
        attitude_control->get_rate_yaw_pid().filt_T_hz(orig_yaw_fltt);
        attitude_control->get_rate_yaw_pid().slew_limit(orig_yaw_smax);
        attitude_control->get_rate_yaw_pid().filt_E_hz(orig_yaw_rLPF);
        attitude_control->get_angle_yaw_p().kP(orig_yaw_sp);
        attitude_control->set_accel_yaw_max_cdss(orig_yaw_accel);
    }
}

/*
  load a specified set of gains
 */
void AC_AutoTune::load_gains(enum GainType gain_type)
{
    switch (gain_type) {
    case GAIN_ORIGINAL:
        load_orig_gains();
        break;
    case GAIN_INTRA_TEST:
        load_intra_test_gains();
        break;
    case GAIN_TEST:
        load_test_gains();
        break;
    case GAIN_TUNED:
        load_tuned_gains();
        break;
    }
}

// update_gcs - send message to ground station
void AC_AutoTune::update_gcs(uint8_t message_id) const
{
    switch (message_id) {
    case AUTOTUNE_MESSAGE_STARTED:
        gcs().send_text(MAV_SEVERITY_INFO,"AutoTune: Started");
        break;
    case AUTOTUNE_MESSAGE_STOPPED:
        gcs().send_text(MAV_SEVERITY_INFO,"AutoTune: Stopped");
        break;
    case AUTOTUNE_MESSAGE_SUCCESS:
        gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: Success");
        break;
    case AUTOTUNE_MESSAGE_FAILED:
        gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: Failed");
        break;
    case AUTOTUNE_MESSAGE_TESTING:
        gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: Pilot Testing");
        break;
    case AUTOTUNE_MESSAGE_SAVED_GAINS:
        gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: Saved gains for %s%s%s",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_ROLL)?"Roll ":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_PITCH)?"Pitch ":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_YAW)?"Yaw":"");
        break;
    }
}

// axis helper functions
bool AC_AutoTune::roll_enabled() const
{
    return axis_bitmask & AUTOTUNE_AXIS_BITMASK_ROLL;
}

bool AC_AutoTune::pitch_enabled() const
{
    return axis_bitmask & AUTOTUNE_AXIS_BITMASK_PITCH;
}

bool AC_AutoTune::yaw_enabled() const
{
    return axis_bitmask & AUTOTUNE_AXIS_BITMASK_YAW;
}

/*
  check if we have a good position estimate
 */
bool AC_AutoTune::position_ok(void)
{
    if (!AP::ahrs().have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav->get_filter_status();

    // require a good absolute position and EKF must not be in const_pos_mode
    return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
}

// get attitude for slow position hold in autotune mode
void AC_AutoTune::get_poshold_attitude(float &roll_cd_out, float &pitch_cd_out, float &yaw_cd_out)
{
    roll_cd_out = pitch_cd_out = 0;

    if (!use_poshold) {
        // we are not trying to hold position
        return;
    }

    // do we know where we are? If not then don't do poshold
    if (!position_ok()) {
        return;
    }

    if (!have_position) {
        have_position = true;
        start_position = inertial_nav->get_position_neu_cm();
    }

    // don't go past 10 degrees, as autotune result would deteriorate too much
    const float angle_max_cd = 1000;

    // hit the 10 degree limit at 20 meters position error
    const float dist_limit_cm = 2000;

    // we only start adjusting yaw if we are more than 5m from the
    // target position. That corresponds to a lean angle of 2.5 degrees
    const float yaw_dist_limit_cm = 500;

    Vector3f pdiff = inertial_nav->get_position_neu_cm() - start_position;
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
    pitch_cd_out = angle_ne.x * ahrs_view->cos_yaw() + angle_ne.y * ahrs_view->sin_yaw();
    roll_cd_out  = angle_ne.x * ahrs_view->sin_yaw() - angle_ne.y * ahrs_view->cos_yaw();

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
