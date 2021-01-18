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
#define AUTOTUNE_Y_FILT_FREQ              10.0f     // Autotune filter frequency when testing Yaw
#define AUTOTUNE_RD_BACKOFF                1.0f     // Rate D gains are reduced to 50% of their maximum value discovered during tuning
#define AUTOTUNE_RP_BACKOFF                1.0f     // Rate P gains are reduced to 97.5% of their maximum value discovered during tuning
#define AUTOTUNE_ACCEL_RP_BACKOFF          1.0f     // back off from maximum acceleration
#define AUTOTUNE_ACCEL_Y_BACKOFF           1.0f     // back off from maximum acceleration

// roll and pitch axes
#define AUTOTUNE_TARGET_ANGLE_RLLPIT_CD     2000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_RATE_RLLPIT_CDS     18000   // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD 1000    // minimum target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS 4500    // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step

// yaw axis
#define AUTOTUNE_TARGET_ANGLE_YAW_CD        3000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_RATE_YAW_CDS        9000    // target yaw rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_ANGLE_YAW_CD     500    // minimum target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_MIN_RATE_YAW_CDS    1500    // minimum target yaw rate during AUTOTUNE_STEP_TWITCHING step

// ifdef is not working.  Modified multi values to reflect heli requirements
#ifdef HELI_BUILD
// heli defines
#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS   5000U    // timeout for tuning mode's testing step
#define AUTOTUNE_RP_ACCEL_MIN           20000.0f     // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_Y_ACCEL_MIN            10000.0f     // Minimum acceleration for Yaw
#define AUTOTUNE_SP_BACKOFF                1.0f     // Stab P gains are reduced to 90% of their maximum value discovered during tuning
#else
// Frame specific defaults
#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS   5000U    // timeout for tuning mode's testing step
#define AUTOTUNE_RP_ACCEL_MIN           20000.0f     // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_Y_ACCEL_MIN            10000.0f     // Minimum acceleration for Yaw
#define AUTOTUNE_SP_BACKOFF                1.0f     // Stab P gains are reduced to 90% of their maximum value discovered during tuning
#endif // HELI_BUILD

// second table of user settable parameters for quadplanes, this
// allows us to go beyond the 64 parameter limit
const AP_Param::GroupInfo AC_AutoTune::var_info[] = {
    // @Param: AXES
    // @DisplayName: Autotune axis bitmask
    // @Description: 1-byte bitmap of axes to autotune
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw
    // @User: Standard
    AP_GROUPINFO("AXES", 1, AC_AutoTune, axis_bitmask,  7),  // AUTOTUNE_AXIS_BITMASK_DEFAULT

// Indices 2 and 3 where AGGR and MIN_D.  These were moved to the Multi SubClass

    AP_GROUPEND
};

AC_AutoTune::AC_AutoTune()
{
    AP_Param::setup_object_defaults(this, var_info);
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
        if (!is_equal(start_freq,stop_freq)) {
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

    bool zero_rp_input = is_zero(target_roll_cd) && is_zero(target_pitch_cd);
    // allow pilots to make inputs less than 5 deg in pitch and roll
    if (allow_pilot_rp_input() && !pilot_override && fabsf(target_roll_cd) < 500 && fabsf(target_pitch_cd) < 500) {
        zero_rp_input = true;
    }

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
    if (zero_rp_input && !allow_pilot_rp_input()) {
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

        // Initialize test specific variables
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
        attitude_control->set_accel_roll_max(orig_roll_accel);
    }
    if (pitch_enabled()) {
        attitude_control->get_rate_pitch_pid().kP(orig_pitch_rp);
        attitude_control->get_rate_pitch_pid().kI(get_intra_test_ri(axis));
        attitude_control->get_rate_pitch_pid().kD(orig_pitch_rd);
        attitude_control->get_rate_pitch_pid().ff(orig_pitch_rff);
        attitude_control->get_rate_pitch_pid().filt_T_hz(orig_pitch_fltt);
        attitude_control->get_rate_pitch_pid().slew_limit(orig_pitch_smax);
        attitude_control->get_angle_pitch_p().kP(orig_pitch_sp);
        attitude_control->set_accel_pitch_max(orig_pitch_accel);
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
        attitude_control->set_accel_yaw_max(orig_yaw_accel);
    }
}


// load_test_gains - load the to-be-tested gains for a single axis
// called by control_attitude() just before it beings testing a gain (i.e. just before it twitches)
void AC_AutoTune::load_test_gains()
{
    switch (axis) {
    case ROLL:
        if (tune_type == MAX_GAINS && !is_zero(tune_roll_rff)) {
            attitude_control->get_rate_roll_pid().kP(0.0f);
            attitude_control->get_rate_roll_pid().kD(0.0f);
        } else {
            attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
            attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
        }
        attitude_control->get_angle_roll_p().kP(tune_roll_sp);
        break;
    case PITCH:
        if (tune_type == MAX_GAINS && !is_zero(tune_pitch_rff)) {
            attitude_control->get_rate_pitch_pid().kP(0.0f);
            attitude_control->get_rate_pitch_pid().kD(0.0f);
        } else {
            attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
            attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
        }
        attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
        break;
    case YAW:
        attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
        attitude_control->get_rate_yaw_pid().filt_E_hz(tune_yaw_rLPF);
        attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
        break;
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

// save_tuning_gains - save the final tuned gains for each axis
// save discovered gains to eeprom if autotuner is enabled (i.e. switch is in the high position)
void AC_AutoTune::save_tuning_gains()
{
    // see if we successfully completed tuning of at least one axis
    if (axes_completed == 0) {
        return;
    }

    if (!attitude_control->get_bf_feedforward()) {
        attitude_control->bf_feedforward_save(true);
        attitude_control->save_accel_roll_max_cdss(0.0f);
        attitude_control->save_accel_pitch_max_cdss(0.0f);
    }

    // sanity check the rate P values
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL) && roll_enabled() && (!is_zero(tune_roll_rp) || allow_zero_rate_p())) {
        // rate roll gains
        attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
        attitude_control->get_rate_roll_pid().kD(tune_roll_rd);

        // stabilize roll
        attitude_control->get_angle_roll_p().kP(tune_roll_sp);
        attitude_control->get_angle_roll_p().save_gains();

        // acceleration roll
        attitude_control->save_accel_roll_max_cdss(tune_roll_accel);

        // resave pids to originals in case the autotune is run again
        orig_roll_rp = attitude_control->get_rate_roll_pid().kP();
        orig_roll_rd = attitude_control->get_rate_roll_pid().kD();
        orig_roll_sp = attitude_control->get_angle_roll_p().kP();
        orig_roll_accel = attitude_control->get_accel_roll_max_cdss();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) && pitch_enabled() && (!is_zero(tune_pitch_rp) || allow_zero_rate_p())) {
        // rate pitch gains
        attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
        attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);

        // stabilize pitch
        attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
        attitude_control->get_angle_pitch_p().save_gains();

        // acceleration pitch
        attitude_control->save_accel_pitch_max_cdss(tune_pitch_accel);

        // resave pids to originals in case the autotune is run again
        orig_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
        orig_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
        orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
        orig_pitch_accel = attitude_control->get_accel_pitch_max_cdss();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_YAW) && yaw_enabled() && !is_zero(tune_yaw_rp)) {
        // rate yaw gains
        attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);

        // stabilize yaw
        attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
        attitude_control->get_angle_yaw_p().save_gains();

        // acceleration yaw
        attitude_control->save_accel_yaw_max_cdss(tune_yaw_accel);

        // resave pids to originals in case the autotune is run again
        orig_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
        orig_yaw_sp = attitude_control->get_angle_yaw_p().kP();
        orig_yaw_accel = attitude_control->get_accel_yaw_max_cdss();
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

// twitching_test_rate - twitching tests
// update min and max and test for end conditions
void AC_AutoTune::twitching_test_rate(float rate, float rate_target_max, float &meas_rate_min, float &meas_rate_max)
{
    const uint32_t now = AP_HAL::millis();

    // capture maximum rate
    if (rate > meas_rate_max) {
        // the measurement is continuing to increase without stopping
        meas_rate_max = rate;
        meas_rate_min = rate;
    }

    // capture minimum measurement after the measurement has peaked (aka "bounce back")
    if ((rate < meas_rate_min) && (meas_rate_max > rate_target_max * 0.5f)) {
        // the measurement is bouncing back
        meas_rate_min = rate;
    }

    // calculate early stopping time based on the time it takes to get to 75%
    if (meas_rate_max < rate_target_max * 0.75f) {
        // the measurement not reached the 75% threshold yet
        step_time_limit_ms = (now - step_start_time_ms) * 3;
        step_time_limit_ms = MIN(step_time_limit_ms, AUTOTUNE_TESTING_STEP_TIMEOUT_MS);
    }

    if (meas_rate_max > rate_target_max) {
        // the measured rate has passed the maximum target rate
        step = UPDATE_GAINS;
    }

    if (meas_rate_max-meas_rate_min > meas_rate_max*aggressiveness) {
        // the measurement has passed 50% of the maximum rate and bounce back is larger than the threshold
        step = UPDATE_GAINS;
    }

    if (now - step_start_time_ms >= step_time_limit_ms) {
        // we have passed the maximum stop time
        step = UPDATE_GAINS;
    }
}

// twitching_test_rate - twitching tests
// update min and max and test for end conditions
void AC_AutoTune::twitching_abort_rate(float angle, float rate, float angle_max, float meas_rate_min)
{
    if (angle >= angle_max) {
        if (is_equal(rate, meas_rate_min) && step_scaler > 0.5f) {
            // we have reached the angle limit before completing the measurement of maximum and minimum
            // reduce the maximum target rate
            step_scaler *= 0.9f;
            // ignore result and start test again
            step = WAITING_FOR_LEVEL;
        } else {
            step = UPDATE_GAINS;
        }
    }
}

// twitching_test_angle - twitching tests
// update min and max and test for end conditions
void AC_AutoTune::twitching_test_angle(float angle, float rate, float angle_target_max, float &meas_angle_min, float &meas_angle_max, float &meas_rate_min, float &meas_rate_max)
{
    const uint32_t now = AP_HAL::millis();

    // capture maximum angle
    if (angle > meas_angle_max) {
        // the angle still increasing
        meas_angle_max = angle;
        meas_angle_min = angle;
    }

    // capture minimum angle after we have reached a reasonable maximum angle
    if ((angle < meas_angle_min) && (meas_angle_max > angle_target_max * 0.5f)) {
        // the measurement is bouncing back
        meas_angle_min = angle;
    }

    // capture maximum rate
    if (rate > meas_rate_max) {
        // the measurement is still increasing
        meas_rate_max = rate;
        meas_rate_min = rate;
    }

    // capture minimum rate after we have reached maximum rate
    if (rate < meas_rate_min) {
        // the measurement is still decreasing
        meas_rate_min = rate;
    }

    // calculate early stopping time based on the time it takes to get to 75%
    if (meas_angle_max < angle_target_max * 0.75f) {
        // the measurement not reached the 75% threshold yet
        step_time_limit_ms = (now - step_start_time_ms) * 3;
        step_time_limit_ms = MIN(step_time_limit_ms, AUTOTUNE_TESTING_STEP_TIMEOUT_MS);
    }

    if (meas_angle_max > angle_target_max) {
        // the measurement has passed the maximum angle
        step = UPDATE_GAINS;
    }

    if (meas_angle_max-meas_angle_min > meas_angle_max*aggressiveness) {
        // the measurement has passed 50% of the maximum angle and bounce back is larger than the threshold
        step = UPDATE_GAINS;
    }

    if (now - step_start_time_ms >= step_time_limit_ms) {
        // we have passed the maximum stop time
        step = UPDATE_GAINS;
    }
}

// twitching_measure_acceleration - measure rate of change of measurement
void AC_AutoTune::twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max) const
{
    if (rate_measurement_max < rate_measurement) {
        rate_measurement_max = rate_measurement;
        rate_of_change = (1000.0f*rate_measurement_max)/(AP_HAL::millis() - step_start_time_ms);
    }
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

void AC_AutoTune::twitch_test_init()
{
    float target_max_rate;
    switch (axis) {
    case ROLL: {
        target_max_rate = MAX(AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, step_scaler*AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
        target_rate = constrain_float(ToDeg(attitude_control->max_rate_step_bf_roll())*100.0f, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, target_max_rate);
        target_angle = constrain_float(ToDeg(attitude_control->max_angle_step_bf_roll())*100.0f, AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD, AUTOTUNE_TARGET_ANGLE_RLLPIT_CD);
        rotation_rate_filt.set_cutoff_frequency(attitude_control->get_rate_roll_pid().filt_D_hz()*2.0f);
        break;
    }
    case PITCH: {
        target_max_rate = MAX(AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, step_scaler*AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
        target_rate = constrain_float(ToDeg(attitude_control->max_rate_step_bf_pitch())*100.0f, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, target_max_rate);
        target_angle = constrain_float(ToDeg(attitude_control->max_angle_step_bf_pitch())*100.0f, AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD, AUTOTUNE_TARGET_ANGLE_RLLPIT_CD);
        rotation_rate_filt.set_cutoff_frequency(attitude_control->get_rate_pitch_pid().filt_D_hz()*2.0f);
        break;
    }
    case YAW: {
        target_max_rate = MAX(AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, step_scaler*AUTOTUNE_TARGET_RATE_YAW_CDS);
        target_rate = constrain_float(ToDeg(attitude_control->max_rate_step_bf_yaw()*0.75f)*100.0f, AUTOTUNE_TARGET_MIN_RATE_YAW_CDS, target_max_rate);
        target_angle = constrain_float(ToDeg(attitude_control->max_angle_step_bf_yaw()*0.75f)*100.0f, AUTOTUNE_TARGET_MIN_ANGLE_YAW_CD, AUTOTUNE_TARGET_ANGLE_YAW_CD);
        rotation_rate_filt.set_cutoff_frequency(AUTOTUNE_Y_FILT_FREQ);
        break;
    }
    }

    if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
        rotation_rate_filt.reset(start_rate);
    } else {
        rotation_rate_filt.reset(0);
    }

}

//run twitch test
void AC_AutoTune::twitch_test_run(AxisType test_axis, const float dir_sign)
{
    // disable rate limits
    attitude_control->use_sqrt_controller(false);
    // hold current attitude
    attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);

    if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
        // step angle targets on first iteration
        if (twitch_first_iter) {
            twitch_first_iter = false;
            // Testing increasing stabilize P gain so will set lean angle target
            switch (test_axis) {
            case ROLL:
                // request roll to 20deg
                attitude_control->input_angle_step_bf_roll_pitch_yaw(dir_sign * target_angle, 0.0f, 0.0f);
                break;
            case PITCH:
                // request pitch to 20deg
                attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f, dir_sign * target_angle, 0.0f);
                break;
            case YAW:
                // request pitch to 20deg
                attitude_control->input_angle_step_bf_roll_pitch_yaw(0.0f, 0.0f, dir_sign * target_angle);
                break;
            }
        }
    } else {
        // Testing rate P and D gains so will set body-frame rate targets.
        // Rate controller will use existing body-frame rates and convert to motor outputs
        // for all axes except the one we override here.
        switch (test_axis) {
        case ROLL:
            // override body-frame roll rate
            attitude_control->rate_bf_roll_target(dir_sign * target_rate + start_rate);
            break;
        case PITCH:
            // override body-frame pitch rate
            attitude_control->rate_bf_pitch_target(dir_sign * target_rate + start_rate);
            break;
        case YAW:
            // override body-frame yaw rate
            attitude_control->rate_bf_yaw_target(dir_sign * target_rate + start_rate);
            break;
        }
    }

    // capture this iterations rotation rate and lean angle
    float gyro_reading = 0;
    switch (test_axis) {
    case ROLL:
        gyro_reading = ahrs_view->get_gyro().x;
        lean_angle = dir_sign * (ahrs_view->roll_sensor - (int32_t)start_angle);
        break;
    case PITCH:
        gyro_reading = ahrs_view->get_gyro().y;
        lean_angle = dir_sign * (ahrs_view->pitch_sensor - (int32_t)start_angle);
        break;
    case YAW:
        gyro_reading = ahrs_view->get_gyro().z;
        lean_angle = dir_sign * wrap_180_cd(ahrs_view->yaw_sensor-(int32_t)start_angle);
        break;
    }

    // Add filter to measurements
    float filter_value;
    switch (tune_type) {
    case SP_DOWN:
    case SP_UP:
        filter_value = dir_sign * (ToDeg(gyro_reading) * 100.0f);
        break;
    default:
        filter_value = dir_sign * (ToDeg(gyro_reading) * 100.0f - start_rate);
        break;
    }
    rotation_rate = rotation_rate_filt.apply(filter_value,
                    AP::scheduler().get_loop_period_s());

    switch (tune_type) {
    case RD_UP:
    case RD_DOWN:
        twitching_test_rate(rotation_rate, target_rate, test_rate_min, test_rate_max);
        twitching_measure_acceleration(test_accel_max, rotation_rate, rate_max);
        twitching_abort_rate(lean_angle, rotation_rate, abort_angle, test_rate_min);
        break;
    case RP_UP:
        twitching_test_rate(rotation_rate, target_rate*(1+0.5f*aggressiveness), test_rate_min, test_rate_max);
        twitching_measure_acceleration(test_accel_max, rotation_rate, rate_max);
        twitching_abort_rate(lean_angle, rotation_rate, abort_angle, test_rate_min);
        break;
    case SP_DOWN:
    case SP_UP:
        twitching_test_angle(lean_angle, rotation_rate, target_angle*(1+0.5f*aggressiveness), test_angle_min, test_angle_max, test_rate_min, test_rate_max);
        twitching_measure_acceleration(test_accel_max, rotation_rate - dir_sign * start_rate, rate_max);
        break;
    default:
        break;
    }
}

void AC_AutoTune::rate_ff_test_init()
{
    ff_test_phase = 0;
    rotation_rate_filt.reset(0);
    rotation_rate_filt.set_cutoff_frequency(5.0f);
    command_filt.reset(0);
    command_filt.set_cutoff_frequency(5.0f);
    target_rate_filt.reset(0);
    target_rate_filt.set_cutoff_frequency(5.0f);
    test_command_filt = 0.0f;
    test_rate_filt = 0.0f;
    test_tgt_rate_filt = 0.0f;
    filt_target_rate = 0.0f;
    settle_time = 200;
    phase_out_time = 500;
}

void AC_AutoTune::rate_ff_test_run(float max_angle_cd, float target_rate_cds, float dir_sign)
{
    float gyro_reading = 0.0f;
    float command_reading = 0.0f;
    float tgt_rate_reading = 0.0f;
    const uint32_t now = AP_HAL::millis();
    static float trim_command_reading = 0.0f;
    static float trim_heading = 0.0f;
    static float rate_request_cds;
    static float angle_request_cd;

    // TODO make filter leak dependent on dt
    const float filt_alpha = 0.0123f;

    target_rate_cds = dir_sign * target_rate_cds;

    switch (axis) {
    case ROLL:
        gyro_reading = ahrs_view->get_gyro().x;
        command_reading = motors->get_roll();
        tgt_rate_reading = attitude_control->rate_bf_targets().x;
        if (settle_time > 0) {
            settle_time--;
            trim_command_reading = motors->get_roll();
            rate_request_cds = gyro_reading;
        } else if (((ahrs_view->roll_sensor <= max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->roll_sensor >= -max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 0) {
            rate_request_cds += (target_rate_cds - rate_request_cds) * filt_alpha;
            attitude_control->input_rate_bf_roll_pitch_yaw(rate_request_cds, 0.0f, 0.0f);
        } else if (((ahrs_view->roll_sensor > max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->roll_sensor < -max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 0) {
            ff_test_phase = 1;
            rate_request_cds += (-target_rate_cds - rate_request_cds) * filt_alpha;
            attitude_control->input_rate_bf_roll_pitch_yaw(rate_request_cds, 0.0f, 0.0f);
            attitude_control->rate_bf_roll_target(rate_request_cds);
        } else if (((ahrs_view->roll_sensor >= -max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->roll_sensor <= max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 1 ) {
            rate_request_cds += (-target_rate_cds - rate_request_cds) * filt_alpha;
            attitude_control->input_rate_bf_roll_pitch_yaw(rate_request_cds, 0.0f, 0.0f);
            attitude_control->rate_bf_roll_target(rate_request_cds);
        } else if (((ahrs_view->roll_sensor < -max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->roll_sensor > max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 1 ) {
            ff_test_phase = 2;
            angle_request_cd = attitude_control->get_att_target_euler_cd().x;
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(angle_request_cd, start_angles.y, 0.0f);
        } else if (ff_test_phase == 2 ) {
            angle_request_cd += (start_angles.x - angle_request_cd) * filt_alpha;
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(angle_request_cd, start_angles.y, 0.0f);
            phase_out_time--;
        }
        break;
    case PITCH:
        gyro_reading = ahrs_view->get_gyro().y;
        command_reading = motors->get_pitch();
        tgt_rate_reading = attitude_control->rate_bf_targets().y;
        if (settle_time > 0) {
            settle_time--;
            trim_command_reading = motors->get_pitch();
            rate_request_cds = gyro_reading;
        } else if (((ahrs_view->pitch_sensor <= max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->pitch_sensor >= -max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 0) {
            rate_request_cds += (target_rate_cds - rate_request_cds) * filt_alpha;
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, rate_request_cds, 0.0f);
        } else if (((ahrs_view->pitch_sensor > max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->pitch_sensor < -max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 0) {
            ff_test_phase = 1;
            rate_request_cds += (-target_rate_cds - rate_request_cds) * filt_alpha;
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, rate_request_cds, 0.0f);
            attitude_control->rate_bf_pitch_target(rate_request_cds);
        } else if (((ahrs_view->pitch_sensor >= -max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->pitch_sensor <= max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 1 ) {
            rate_request_cds += (-target_rate_cds - rate_request_cds) * filt_alpha;
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, rate_request_cds, 0.0f);
            attitude_control->rate_bf_pitch_target(rate_request_cds);
        } else if (((ahrs_view->pitch_sensor < -max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->pitch_sensor > max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 1 ) {
            ff_test_phase = 2;
            angle_request_cd = attitude_control->get_att_target_euler_cd().y;
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, angle_request_cd, 0.0f);
        } else if (ff_test_phase == 2 ) {
            angle_request_cd += (start_angles.x - angle_request_cd) * filt_alpha;
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, angle_request_cd, 0.0f);
            phase_out_time--;
        }
        break;
    case YAW:
        gyro_reading = ahrs_view->get_gyro().z;
        command_reading = motors->get_yaw();
        tgt_rate_reading = attitude_control->rate_bf_targets().z;
        if (settle_time > 0) {
            settle_time--;
            trim_command_reading = motors->get_yaw();
            trim_heading = ahrs_view->yaw_sensor;
        } else if (((wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) <= 2.0f * max_angle_cd && is_positive(dir_sign))
                   || (wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) >= -2.0f * max_angle_cd && !is_positive(dir_sign)))
                   && ff_test_phase == 0) {
            rate_request_cds += (target_rate_cds - rate_request_cds) * filt_alpha;
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, rate_request_cds);
        } else if (((wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) > 2.0f * max_angle_cd && is_positive(dir_sign))
                   || (wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) < -2.0f * max_angle_cd && !is_positive(dir_sign)))
                   && ff_test_phase == 0) {
            ff_test_phase = 1;
            rate_request_cds += (-target_rate_cds - rate_request_cds) * filt_alpha;
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, rate_request_cds);
            attitude_control->rate_bf_yaw_target(rate_request_cds);
        } else if (((wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) >= -2.0f * max_angle_cd && is_positive(dir_sign))
                   || (wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) <= 2.0f * max_angle_cd && !is_positive(dir_sign)))
                   && ff_test_phase == 1 ) {
            rate_request_cds += (-target_rate_cds - rate_request_cds) * filt_alpha;
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, rate_request_cds);
            attitude_control->rate_bf_yaw_target(rate_request_cds);
        } else if (((wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) < -2.0f * max_angle_cd && is_positive(dir_sign))
                   || (wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) > 2.0f * max_angle_cd && !is_positive(dir_sign)))
                   && ff_test_phase == 1 ) {
            ff_test_phase = 2;
            angle_request_cd = attitude_control->get_att_target_euler_cd().z;
            attitude_control->input_euler_angle_roll_pitch_yaw(start_angles.x, start_angles.y, angle_request_cd, false);
        } else if (ff_test_phase == 2 ) {
            angle_request_cd += wrap_180_cd(trim_heading - angle_request_cd) * filt_alpha;
            attitude_control->input_euler_angle_roll_pitch_yaw(start_angles.x, start_angles.y, angle_request_cd, false);
        }
        break;
    }

    rotation_rate = rotation_rate_filt.apply(gyro_reading,
                    AP::scheduler().get_loop_period_s());
    command_out = command_filt.apply((command_reading - trim_command_reading),
                                     AP::scheduler().get_loop_period_s());
    filt_target_rate = target_rate_filt.apply(tgt_rate_reading,
                       AP::scheduler().get_loop_period_s());

    // record steady state rate and motor command
    switch (axis) {
    case ROLL:
        if (((ahrs_view->roll_sensor >= -max_angle_cd + start_angle && is_positive(dir_sign))
            || (ahrs_view->roll_sensor <= max_angle_cd + start_angle && !is_positive(dir_sign)))
            && ff_test_phase == 1 ) {
            test_rate_filt = rotation_rate;
            test_command_filt = command_out;
            test_tgt_rate_filt = filt_target_rate;
        }
        break;
    case PITCH:
        if (((ahrs_view->pitch_sensor >= -max_angle_cd + start_angle && is_positive(dir_sign))
            || (ahrs_view->pitch_sensor <= max_angle_cd + start_angle && !is_positive(dir_sign)))
            && ff_test_phase == 1 ) {
            test_rate_filt = rotation_rate;
            test_command_filt = command_out;
            test_tgt_rate_filt = filt_target_rate;
        }
        break;
    case YAW:
        if (((wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) >= -2.0f * max_angle_cd && is_positive(dir_sign))
            || (wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) <= 2.0f * max_angle_cd && !is_positive(dir_sign)))
            && ff_test_phase == 1 ) {
            test_rate_filt = rotation_rate;
            test_command_filt = command_out;
            test_tgt_rate_filt = filt_target_rate;
        }
        break;
    }
    if (now - step_start_time_ms >= step_time_limit_ms || (ff_test_phase == 2 && phase_out_time == 0)) {
        // we have passed the maximum stop time
        step = UPDATE_GAINS;
        rate_request_cds = 0.0f;
        angle_request_cd = 0.0f;
    }

}

void AC_AutoTune::dwell_test_init(float filt_freq)
{
    rotation_rate_filt.reset(0);
    rotation_rate_filt.set_cutoff_frequency(filt_freq);
    command_filt.reset(0);
    command_filt.set_cutoff_frequency(filt_freq);
    target_rate_filt.reset(0);
    target_rate_filt.set_cutoff_frequency(filt_freq);
    test_command_filt = 0.0f;
    test_rate_filt = 0.0f;
    test_tgt_rate_filt = 0.0f;
    filt_target_rate = 0.0f;
    dwell_start_time_ms = 0.0f;
    settle_time = 200;
    if (!is_equal(start_freq,stop_freq)) {
        sweep.ph180_freq = 0.0f;
        sweep.ph180_gain = 0.0f;
        sweep.ph180_phase = 0.0f;
        sweep.ph270_freq = 0.0f;
        sweep.ph270_gain = 0.0f;
        sweep.ph270_phase = 0.0f;
        sweep.maxgain_gain = 0.0f;
        sweep.maxgain_freq = 0.0f;
        sweep.maxgain_phase = 0.0f;
        sweep.progress = 0;
        curr_test_gain = 0.0f;
        curr_test_phase = 0.0f;
    }

    // save the trim output from PID controller
    float ff_term = 0.0f;
    float p_term = 0.0f;
    switch (axis) {
    case ROLL:
        trim_meas_rate = ahrs_view->get_gyro().x;
        ff_term = attitude_control->get_rate_roll_pid().get_ff();
        p_term = attitude_control->get_rate_roll_pid().get_p();
        break;
    case PITCH:
        trim_meas_rate = ahrs_view->get_gyro().y;
        ff_term = attitude_control->get_rate_pitch_pid().get_ff();
        p_term = attitude_control->get_rate_pitch_pid().get_p();
        break;
    case YAW:
        trim_meas_rate = ahrs_view->get_gyro().z;
        ff_term = attitude_control->get_rate_yaw_pid().get_ff();
        p_term = attitude_control->get_rate_yaw_pid().get_p();
        break;
    }
    trim_pff_out = ff_term + p_term;
}

void AC_AutoTune::dwell_test_run(uint8_t freq_resp_input, float start_frq, float stop_frq, float &dwell_gain, float &dwell_phase)
{
    float gyro_reading = 0.0f;
    float command_reading = 0.0f;
    float tgt_rate_reading = 0.0f;
    float tgt_attitude = 2.5f * 0.01745f;
    const uint32_t now = AP_HAL::millis();
    float target_rate_cds;
    static float trim_command;
    static Vector3f trim_attitude_cd;
    float sweep_time_ms = 23000;
    const float att_hold_gain = 4.5f;
    static Vector3f filt_attitude_cd;
    Vector3f attitude_cd;
    static float filt_command_reading;
    static float filt_gyro_reading;
    static float filt_tgt_rate_reading;
    const float vel_hold_gain = 0.04f;

    float dwell_freq = start_frq;
    float cycle_time_ms = 0;
    if (!is_zero(dwell_freq)) {
        cycle_time_ms = 1000.0f * 2.0f * M_PI / dwell_freq;
    }

    const float alpha = calc_lowpass_alpha_dt(0.0025f, 0.2f * start_frq);
    attitude_cd = Vector3f((float)ahrs_view->roll_sensor, (float)ahrs_view->pitch_sensor, (float)ahrs_view->yaw_sensor);
    Vector3f velocity_ned, velocity_bf;
    if (ahrs_view->get_velocity_NED(velocity_ned)) {
        velocity_bf.x = velocity_ned.x * ahrs_view->cos_yaw() + velocity_ned.y * ahrs_view->sin_yaw();
        velocity_bf.y = velocity_ned.x * ahrs_view->sin_yaw() + velocity_ned.y * ahrs_view->cos_yaw();
    }

    // keep controller from requesting too high of a rate
    float target_rate_mag_cds = dwell_freq * tgt_attitude * 5730.0f;
    if (target_rate_mag_cds > 5000.0f) {
        target_rate_mag_cds = 5000.0f;
    }
    if (settle_time == 0) {
        // give gentler start for the dwell
        if ((float)(now - dwell_start_time_ms) < 0.5f * cycle_time_ms) {
            target_rate_cds = -0.5f * target_rate_mag_cds * sinf(dwell_freq * (now - dwell_start_time_ms) * 0.001);
        } else {
            if (is_equal(start_frq,stop_frq)) {
                target_rate_cds = - target_rate_mag_cds * cosf(dwell_freq * (now - dwell_start_time_ms - 0.25f * cycle_time_ms) * 0.001);
            } else {
                target_rate_cds = waveform((now - dwell_start_time_ms - 0.5f * cycle_time_ms) * 0.001, (sweep_time_ms - 0.5f * cycle_time_ms) * 0.001f, target_rate_mag_cds, start_frq, stop_frq);
                dwell_freq = waveform_freq_rads;
            }
        }
        filt_attitude_cd.x += alpha * (attitude_cd.x - filt_attitude_cd.x);
        filt_attitude_cd.y += alpha * (attitude_cd.y - filt_attitude_cd.y);
        filt_attitude_cd.z += alpha * wrap_180_cd(attitude_cd.z - filt_attitude_cd.z);
    } else {
        target_rate_cds = 0.0f;
        settle_time--;
        dwell_start_time_ms = now;
        trim_command = command_out;
        filt_attitude_cd = attitude_cd;
        trim_attitude_cd = attitude_cd;
    }

    switch (axis) {
    case ROLL:
        gyro_reading = ahrs_view->get_gyro().x;
        command_reading = motors->get_roll();
        tgt_rate_reading = attitude_control->rate_bf_targets().x;
        if (settle_time == 0) {
            float ff_rate_contr = 0.0f;
            if (tune_roll_rff > 0.0f) {
                ff_rate_contr = 5730.0f * trim_command / tune_roll_rff;
            }
            float trim_rate_cds = ff_rate_contr + att_hold_gain * (trim_attitude_cd.x - filt_attitude_cd.x) - 5730.0f * vel_hold_gain * velocity_bf.y;
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, att_hold_gain * (trim_attitude_cd.y - filt_attitude_cd.y), 0.0f);
            attitude_control->rate_bf_roll_target(target_rate_cds + trim_rate_cds);
        } else {
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
            if (!is_zero(attitude_control->get_rate_roll_pid().ff() + attitude_control->get_rate_roll_pid().kP())) {
                float trim_tgt_rate_cds = 5730.0f * (trim_pff_out + trim_meas_rate * attitude_control->get_rate_roll_pid().kP()) / (attitude_control->get_rate_roll_pid().ff() + attitude_control->get_rate_roll_pid().kP());
                attitude_control->rate_bf_roll_target(trim_tgt_rate_cds);
            }
        }
        break;
    case PITCH:
        gyro_reading = ahrs_view->get_gyro().y;
        command_reading = motors->get_pitch();
        tgt_rate_reading = attitude_control->rate_bf_targets().y;
        if (settle_time == 0) {
            float ff_rate_contr = 0.0f;
            if (tune_pitch_rff > 0.0f) {
                ff_rate_contr = 5730.0f * trim_command / tune_pitch_rff;
            }
            float trim_rate_cds = ff_rate_contr + att_hold_gain * (trim_attitude_cd.y - filt_attitude_cd.y) + 5730.0f * vel_hold_gain * velocity_bf.x;
            attitude_control->input_rate_bf_roll_pitch_yaw(att_hold_gain * (trim_attitude_cd.x - filt_attitude_cd.x), 0.0f, 0.0f);
            attitude_control->rate_bf_pitch_target(target_rate_cds + trim_rate_cds);
        } else {
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
            if (!is_zero(attitude_control->get_rate_pitch_pid().ff() + attitude_control->get_rate_pitch_pid().kP())) {
                float trim_tgt_rate_cds = 5730.0f * (trim_pff_out + trim_meas_rate * attitude_control->get_rate_pitch_pid().kP()) / (attitude_control->get_rate_pitch_pid().ff() + attitude_control->get_rate_pitch_pid().kP());
                attitude_control->rate_bf_pitch_target(trim_tgt_rate_cds);
            }
        }
        break;
    case YAW:
        gyro_reading = ahrs_view->get_gyro().z;
        command_reading = motors->get_yaw();
        tgt_rate_reading = attitude_control->rate_bf_targets().z;
        if (settle_time == 0) {
            float rp_rate_contr = 0.0f;
            if (tune_yaw_rp > 0.0f) {
                rp_rate_contr = 5730.0f * trim_command / tune_yaw_rp;
            }
            float trim_rate_cds = rp_rate_contr + att_hold_gain * wrap_180_cd(trim_attitude_cd.z - filt_attitude_cd.z);
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
            attitude_control->rate_bf_yaw_target(target_rate_cds + trim_rate_cds);
        } else {
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
            if (!is_zero(attitude_control->get_rate_yaw_pid().ff() + attitude_control->get_rate_yaw_pid().kP())) {
                float trim_tgt_rate_cds = 5730.0f * (trim_pff_out + trim_meas_rate * attitude_control->get_rate_yaw_pid().kP()) / (attitude_control->get_rate_yaw_pid().ff() + attitude_control->get_rate_yaw_pid().kP());
                attitude_control->rate_bf_yaw_target(trim_tgt_rate_cds);
            }
        }
        break;
    }

    if (settle_time == 0) {
        filt_command_reading += alpha * (command_reading - filt_command_reading);
        filt_gyro_reading += alpha * (gyro_reading - filt_gyro_reading);
        filt_tgt_rate_reading += alpha * (tgt_rate_reading - filt_tgt_rate_reading);
    } else {
        filt_command_reading = command_reading;
        filt_gyro_reading = gyro_reading;
        filt_tgt_rate_reading = tgt_rate_reading;
    }

    // looks at gain and phase of input rate to output rate
    rotation_rate = rotation_rate_filt.apply((gyro_reading - filt_gyro_reading),
                    AP::scheduler().get_loop_period_s());
    filt_target_rate = target_rate_filt.apply((tgt_rate_reading - filt_tgt_rate_reading),
                       AP::scheduler().get_loop_period_s());
    command_out = command_filt.apply((command_reading - filt_command_reading),
                                     AP::scheduler().get_loop_period_s());

    // wait for dwell to start before determining gain and phase or just start if sweep
    if ((float)(now - dwell_start_time_ms) > 6.25f * cycle_time_ms || (!is_equal(start_frq,stop_frq) && settle_time == 0)) {
        if (freq_resp_input == 1) {
            freqresp_rate.determine_gain(filt_target_rate,rotation_rate, dwell_freq);
        } else {
            freqresp_rate.determine_gain(command_out,rotation_rate, dwell_freq);
        }
        if (freqresp_rate.is_cycle_complete()) {
            if (!is_equal(start_frq,stop_frq)) {
                curr_test_freq = freqresp_rate.get_freq();
                curr_test_gain = freqresp_rate.get_gain();
                curr_test_phase = freqresp_rate.get_phase();
                // reset cycle_complete to allow indication of next cycle
                freqresp_rate.reset_cycle_complete();
                // log sweep data
                Log_AutoTuneSweep();
                gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f phase=%f", (double)(curr_test_freq), (double)(curr_test_gain), (double)(curr_test_phase));
            } else {
                dwell_gain = freqresp_rate.get_gain();
                dwell_phase = freqresp_rate.get_phase();
            }

        }
    }

    // set sweep data if a frequency sweep is being conducted
    if (!is_equal(start_frq,stop_frq) && (float)(now - dwell_start_time_ms) > 2.5f * cycle_time_ms) {
        // track sweep phase to prevent capturing 180 deg and 270 deg data after phase has wrapped.
        if (curr_test_phase > 180.0f && sweep.progress == 0) {
            sweep.progress = 1;
        } else if (curr_test_phase > 270.0f && sweep.progress == 1) {
            sweep.progress = 2;
        }
        if (curr_test_phase <= 160.0f && curr_test_phase >= 150.0f && sweep.progress == 0) {
            sweep.ph180_freq = curr_test_freq;
            sweep.ph180_gain = curr_test_gain;
            sweep.ph180_phase = curr_test_phase;
        }
        if (curr_test_phase <= 250.0f && curr_test_phase >= 240.0f && sweep.progress == 1) {
            sweep.ph270_freq = curr_test_freq;
            sweep.ph270_gain = curr_test_gain;
            sweep.ph270_phase = curr_test_phase;
        }
        if (curr_test_gain > sweep.maxgain_gain) {
            sweep.maxgain_gain = curr_test_gain;
            sweep.maxgain_freq = curr_test_freq;
            sweep.maxgain_phase = curr_test_phase;
        }
        if (now - step_start_time_ms >= sweep_time_ms + 200) {
            // we have passed the maximum stop time
            step = UPDATE_GAINS;
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: max_freq=%f max_gain=%f", (double)(sweep.maxgain_freq), (double)(sweep.maxgain_gain));
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: ph180_freq=%f ph180_gain=%f", (double)(sweep.ph180_freq), (double)(sweep.ph180_gain));
        }

    } else {
        if (now - step_start_time_ms >= step_time_limit_ms || freqresp_rate.is_cycle_complete()) {
            // we have passed the maximum stop time
            step = UPDATE_GAINS;
        }
    }
}

void AC_AutoTune::angle_dwell_test_init(float filt_freq)
{
    rotation_rate_filt.set_cutoff_frequency(filt_freq);
    command_filt.set_cutoff_frequency(filt_freq);
    target_rate_filt.set_cutoff_frequency(filt_freq);
    dwell_start_time_ms = 0.0f;
    settle_time = 200;
    switch (axis) {
    case ROLL:
        rotation_rate_filt.reset(((float)ahrs_view->roll_sensor) / 5730.0f);
        command_filt.reset(motors->get_roll());
        target_rate_filt.reset(((float)attitude_control->get_att_target_euler_cd().x) / 5730.0f);
        rotation_rate = ((float)ahrs_view->roll_sensor) / 5730.0f;
        command_out = motors->get_roll();
        filt_target_rate = ((float)attitude_control->get_att_target_euler_cd().x) / 5730.0f;
        break;
    case PITCH:
        rotation_rate_filt.reset(((float)ahrs_view->pitch_sensor) / 5730.0f);
        command_filt.reset(motors->get_pitch());
        target_rate_filt.reset(((float)attitude_control->get_att_target_euler_cd().y) / 5730.0f);
        rotation_rate = ((float)ahrs_view->pitch_sensor) / 5730.0f;
        command_out = motors->get_pitch();
        filt_target_rate = ((float)attitude_control->get_att_target_euler_cd().y) / 5730.0f;
        break;
    case YAW:
        // yaw angle will be centered on zero by removing trim heading
        rotation_rate_filt.reset(0.0f);
        command_filt.reset(motors->get_yaw());
        target_rate_filt.reset(0.0f);
        rotation_rate = 0.0f;
        command_out = motors->get_yaw();
        filt_target_rate = 0.0f;
        break;
    }
    if (!is_equal(start_freq,stop_freq)) {
        sweep.ph180_freq = 0.0f;
        sweep.ph180_gain = 0.0f;
        sweep.ph180_phase = 0.0f;
        sweep.ph270_freq = 0.0f;
        sweep.ph270_gain = 0.0f;
        sweep.ph270_phase = 0.0f;
        sweep.maxgain_gain = 0.0f;
        sweep.maxgain_freq = 0.0f;
        sweep.maxgain_phase = 0.0f;
        curr_test_gain = 0.0f;
        curr_test_phase = 0.0f;
    }
}

void AC_AutoTune::angle_dwell_test_run(float start_frq, float stop_frq, float &dwell_gain, float &dwell_phase)
{
    float gyro_reading = 0.0f;
    float command_reading = 0.0f;
    float tgt_rate_reading = 0.0f;
    float tgt_attitude = 5.0f * 0.01745f;
    const uint32_t now = AP_HAL::millis();
    float target_angle_cd;
    static float trim_yaw_tgt_reading = 0.0f;
    static float trim_yaw_heading_reading = 0.0f;
    float sweep_time_ms = 23000;
    float dwell_freq = start_frq;
    static float filt_command_reading;
    static float filt_gyro_reading;
    static float filt_tgt_rate_reading;

    const float alpha = calc_lowpass_alpha_dt(0.0025f, 0.2f * start_frq);

    // adjust target attitude based on input_tc so amplitude decrease with increased frequency is minimized
    const float freq_co = 1.0f / attitude_control->get_input_tc();
    const float added_ampl = (safe_sqrt(powf(dwell_freq,2.0) + powf(freq_co,2.0)) / freq_co) - 1.0f;
    tgt_attitude = constrain_float(0.08725f * (1.0f + 0.2f * added_ampl), 0.08725f, 0.5235f);

    float cycle_time_ms = 0;
    if (!is_zero(dwell_freq)) {
        cycle_time_ms = 1000.0f * 6.28f / dwell_freq;
    }

    if (settle_time == 0) {
        // give gentler start for the dwell
        if ((float)(now - dwell_start_time_ms) < 0.5f * cycle_time_ms) {
            target_angle_cd = 0.5f * tgt_attitude * 5730.0f * (cosf(dwell_freq * (now - dwell_start_time_ms) * 0.001) - 1.0f);
        } else {
            if (is_equal(start_frq,stop_frq)) {
                target_angle_cd = -tgt_attitude * 5730.0f * sinf(dwell_freq * (now - dwell_start_time_ms - 0.25f * cycle_time_ms) * 0.001);
            } else {
                target_angle_cd = -waveform((now - dwell_start_time_ms - 0.25f * cycle_time_ms) * 0.001, (sweep_time_ms - 0.25f * cycle_time_ms) * 0.001f, tgt_attitude * 5730.0f, start_frq, stop_frq);
                dwell_freq = waveform_freq_rads;
            }
        }
    } else {
        target_angle_cd = 0.0f;
        trim_yaw_tgt_reading = (float)attitude_control->get_att_target_euler_cd().z;
        trim_yaw_heading_reading = (float)ahrs_view->yaw_sensor;
        settle_time--;
        dwell_start_time_ms = now;
    }

    float target_roll_cd, target_pitch_cd, target_yaw_rate_cds;
    get_pilot_desired_rp_yrate_cd(target_roll_cd, target_pitch_cd, target_yaw_rate_cds);

    switch (axis) {
    case ROLL:
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll_cd + target_angle_cd, target_pitch_cd, 0.0f);
        command_reading = motors->get_roll();
        tgt_rate_reading = ((float)attitude_control->get_att_target_euler_cd().x) / 5730.0f;
        gyro_reading = ((float)ahrs_view->roll_sensor) / 5730.0f;
        break;
    case PITCH:
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll_cd, target_pitch_cd + target_angle_cd, 0.0f);
        command_reading = motors->get_pitch();
        tgt_rate_reading = ((float)attitude_control->get_att_target_euler_cd().y) / 5730.0f;
        gyro_reading = ((float)ahrs_view->pitch_sensor) / 5730.0f;
        break;
    case YAW:
        command_reading = motors->get_yaw();
        tgt_rate_reading = (wrap_180_cd((float)attitude_control->get_att_target_euler_cd().z - trim_yaw_tgt_reading)) / 5730.0f;
        gyro_reading = (wrap_180_cd((float)ahrs_view->yaw_sensor - trim_yaw_heading_reading)) / 5730.0f;
        attitude_control->input_euler_angle_roll_pitch_yaw(target_roll_cd, target_pitch_cd, wrap_180_cd(trim_yaw_tgt_reading + target_angle_cd), false);
        break;
    }

    if (settle_time == 0) {
        filt_command_reading += alpha * (command_reading - filt_command_reading);
        filt_gyro_reading += alpha * (gyro_reading - filt_gyro_reading);
        filt_tgt_rate_reading += alpha * (tgt_rate_reading - filt_tgt_rate_reading);
    } else {
        filt_command_reading = command_reading;
        filt_gyro_reading = gyro_reading;
        filt_tgt_rate_reading = tgt_rate_reading;
    }

    // looks at gain and phase of input rate to output rate
    rotation_rate = rotation_rate_filt.apply((gyro_reading - filt_gyro_reading),
                AP::scheduler().get_loop_period_s());
    filt_target_rate = target_rate_filt.apply((tgt_rate_reading - filt_tgt_rate_reading),
                AP::scheduler().get_loop_period_s());
    command_out = command_filt.apply((command_reading - filt_command_reading),
                AP::scheduler().get_loop_period_s());

    // wait for dwell to start before determining gain and phase
    if ((float)(now - dwell_start_time_ms) > 6.25f * cycle_time_ms || (!is_equal(start_frq,stop_frq) && settle_time == 0)) {
        freqresp_angle.determine_gain_angle(command_out, filt_target_rate, rotation_rate, dwell_freq);
        if (freqresp_angle.is_cycle_complete()) {
            if (!is_equal(start_frq,stop_frq)) {
                curr_test_freq = freqresp_angle.get_freq();
                curr_test_gain = freqresp_angle.get_gain();
                curr_test_phase = freqresp_angle.get_phase();
                test_accel_max = freqresp_angle.get_accel_max();
                // reset cycle_complete to allow indication of next cycle
                freqresp_angle.reset_cycle_complete();
                // log sweep data
                Log_AutoTuneSweep();
                gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f phase=%f", (double)(curr_test_freq), (double)(curr_test_gain), (double)(curr_test_phase));
            } else {
                dwell_gain = freqresp_angle.get_gain();
                dwell_phase = freqresp_angle.get_phase();
            }
        }
    }

    // set sweep data if a frequency sweep is being conducted
    if (!is_equal(start_frq,stop_frq)) {
        if (curr_test_phase <= 160.0f && curr_test_phase >= 150.0f) {
            sweep.ph180_freq = curr_test_freq;
            sweep.ph180_gain = curr_test_gain;
            sweep.ph180_phase = curr_test_phase;
        }
        if (curr_test_phase <= 250.0f && curr_test_phase >= 240.0f) {
            sweep.ph270_freq = curr_test_freq;
            sweep.ph270_gain = curr_test_gain;
            sweep.ph270_phase = curr_test_phase;
        }
        if (curr_test_gain > sweep.maxgain_gain) {
            sweep.maxgain_gain = curr_test_gain;
            sweep.maxgain_freq = curr_test_freq;
            sweep.maxgain_phase = curr_test_phase;
        }
        if (now - step_start_time_ms >= sweep_time_ms + 200) {
            // we have passed the maximum stop time
            step = UPDATE_GAINS;
        }
    } else {
        if (now - step_start_time_ms >= step_time_limit_ms || freqresp_angle.is_cycle_complete()) {
            // we have passed the maximum stop time
            step = UPDATE_GAINS;
        }
    }
}

// init_test - initialises the test
float AC_AutoTune::waveform(float time, float time_record, float waveform_magnitude, float wMin, float wMax)
{
    float time_fade_in = 0.0f;      // Time to reach maximum amplitude of chirp
    float time_fade_out = 0.1 * time_record;     // Time to reach zero amplitude after chirp finishes
    float time_const_freq = 0.0f;

    float window;
    float output;

    float B = logf(wMax / wMin);

    if (time <= 0.0f) {
        window = 0.0f;
    } else if (time <= time_fade_in) {
        window = 0.5 - 0.5 * cosf(M_PI * time / time_fade_in);
    } else if (time <= time_record - time_fade_out) {
        window = 1.0;
    } else if (time <= time_record) {
        window = 0.5 - 0.5 * cosf(M_PI * (time - (time_record - time_fade_out)) / time_fade_out + M_PI);
    } else {
        window = 0.0;
    }

    if (time <= 0.0f) {
        waveform_freq_rads = wMin;
        output = 0.0f;
    } else if (time <= time_const_freq) {
        waveform_freq_rads = wMin;
        output = window * waveform_magnitude * sinf(wMin * time - wMin * time_const_freq);
    } else if (time <= time_record) {
        waveform_freq_rads = wMin * expf(B * (time - time_const_freq) / (time_record - time_const_freq));
        output = window * waveform_magnitude * sinf((wMin * (time_record - time_const_freq) / B) * (expf(B * (time - time_const_freq) / (time_record - time_const_freq)) - 1));
    } else {
        waveform_freq_rads = wMax;
        output = 0.0f;
    }
    return output;
}

