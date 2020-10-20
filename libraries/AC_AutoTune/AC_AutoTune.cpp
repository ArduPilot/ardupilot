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
#define AUTOTUNE_RP_ACCEL_MIN           4000.0f     // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_Y_ACCEL_MIN            1000.0f     // Minimum acceleration for Yaw
#define AUTOTUNE_Y_FILT_FREQ              10.0f     // Autotune filter frequency when testing Yaw
#define AUTOTUNE_RD_BACKOFF                1.0f     // Rate D gains are reduced to 50% of their maximum value discovered during tuning
#define AUTOTUNE_RP_BACKOFF                1.0f     // Rate P gains are reduced to 97.5% of their maximum value discovered during tuning
#define AUTOTUNE_SP_BACKOFF                0.9f     // Stab P gains are reduced to 90% of their maximum value discovered during tuning
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

#ifdef HELI_BUILD
// heli defines
#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS   5000U    // timeout for tuning mode's testing step
#else
// Frame specific defaults
#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS   1000U    // timeout for tuning mode's testing step
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

    // @Param: AGGR
    // @DisplayName: Autotune aggressiveness
    // @Description: Autotune aggressiveness. Defines the bounce back used to detect size of the D term.
    // @Range: 0.05 0.10
    // @User: Standard
    AP_GROUPINFO("AGGR", 2, AC_AutoTune, aggressiveness, 0.1f),

    // @Param: MIN_D
    // @DisplayName: AutoTune minimum D
    // @Description: Defines the minimum D gain
    // @Range: 0.001 0.006
    // @User: Standard
    AP_GROUPINFO("MIN_D", 3, AC_AutoTune, min_d,  0.001f),

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

    ff_up_first_iter = true;

    // re-initializes dwell test sequence for rate_p_up and rate_d_up tests for tradheli
    freq_cnt = 0;

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

// attitude_controller - sets attitude control targets during tuning
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
                    tune_yaw_rLPF = MAX(get_rlpf_min(), tune_yaw_rLPF * AUTOTUNE_RD_BACKOFF);
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
                    tune_roll_accel = MAX(AUTOTUNE_RP_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    break;
                case PITCH:
                    tune_pitch_sp = MAX(get_sp_min(), tune_pitch_sp * AUTOTUNE_SP_BACKOFF);
                    tune_pitch_accel = MAX(AUTOTUNE_RP_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    break;
                case YAW:
                    tune_yaw_sp = MAX(get_sp_min(), tune_yaw_sp * AUTOTUNE_SP_BACKOFF);
                    tune_yaw_accel = MAX(AUTOTUNE_Y_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_Y_BACKOFF);
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

        // reverse direction
        positive_direction = !positive_direction;

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
            attitude_control->get_rate_roll_pid().kI(get_load_tuned_ri(axis));
            attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
            attitude_control->get_rate_roll_pid().ff(tune_roll_rff);
            attitude_control->get_angle_roll_p().kP(tune_roll_sp);
            attitude_control->set_accel_roll_max_cdss(tune_roll_accel);
        }
    }
    if (pitch_enabled()) {
        if (!is_zero(tune_pitch_rp) || allow_zero_rate_p()) {
            attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(get_load_tuned_ri(axis));
            attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
            attitude_control->get_rate_pitch_pid().ff(tune_pitch_rff);
            attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
            attitude_control->set_accel_pitch_max_cdss(tune_pitch_accel);
        }
    }
    if (yaw_enabled()) {
        if (!is_zero(tune_yaw_rp)) {
            attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(get_load_tuned_ri(axis));
            attitude_control->get_rate_yaw_pid().kD(get_load_tuned_yaw_rd());
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
    }
    if (pitch_enabled()) {
        attitude_control->get_rate_pitch_pid().kP(orig_pitch_rp);
        attitude_control->get_rate_pitch_pid().kI(get_intra_test_ri(axis));
        attitude_control->get_rate_pitch_pid().kD(orig_pitch_rd);
        attitude_control->get_rate_pitch_pid().ff(orig_pitch_rff);
        attitude_control->get_rate_pitch_pid().filt_T_hz(orig_pitch_fltt);
        attitude_control->get_rate_pitch_pid().slew_limit(orig_pitch_smax);
        attitude_control->get_angle_pitch_p().kP(orig_pitch_sp);
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
    }
}


// load_test_gains - load the to-be-tested gains for a single axis
// called by control_attitude() just before it beings testing a gain (i.e. just before it twitches)
void AC_AutoTune::load_test_gains()
{
    switch (axis) {
    case ROLL:
        attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
        attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
        attitude_control->get_angle_roll_p().kP(tune_roll_sp);
        break;
    case PITCH:
        attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
        attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
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
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL) && roll_enabled() && !is_zero(tune_roll_rp)) {
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

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) && pitch_enabled() && !is_zero(tune_pitch_rp)) {
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
}

void AC_AutoTune::rate_ff_test_run(float max_angle_cds, float target_rate_cds)
{
    float gyro_reading = 0.0f;
    float command_reading = 0.0f;
    float tgt_rate_reading = 0.0f;
    const uint32_t now = AP_HAL::millis();
    switch (axis) {
    case ROLL:
        gyro_reading = ahrs_view->get_gyro().x;
        command_reading = motors->get_roll();
        tgt_rate_reading = attitude_control->rate_bf_targets().x;
        if (ahrs_view->roll_sensor <= max_angle_cds + start_angle - 100.0f && ff_test_phase == 0) {
            attitude_control->input_rate_bf_roll_pitch_yaw(target_rate_cds, 0.0f, 0.0f);
        } else if (ahrs_view->roll_sensor > max_angle_cds + start_angle - 100.0f && ff_test_phase == 0) {
            ff_test_phase = 1;
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
            attitude_control->rate_bf_roll_target(-target_rate_cds);
        } else if (ahrs_view->roll_sensor >= -max_angle_cds + start_angle && ff_test_phase == 1 ) {
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
            attitude_control->rate_bf_roll_target(-target_rate_cds);
        } else if (ahrs_view->roll_sensor < -max_angle_cds + start_angle && ff_test_phase == 1 ) {
            ff_test_phase = 2;
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, start_angles.y, 0.0f);
        } else if (ff_test_phase == 2 ) {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, start_angles.y, 0.0f);
        }
        break;
    case PITCH:
        gyro_reading = ahrs_view->get_gyro().y;
        command_reading = motors->get_pitch();
        tgt_rate_reading = attitude_control->rate_bf_targets().y;
        if (ahrs_view->pitch_sensor <= max_angle_cds + start_angle - 100.0f && ff_test_phase == 0) {
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, target_rate_cds, 0.0f);
        } else if (ahrs_view->pitch_sensor > max_angle_cds + start_angle - 100.0f && ff_test_phase == 0) {
            ff_test_phase = 1;
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
            attitude_control->rate_bf_pitch_target(-target_rate_cds);
        } else if (ahrs_view->pitch_sensor >= -max_angle_cds + start_angle && ff_test_phase == 1 ) {
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
            attitude_control->rate_bf_pitch_target(-target_rate_cds);
        } else if (ahrs_view->pitch_sensor < -max_angle_cds + start_angle && ff_test_phase == 1 ) {
            ff_test_phase = 2;
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, start_angles.y, 0.0f);
        } else if (ff_test_phase == 2 ) {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, start_angles.y, 0.0f);
        }
        break;
    case YAW:
        gyro_reading = ahrs_view->get_gyro().z;
        command_reading = motors->get_yaw();
        tgt_rate_reading = attitude_control->rate_bf_targets().z;
        if (wrap_180_cd(ahrs_view->yaw_sensor) <= wrap_180_cd(max_angle_cds + start_angle - 100.0f) && ff_test_phase == 0) {
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.5f * target_rate_cds);
        } else if (wrap_180_cd(ahrs_view->yaw_sensor) > wrap_180_cd(max_angle_cds + start_angle - 100.0f) && ff_test_phase == 0) {
            ff_test_phase = 1;
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
            attitude_control->rate_bf_yaw_target(-target_rate_cds);
        } else if (wrap_180_cd(ahrs_view->yaw_sensor) >= wrap_180_cd(-max_angle_cds + start_angle) && ff_test_phase == 1 ) {
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
            attitude_control->rate_bf_yaw_target(-target_rate_cds);
        } else if (wrap_180_cd(ahrs_view->yaw_sensor) < wrap_180_cd(-max_angle_cds + start_angle) && ff_test_phase == 1 ) {
            ff_test_phase = 2;
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, start_angles.y, 0.0f);
        } else if (ff_test_phase == 2 ) {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, start_angles.y, 0.0f);
        }
        break;
    }

    rotation_rate = rotation_rate_filt.apply(gyro_reading,
                    AP::scheduler().get_loop_period_s());
    command_out = command_filt.apply(command_reading,
                                     AP::scheduler().get_loop_period_s());
    filt_target_rate = target_rate_filt.apply(tgt_rate_reading,
                       AP::scheduler().get_loop_period_s());

    // record steady state rate and motor command
    switch (axis) {
    case ROLL:
        if (ahrs_view->roll_sensor >= -max_angle_cds + start_angle && ff_test_phase == 1 ) {
            test_rate_filt = rotation_rate;
            test_command_filt = command_out;
            test_tgt_rate_filt = filt_target_rate;
        }
        break;
    case PITCH:
        if (ahrs_view->pitch_sensor >= -max_angle_cds + start_angle && ff_test_phase == 1 ) {
            test_rate_filt = rotation_rate;
            test_command_filt = command_out;
            test_tgt_rate_filt = filt_target_rate;
        }
        break;
    case YAW:
        if (wrap_180_cd(ahrs_view->yaw_sensor) >= wrap_180_cd(-max_angle_cds + start_angle) && ff_test_phase == 1 ) {
            test_rate_filt = rotation_rate;
            test_command_filt = command_out;
            test_tgt_rate_filt = filt_target_rate;
        }
        break;
    }
    if (now - step_start_time_ms >= step_time_limit_ms || ff_test_phase == 2) {
        // we have passed the maximum stop time
        step = UPDATE_GAINS;
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
}

void AC_AutoTune::dwell_test_run(uint8_t freq_resp_input, float dwell_freq, float &dwell_gain, float &dwell_phase)
{
    float gyro_reading = 0.0f;
    float command_reading = 0.0f;
    float tgt_rate_reading = 0.0f;
    float tgt_attitude = 2.5f * 0.01745f;
    const uint32_t now = AP_HAL::millis();
    float target_rate_cds;

    switch (axis) {
    case ROLL:
        gyro_reading = ahrs_view->get_gyro().x;
        command_reading = motors->get_roll();
        tgt_rate_reading = attitude_control->rate_bf_targets().x;
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
        if (is_zero(dwell_start_time_ms) && ahrs_view->roll_sensor > -0.95f * tgt_attitude * 5730.0f + start_angle) {
            attitude_control->rate_bf_roll_target(-1000.0f);
        } else if (is_zero(dwell_start_time_ms) && ahrs_view->roll_sensor < -0.95f * tgt_attitude * 5730.0f + start_angle) {
            attitude_control->rate_bf_roll_target(0.0f);
            dwell_start_time_ms = now;
        } else {
            target_rate_cds = tgt_attitude * dwell_freq * 5730.0f * sinf(dwell_freq * (now - dwell_start_time_ms) * 0.001);
            attitude_control->rate_bf_roll_target(target_rate_cds);
        }
        break;
    case PITCH:
        gyro_reading = ahrs_view->get_gyro().y;
        command_reading = motors->get_pitch();
        tgt_rate_reading = attitude_control->rate_bf_targets().y;
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
        if (is_zero(dwell_start_time_ms) && ahrs_view->pitch_sensor > -0.95f * tgt_attitude * 5730.0f + start_angle) {
            attitude_control->rate_bf_pitch_target(-1000.0f);
        } else if (is_zero(dwell_start_time_ms) && ahrs_view->pitch_sensor < -0.95f * tgt_attitude * 5730.0f + start_angle) {
            attitude_control->rate_bf_pitch_target(0.0f);
            dwell_start_time_ms = now;
        } else {
            target_rate_cds = tgt_attitude * dwell_freq * 5730.0f * sinf(dwell_freq * (now - dwell_start_time_ms) * 0.001);
            attitude_control->rate_bf_pitch_target(target_rate_cds);
        }
        break;
    case YAW:
        gyro_reading = ahrs_view->get_gyro().z;
        command_reading = motors->get_yaw();
        tgt_rate_reading = attitude_control->rate_bf_targets().z;
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
        if (is_zero(dwell_start_time_ms) && ahrs_view->yaw_sensor > -0.95f * tgt_attitude * 5730.0f + start_angle) {
            attitude_control->rate_bf_yaw_target(-1000.0f);
        } else if (is_zero(dwell_start_time_ms) && ahrs_view->yaw_sensor < -0.95f * tgt_attitude * 5730.0f + start_angle) {
            attitude_control->rate_bf_yaw_target(0.0f);
            dwell_start_time_ms = now;
        } else {
            target_rate_cds = tgt_attitude * dwell_freq * 5730.0f * sinf(dwell_freq * (now - dwell_start_time_ms) * 0.001);
            attitude_control->rate_bf_yaw_target(target_rate_cds);
        }
        break;
    }

    // looks at gain and phase of input rate to output rate
    rotation_rate = rotation_rate_filt.apply(gyro_reading,
                    AP::scheduler().get_loop_period_s());
    filt_target_rate = target_rate_filt.apply(tgt_rate_reading,
                       AP::scheduler().get_loop_period_s());
    command_out = command_filt.apply(command_reading,
                                     AP::scheduler().get_loop_period_s());

    // wait for dwell to start before determining gain and phase
    if (!is_zero(dwell_start_time_ms)) {
        if (freq_resp_input == 1) {
            determine_gain(filt_target_rate,rotation_rate, dwell_freq, dwell_gain, dwell_phase, dwell_complete, false);
        } else {
            determine_gain(command_out,rotation_rate, dwell_freq, dwell_gain, dwell_phase, dwell_complete, false);
        }
    }

    if (now - step_start_time_ms >= step_time_limit_ms || dwell_complete) {
        // we have passed the maximum stop time
        step = UPDATE_GAINS;
    }
}

void AC_AutoTune::angle_dwell_test_init(float filt_freq)
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
}

void AC_AutoTune::angle_dwell_test_run(float dwell_freq, float &dwell_gain, float &dwell_phase)
{
    float gyro_reading = 0.0f;
    float command_reading = 0.0f;
    float tgt_rate_reading = 0.0f;
    float tgt_attitude = 5.0f * 0.01745f;
    const uint32_t now = AP_HAL::millis();
    float target_angle_cd;
    float target_rate_cds;
    static uint32_t settle_time = 200;
    static bool dtrmn_gain;

    switch (axis) {
    case ROLL:
        //        gyro_reading = ahrs_view->get_gyro().x;
        command_reading = motors->get_roll();
        //        tgt_rate_reading = attitude_control->rate_bf_targets().x;

        attitude_control->bf_feedforward(false);
        if (settle_time == 0) {
            target_angle_cd = -tgt_attitude * 5730.0f * sinf(dwell_freq * (now - dwell_start_time_ms) * 0.001);
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_angle_cd + start_angles.x, start_angles.y, 0.0f);
        } else {
            target_angle_cd = 0.0f;
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, start_angles.y, 0.0f);
            settle_time--;
            dwell_start_time_ms = now;
        }
        tgt_rate_reading = target_angle_cd / 5730.0f;
        gyro_reading = ((float)ahrs_view->roll_sensor - (float)start_angle + target_angle_cd) / 5730.0f;
        break;
    case PITCH:
        gyro_reading = ahrs_view->get_gyro().y;
        command_reading = motors->get_pitch();
        tgt_rate_reading = attitude_control->rate_bf_targets().y;
        target_angle_cd = tgt_attitude * 5730.0f * sinf(dwell_freq * (now - dwell_start_time_ms) * 0.001);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, target_angle_cd + start_angles.y, 0.0f);
        break;
    case YAW:
        gyro_reading = ahrs_view->get_gyro().z;
        command_reading = motors->get_yaw();
        tgt_rate_reading = attitude_control->rate_bf_targets().z;
        target_rate_cds = tgt_attitude * dwell_freq * 5730.0f * sinf(dwell_freq * (now - dwell_start_time_ms) * 0.001);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, start_angles.y, target_rate_cds);
        break;
    }

    // need to hold off from starting determine gain until neg rate is established
    if (!is_zero(dwell_start_time_ms) && tgt_rate_reading < -0.01745f && !dtrmn_gain) {
        dtrmn_gain = true;
    }

    // looks at gain and phase of input rate to output rate
    rotation_rate = rotation_rate_filt.apply(gyro_reading,
                    AP::scheduler().get_loop_period_s());
    filt_target_rate = target_rate_filt.apply(tgt_rate_reading,
                       AP::scheduler().get_loop_period_s());
    command_out = command_filt.apply(command_reading,
                                     AP::scheduler().get_loop_period_s());

    // wait for dwell to start before determining gain and phase
    if (!is_zero(dwell_start_time_ms) && dtrmn_gain) {
        determine_gain(filt_target_rate,rotation_rate, dwell_freq, dwell_gain, dwell_phase, dwell_complete, false);
    }
    //    if (now - step_start_time_ms >= step_time_limit_ms || dwell_complete) {
    if (now - step_start_time_ms >= step_time_limit_ms || dwell_complete) {
        // we have passed the maximum stop time
        step = UPDATE_GAINS;
        settle_time = 200;
        dtrmn_gain = false;
        // announce results of dwell and update
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f ph=%f", (double)(dwell_freq), (double)(dwell_gain), (double)(dwell_phase));

    }
}


// determine_gain - this function receives time history data during a dwell test input and determines the gain and phase of the response to the input.
// Once the designated number of cycles are complete, the average of the gain and phase are determined over the last 5 cycles and the cycles_complete flag
// is set.  This function must be reset using the reset flag prior to the next dwell.
void AC_AutoTune::determine_gain(float tgt_rate, float meas_rate, float freq, float &gain, float &phase, bool &cycles_complete, bool funct_reset)
{
    static float max_target, max_meas, prev_target, prev_meas;
    static float min_target, min_meas, output_ampl[AUTOTUNE_DWELL_CYCLES+1], input_ampl[AUTOTUNE_DWELL_CYCLES+1];
    static float temp_max_target, temp_min_target;
    static float temp_max_meas, temp_min_meas;
    static uint32_t temp_max_tgt_time[AUTOTUNE_DWELL_CYCLES+1], temp_max_meas_time[AUTOTUNE_DWELL_CYCLES+1];
    static uint32_t max_tgt_time, max_meas_time, new_tgt_time_ms, new_meas_time_ms;
    static uint8_t min_target_cnt, max_target_cnt, max_meas_cnt, min_meas_cnt;
    static bool new_target = false;
    static bool new_meas = false;
    uint32_t now = AP_HAL::millis();

    if (funct_reset) {
        max_target_cnt = 0;
        min_target_cnt = 0;
        max_meas_cnt = 0;
        min_meas_cnt = 0;
        new_tgt_time_ms = 0;
        new_meas_time_ms = 0;
        new_target = false;
        new_meas = false;
        gain = 0.0f;
        phase = 0.0f;
        cycles_complete = false;
        funct_reset = false;
        return;
    }

    uint32_t half_cycle_time_ms = 0;
    if (!is_zero(freq)) {
        half_cycle_time_ms = (uint32_t)(400 * 6.28 / freq);
    }

    // cycles are complete! determine gain and phase and exit
    if (max_meas_cnt > AUTOTUNE_DWELL_CYCLES + 1 && max_target_cnt > AUTOTUNE_DWELL_CYCLES + 1) {
        float delta_time = 0.0f;
        float sum_gain = 0.0f;
        uint8_t cnt = 0;
        uint8_t gcnt = 0;
        for (int i = 0;  i < 5; i++) {
            if (input_ampl[AUTOTUNE_DWELL_CYCLES - i] > 0) {
                sum_gain += output_ampl[AUTOTUNE_DWELL_CYCLES - i] / input_ampl[AUTOTUNE_DWELL_CYCLES - i];
                gcnt++;
            }
            float d_time = (float)(temp_max_meas_time[AUTOTUNE_DWELL_CYCLES - i] - temp_max_tgt_time[AUTOTUNE_DWELL_CYCLES - i]);
            if (d_time * 0.001f < 5.0f * 6.28f / freq) {
                delta_time += d_time;
                cnt++;
            }
        }
        if (gcnt > 0) {
            gain = sum_gain / gcnt;
        }
        if (cnt > 0) {
            delta_time = delta_time / cnt;
        }
        phase = freq * delta_time * 0.001f * 360.0f / 6.28f;
        if (phase > 360.0f) {
            phase = phase - 360.0f;
        }
        cycles_complete = true;
        return;
    }

    // Indicates when the target(input) is positive or negative half of the cycle to notify when the max or min should be sought
    if (!is_positive(prev_target) && is_positive(tgt_rate) && !new_target && now > new_tgt_time_ms) {
        new_target = true;
        new_tgt_time_ms = now + half_cycle_time_ms;
        // reset max_target
        max_target = 0.0f;
        max_target_cnt++;
        temp_min_target = min_target;
        if (min_target_cnt > 0 && min_target_cnt < AUTOTUNE_DWELL_CYCLES + 1) {
            input_ampl[min_target_cnt] = temp_max_target - temp_min_target;
        }
        //        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: max_tgt_cnt=%f", (double)(max_target_cnt));

    } else if (is_positive(prev_target) && !is_positive(tgt_rate) && new_target && now > new_tgt_time_ms && max_target_cnt > 0) {
        new_target = false;
        new_tgt_time_ms = now + half_cycle_time_ms;
        min_target_cnt++;
        temp_max_target = max_target;
        if (min_target_cnt < AUTOTUNE_DWELL_CYCLES + 1) {
            temp_max_tgt_time[min_target_cnt] = max_tgt_time;
        }
        min_target = 0.0f;
        //        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: min_tgt_cnt=%f", (double)(min_target_cnt));
    }

    // Indicates when the measured value (output) is positive or negative half of the cycle to notify when the max or min should be sought
    if (!is_positive(prev_meas) && is_positive(meas_rate) && !new_meas && now > new_meas_time_ms && max_target_cnt > 0) {
        new_meas = true;
        new_meas_time_ms = now + half_cycle_time_ms;
        // reset max_meas
        max_meas = 0.0f;
        max_meas_cnt++;
        temp_min_meas = min_meas;
        if (min_meas_cnt > 0 && min_target_cnt > 0 && min_meas_cnt < AUTOTUNE_DWELL_CYCLES + 1) {
            output_ampl[min_meas_cnt] = temp_max_meas - temp_min_meas;
        }
        //        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: max_meas_cnt=%f", (double)(max_meas_cnt));
    } else if (is_positive(prev_meas) && !is_positive(meas_rate) && new_meas && now > new_meas_time_ms && max_meas_cnt > 0) {
        new_meas = false;
        new_meas_time_ms = now + half_cycle_time_ms;
        min_meas_cnt++;
        temp_max_meas = max_meas;
        if (min_meas_cnt < AUTOTUNE_DWELL_CYCLES + 1) {
            temp_max_meas_time[min_meas_cnt] = max_meas_time;
        }
        min_meas = 0.0f;
        //        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: min_meas_cnt=%f", (double)(min_meas_cnt));
    }

    if (tgt_rate > max_target && new_target) {
        max_target = tgt_rate;
        max_tgt_time = now;
    }

    if (tgt_rate < min_target && !new_target) {
        min_target = tgt_rate;
    }

    if (meas_rate > max_meas && new_meas) {
        max_meas = meas_rate;
        max_meas_time = now;
    }

    if (meas_rate < min_meas && !new_meas) {
        min_meas = meas_rate;
    }

    prev_target = tgt_rate;
    prev_meas = meas_rate;
}


