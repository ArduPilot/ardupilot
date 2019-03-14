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

#define AUTOTUNE_AXIS_BITMASK_ROLL            1
#define AUTOTUNE_AXIS_BITMASK_PITCH           2
#define AUTOTUNE_AXIS_BITMASK_YAW             4

#define AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS  500     // restart tuning if pilot has left sticks in middle for 2 seconds
#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS   1000U    // timeout for tuning mode's testing step
#define AUTOTUNE_LEVEL_ANGLE_CD             500     // angle which qualifies as level
#define AUTOTUNE_LEVEL_RATE_RP_CD          1000     // rate which qualifies as level for roll and pitch
#define AUTOTUNE_LEVEL_RATE_Y_CD            750     // rate which qualifies as level for yaw
#define AUTOTUNE_REQUIRED_LEVEL_TIME_MS     500     // time we require the aircraft to be level
#define AUTOTUNE_LEVEL_TIMEOUT_MS          2000     // time out for level
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
#define AUTOTUNE_TARGET_RATE_RLLPIT_CDS     18000   // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step
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

// second table of user settable parameters for quadplanes, this
// allows us to go beyond the 64 parameter limit
const AP_Param::GroupInfo AC_AutoTune::var_info[] = {
    // @Param: AXES
    // @DisplayName: Autotune axis bitmask
    // @Description: 1-byte bitmap of axes to autotune
    // @Values: 7:All,1:Roll Only,2:Pitch Only,4:Yaw Only,3:Roll and Pitch,5:Roll and Yaw,6:Pitch and Yaw
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
                                 AC_AttitudeControl_Multi *_attitude_control,
                                 AC_PosControl *_pos_control,
                                 AP_AHRS_View *_ahrs_view,
                                 AP_InertialNav *_inertial_nav)
{
    bool success = true;

    use_poshold = _use_poshold;
    attitude_control = _attitude_control;
    pos_control = _pos_control;
    ahrs_view = _ahrs_view;
    inertial_nav = _inertial_nav;
    motors = AP_Motors::get_singleton();

    switch (mode) {
    case FAILED:
        // autotune has been run but failed so reset state to uninitialized
        mode = UNINITIALISED;
        // fall through to restart the tuning
        FALLTHROUGH;

    case UNINITIALISED:
        // autotune has never been run
        success = start();
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
        success = start();
        if (success) {
            // reset gains to tuning-start gains (i.e. low I term)
            load_gains(GAIN_INTRA_TEST);
            // write dataflash log even and send message to ground station
            Log_Write_Event(EVENT_AUTOTUNE_RESTART);
            update_gcs(AUTOTUNE_MESSAGE_STARTED);
        }
        break;

    case SUCCESS:
        // we have completed a tune and the pilot wishes to test the new gains in the current flight mode
        // so simply apply tuning gains (i.e. do not change flight mode)
        load_gains(GAIN_TUNED);
        Log_Write_Event(EVENT_AUTOTUNE_PILOT_TESTING);
        break;
    }

    have_position = false;

    return success;
}

// stop - should be called when the ch7/ch8 switch is switched OFF
void AC_AutoTune::stop()
{
    // set gains to their original values
    load_gains(GAIN_ORIGINAL);

    // re-enable angle-to-rate request limits
    attitude_control->use_sqrt_controller(true);

    // log off event and send message to ground station
    update_gcs(AUTOTUNE_MESSAGE_STOPPED);
    Log_Write_Event(EVENT_AUTOTUNE_OFF);

    // Note: we leave the mode as it was so that we know how the autotune ended
    // we expect the caller will change the flight mode back to the flight mode indicated by the flight mode switch
}

// start - Initialize autotune flight mode
bool AC_AutoTune::start(void)
{
    if (!motors->armed()) {
        return false;
    }

    // initialize vertical speeds and leash lengths
    init_z_limits();

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav->get_velocity_z());
    }

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

const char *AC_AutoTune::type_string() const
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

void AC_AutoTune::do_gcs_announcements()
{
    const uint32_t now = AP_HAL::millis();
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
void AC_AutoTune::run()
{
    // initialize vertical speeds and acceleration
    init_z_limits();

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    // this should not actually be possible because of the init() checks
    if (!motors->armed() || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_GROUND_IDLE);
        attitude_control->set_throttle_out_unstabilized(0.0f, true, 0);
        pos_control->relax_alt_hold_controllers(0.0f);
        return;
    }

    float target_roll_cd, target_pitch_cd, target_yaw_rate_cds;
    get_pilot_desired_rp_yrate_cd(target_roll_cd, target_pitch_cd, target_yaw_rate_cds);

    // get pilot desired climb rate
    const float target_climb_rate_cms = get_pilot_desired_climb_rate_cms();

    const bool zero_rp_input = is_zero(target_roll_cd) && is_zero(target_pitch_cd);
    if (!zero_rp_input || !is_zero(target_yaw_rate_cds) || !is_zero(target_climb_rate_cms)) {
        if (!pilot_override) {
            pilot_override = true;
            // set gains to their original values
            load_gains(GAIN_ORIGINAL);
            attitude_control->use_sqrt_controller(true);
        }
        // reset pilot override time
        override_time = AP_HAL::millis();
        if (!zero_rp_input) {
            // only reset position on roll or pitch input
            have_position = false;
        }
    } else if (pilot_override) {
        uint32_t now = AP_HAL::millis();
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

    if (zero_rp_input) {
        // pilot input on throttle and yaw will still use position hold if enabled
        get_poshold_attitude(target_roll_cd, target_pitch_cd, desired_yaw_cd);
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

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
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate_cms, AP::scheduler().get_last_loop_time_s(), false);
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

    if (AP_HAL::millis() - level_start_time_ms > AUTOTUNE_LEVEL_TIMEOUT_MS) {
        // after a long wait we use looser threshold, to allow tuning
        // with poor initial gains
        threshold_mul *= 2;
    }

    if (!check_level(LevelIssue::ANGLE_ROLL,
                     abs(ahrs_view->roll_sensor - roll_cd),
                     threshold_mul*AUTOTUNE_LEVEL_ANGLE_CD)) {
        return false;
    }

    if (!check_level(LevelIssue::ANGLE_PITCH,
                     abs(ahrs_view->pitch_sensor - pitch_cd),
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
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Twitch");
            // initiate variables for next step
            step = TWITCHING;
            step_start_time_ms = now;
            step_time_limit_ms = AUTOTUNE_TESTING_STEP_TIMEOUT_MS;
            twitch_first_iter = true;
            test_rate_max = 0.0f;
            test_rate_min = 0.0f;
            test_angle_max = 0.0f;
            test_angle_min = 0.0f;
            rotation_rate_filt.reset(0.0f);
            rate_max = 0.0f;
            // set gains to their to-be-tested values
            load_gains(GAIN_TWITCH);
        } else {
            // when waiting for level we use the intra-test gains
            load_gains(GAIN_INTRA_TEST);
        }

        float target_max_rate;
        switch (axis) {
        case ROLL:
            target_max_rate = MAX(AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, step_scaler*AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
            target_rate = constrain_float(ToDeg(attitude_control->max_rate_step_bf_roll())*100.0f, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, target_max_rate);
            target_angle = constrain_float(ToDeg(attitude_control->max_angle_step_bf_roll())*100.0f, AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD, AUTOTUNE_TARGET_ANGLE_RLLPIT_CD);
            abort_angle = AUTOTUNE_TARGET_ANGLE_RLLPIT_CD;
            start_rate = ToDeg(ahrs_view->get_gyro().x) * 100.0f;
            start_angle = ahrs_view->roll_sensor;
            rotation_rate_filt.set_cutoff_frequency(attitude_control->get_rate_roll_pid().filt_hz()*2.0f);
            break;
        case PITCH:
            target_max_rate = MAX(AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, step_scaler*AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
            target_rate = constrain_float(ToDeg(attitude_control->max_rate_step_bf_pitch())*100.0f, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, target_max_rate);
            target_angle = constrain_float(ToDeg(attitude_control->max_angle_step_bf_pitch())*100.0f, AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD, AUTOTUNE_TARGET_ANGLE_RLLPIT_CD);
            abort_angle = AUTOTUNE_TARGET_ANGLE_RLLPIT_CD;
            start_rate = ToDeg(ahrs_view->get_gyro().y) * 100.0f;
            start_angle = ahrs_view->pitch_sensor;
            rotation_rate_filt.set_cutoff_frequency(attitude_control->get_rate_pitch_pid().filt_hz()*2.0f);
            break;
        case YAW:
            target_max_rate = MAX(AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, step_scaler*AUTOTUNE_TARGET_RATE_YAW_CDS);
            target_rate = constrain_float(ToDeg(attitude_control->max_rate_step_bf_yaw()*0.75f)*100.0f, AUTOTUNE_TARGET_MIN_RATE_YAW_CDS, target_max_rate);
            target_angle = constrain_float(ToDeg(attitude_control->max_angle_step_bf_yaw()*0.75f)*100.0f, AUTOTUNE_TARGET_MIN_ANGLE_YAW_CD, AUTOTUNE_TARGET_ANGLE_YAW_CD);
            abort_angle = AUTOTUNE_TARGET_ANGLE_YAW_CD;
            start_rate = ToDeg(ahrs_view->get_gyro().z) * 100.0f;
            start_angle = ahrs_view->yaw_sensor;
            rotation_rate_filt.set_cutoff_frequency(AUTOTUNE_Y_FILT_FREQ);
            break;
        }
        if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
            rotation_rate_filt.reset(start_rate);
        } else {
            rotation_rate_filt.reset(0);
        }
        break;
    }

    case TWITCHING: {
        // Run the twitching step
        load_gains(GAIN_TWITCH);

        // disable rate limits
        attitude_control->use_sqrt_controller(false);
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
        float gyro_reading = 0;
        switch (axis) {
        case ROLL:
            gyro_reading = ahrs_view->get_gyro().x;
            lean_angle = direction_sign * (ahrs_view->roll_sensor - (int32_t)start_angle);
            break;
        case PITCH:
            gyro_reading = ahrs_view->get_gyro().y;
            lean_angle = direction_sign * (ahrs_view->pitch_sensor - (int32_t)start_angle);
            break;
        case YAW:
            gyro_reading = ahrs_view->get_gyro().z;
            lean_angle = direction_sign * wrap_180_cd(ahrs_view->yaw_sensor-(int32_t)start_angle);
            break;
        }

        // Add filter to measurements
        float filter_value;
        switch (tune_type) {
        case SP_DOWN:
        case SP_UP:
            filter_value = direction_sign * (ToDeg(gyro_reading) * 100.0f);
            break;
        default:
            filter_value = direction_sign * (ToDeg(gyro_reading) * 100.0f - start_rate);
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
            if (lean_angle >= target_angle) {
                step = UPDATE_GAINS;
            }
            break;
        case RP_UP:
            twitching_test_rate(rotation_rate, target_rate*(1+0.5f*aggressiveness), test_rate_min, test_rate_max);
            twitching_measure_acceleration(test_accel_max, rotation_rate, rate_max);
            twitching_abort_rate(lean_angle, rotation_rate, abort_angle, test_rate_min);
            break;
        case SP_DOWN:
        case SP_UP:
            twitching_test_angle(lean_angle, rotation_rate, target_angle*(1+0.5f*aggressiveness), test_angle_min, test_angle_max, test_rate_min, test_rate_max);
            twitching_measure_acceleration(test_accel_max, rotation_rate - direction_sign * start_rate, rate_max);
            break;
        }

        // log this iterations lean angle and rotation rate
        Log_Write_AutoTuneDetails(lean_angle, rotation_rate);
        AP::logger().Write_Rate(ahrs_view, *motors, *attitude_control, *pos_control);
        log_pids();
        break;
    }

    case UPDATE_GAINS:

        // re-enable rate limits
        attitude_control->use_sqrt_controller(true);

        // log the latest gains
        if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
            switch (axis) {
            case ROLL:
                Log_Write_AutoTune(axis, tune_type, target_angle, test_angle_min, test_angle_max, tune_roll_rp, tune_roll_rd, tune_roll_sp, test_accel_max);
                break;
            case PITCH:
                Log_Write_AutoTune(axis, tune_type, target_angle, test_angle_min, test_angle_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, test_accel_max);
                break;
            case YAW:
                Log_Write_AutoTune(axis, tune_type, target_angle, test_angle_min, test_angle_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, test_accel_max);
                break;
            }
        } else {
            switch (axis) {
            case ROLL:
                Log_Write_AutoTune(axis, tune_type, target_rate, test_rate_min, test_rate_max, tune_roll_rp, tune_roll_rd, tune_roll_sp, test_accel_max);
                break;
            case PITCH:
                Log_Write_AutoTune(axis, tune_type, target_rate, test_rate_min, test_rate_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, test_accel_max);
                break;
            case YAW:
                Log_Write_AutoTune(axis, tune_type, target_rate, test_rate_min, test_rate_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, test_accel_max);
                break;
            }
        }

        // Check results after mini-step to increase rate D gain
        switch (tune_type) {
        case RD_UP:
            switch (axis) {
            case ROLL:
                updating_rate_d_up(tune_roll_rd, min_d, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
                break;
            case PITCH:
                updating_rate_d_up(tune_pitch_rd, min_d, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
                break;
            case YAW:
                updating_rate_d_up(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RLPF_MAX, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
                break;
            }
            break;
        // Check results after mini-step to decrease rate D gain
        case RD_DOWN:
            switch (axis) {
            case ROLL:
                updating_rate_d_down(tune_roll_rd, min_d, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
                break;
            case PITCH:
                updating_rate_d_down(tune_pitch_rd, min_d, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
                break;
            case YAW:
                updating_rate_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
                break;
            }
            break;
        // Check results after mini-step to increase rate P gain
        case RP_UP:
            switch (axis) {
            case ROLL:
                updating_rate_p_up_d_down(tune_roll_rd, min_d, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
                break;
            case PITCH:
                updating_rate_p_up_d_down(tune_pitch_rd, min_d, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
                break;
            case YAW:
                updating_rate_p_up_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
                break;
            }
            break;
        // Check results after mini-step to increase stabilize P gain
        case SP_DOWN:
            switch (axis) {
            case ROLL:
                updating_angle_p_down(tune_roll_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
                break;
            case PITCH:
                updating_angle_p_down(tune_pitch_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
                break;
            case YAW:
                updating_angle_p_down(tune_yaw_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
                break;
            }
            break;
        // Check results after mini-step to increase stabilize P gain
        case SP_UP:
            switch (axis) {
            case ROLL:
                updating_angle_p_up(tune_roll_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
                break;
            case PITCH:
                updating_angle_p_up(tune_pitch_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
                break;
            case YAW:
                updating_angle_p_up(tune_yaw_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
                break;
            }
            break;
        }

        // we've complete this step, finalize pids and move to next step
        if (counter >= AUTOTUNE_SUCCESS_COUNT) {

            // reset counter
            counter = 0;

            // reset scaling factor
            step_scaler = 1;

            // move to the next tuning type
            switch (tune_type) {
            case RD_UP:
                tune_type = TuneType(tune_type + 1);
                break;
            case RD_DOWN:
                tune_type = TuneType(tune_type + 1);
                switch (axis) {
                case ROLL:
                    tune_roll_rd = MAX(min_d, tune_roll_rd * AUTOTUNE_RD_BACKOFF);
                    tune_roll_rp = MAX(AUTOTUNE_RP_MIN, tune_roll_rp * AUTOTUNE_RD_BACKOFF);
                    break;
                case PITCH:
                    tune_pitch_rd = MAX(min_d, tune_pitch_rd * AUTOTUNE_RD_BACKOFF);
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
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_ROLL;
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
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_PITCH;
                    tune_pitch_sp = MAX(AUTOTUNE_SP_MIN, tune_pitch_sp * AUTOTUNE_SP_BACKOFF);
                    tune_pitch_accel = MAX(AUTOTUNE_RP_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    if (yaw_enabled()) {
                        axis = YAW;
                    } else {
                        complete = true;
                    }
                    break;
                case YAW:
                    axes_completed |= AUTOTUNE_AXIS_BITMASK_YAW;
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
                    Log_Write_Event(EVENT_AUTOTUNE_SUCCESS);
                    AP_Notify::events.autotune_complete = true;
                } else {
                    AP_Notify::events.autotune_next_axis = true;
                }
                break;
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

    current_gain_type = GAIN_ORIGINAL;
    positive_direction = false;
    step = WAITING_FOR_LEVEL;
    step_start_time_ms = AP_HAL::millis();
    level_start_time_ms = step_start_time_ms;
    tune_type = RD_UP;
    step_scaler = 1;

    desired_yaw_cd = ahrs_view->yaw_sensor;

    aggressiveness = constrain_float(aggressiveness, 0.05f, 0.2f);

    orig_bf_feedforward = attitude_control->get_bf_feedforward();

    // backup original pids and initialise tuned pid values
    orig_roll_rp = attitude_control->get_rate_roll_pid().kP();
    orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
    orig_roll_rd = attitude_control->get_rate_roll_pid().kD();
    orig_roll_rff = attitude_control->get_rate_roll_pid().ff();
    orig_roll_sp = attitude_control->get_angle_roll_p().kP();
    orig_roll_accel = attitude_control->get_accel_roll_max();
    tune_roll_rp = attitude_control->get_rate_roll_pid().kP();
    tune_roll_rd = attitude_control->get_rate_roll_pid().kD();
    tune_roll_sp = attitude_control->get_angle_roll_p().kP();
    tune_roll_accel = attitude_control->get_accel_roll_max();

    orig_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
    orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
    orig_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
    orig_pitch_rff = attitude_control->get_rate_pitch_pid().ff();
    orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
    orig_pitch_accel = attitude_control->get_accel_pitch_max();
    tune_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
    tune_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
    tune_pitch_sp = attitude_control->get_angle_pitch_p().kP();
    tune_pitch_accel = attitude_control->get_accel_pitch_max();

    orig_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
    orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
    orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
    orig_yaw_rff = attitude_control->get_rate_yaw_pid().ff();
    orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_hz();
    orig_yaw_accel = attitude_control->get_accel_yaw_max();
    orig_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    tune_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
    tune_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_hz();
    tune_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    tune_yaw_accel = attitude_control->get_accel_yaw_max();

    Log_Write_Event(EVENT_AUTOTUNE_INITIALISED);
}

// load_orig_gains - set gains to their original values
//  called by stop and failed functions
void AC_AutoTune::load_orig_gains()
{
    attitude_control->bf_feedforward(orig_bf_feedforward);
    if (roll_enabled()) {
        if (!is_zero(orig_roll_rp)) {
            attitude_control->get_rate_roll_pid().kP(orig_roll_rp);
            attitude_control->get_rate_roll_pid().kI(orig_roll_ri);
            attitude_control->get_rate_roll_pid().kD(orig_roll_rd);
            attitude_control->get_rate_roll_pid().ff(orig_roll_rff);
            attitude_control->get_angle_roll_p().kP(orig_roll_sp);
            attitude_control->set_accel_roll_max(orig_roll_accel);
        }
    }
    if (pitch_enabled()) {
        if (!is_zero(orig_pitch_rp)) {
            attitude_control->get_rate_pitch_pid().kP(orig_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(orig_pitch_ri);
            attitude_control->get_rate_pitch_pid().kD(orig_pitch_rd);
            attitude_control->get_rate_pitch_pid().ff(orig_pitch_rff);
            attitude_control->get_angle_pitch_p().kP(orig_pitch_sp);
            attitude_control->set_accel_pitch_max(orig_pitch_accel);
        }
    }
    if (yaw_enabled()) {
        if (!is_zero(orig_yaw_rp)) {
            attitude_control->get_rate_yaw_pid().kP(orig_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(orig_yaw_ri);
            attitude_control->get_rate_yaw_pid().kD(orig_yaw_rd);
            attitude_control->get_rate_yaw_pid().ff(orig_yaw_rff);
            attitude_control->get_rate_yaw_pid().filt_hz(orig_yaw_rLPF);
            attitude_control->get_angle_yaw_p().kP(orig_yaw_sp);
            attitude_control->set_accel_yaw_max(orig_yaw_accel);
        }
    }
}

// load_tuned_gains - load tuned gains
void AC_AutoTune::load_tuned_gains()
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
            attitude_control->get_rate_roll_pid().ff(orig_roll_rff);
            attitude_control->get_angle_roll_p().kP(tune_roll_sp);
            attitude_control->set_accel_roll_max(tune_roll_accel);
        }
    }
    if (pitch_enabled()) {
        if (!is_zero(tune_pitch_rp)) {
            attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
            attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
            attitude_control->get_rate_pitch_pid().ff(orig_pitch_rff);
            attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
            attitude_control->set_accel_pitch_max(tune_pitch_accel);
        }
    }
    if (yaw_enabled()) {
        if (!is_zero(tune_yaw_rp)) {
            attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
            attitude_control->get_rate_yaw_pid().kD(0.0f);
            attitude_control->get_rate_yaw_pid().ff(orig_yaw_rff);
            attitude_control->get_rate_yaw_pid().filt_hz(tune_yaw_rLPF);
            attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
            attitude_control->set_accel_yaw_max(tune_yaw_accel);
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
        attitude_control->get_rate_roll_pid().kI(orig_roll_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_roll_pid().kD(orig_roll_rd);
        attitude_control->get_rate_roll_pid().ff(orig_roll_rff);
        attitude_control->get_angle_roll_p().kP(orig_roll_sp);
    }
    if (pitch_enabled()) {
        attitude_control->get_rate_pitch_pid().kP(orig_pitch_rp);
        attitude_control->get_rate_pitch_pid().kI(orig_pitch_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_pitch_pid().kD(orig_pitch_rd);
        attitude_control->get_rate_pitch_pid().ff(orig_pitch_rff);
        attitude_control->get_angle_pitch_p().kP(orig_pitch_sp);
    }
    if (yaw_enabled()) {
        attitude_control->get_rate_yaw_pid().kP(orig_yaw_rp);
        attitude_control->get_rate_yaw_pid().kI(orig_yaw_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_yaw_pid().kD(orig_yaw_rd);
        attitude_control->get_rate_yaw_pid().ff(orig_yaw_rff);
        attitude_control->get_rate_yaw_pid().filt_hz(orig_yaw_rLPF);
        attitude_control->get_angle_yaw_p().kP(orig_yaw_sp);
    }
}

// load_twitch_gains - load the to-be-tested gains for a single axis
// called by control_attitude() just before it beings testing a gain (i.e. just before it twitches)
void AC_AutoTune::load_twitch_gains()
{
    switch (axis) {
    case ROLL:
        attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
        attitude_control->get_rate_roll_pid().kI(tune_roll_rp*0.01f);
        attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
        attitude_control->get_rate_roll_pid().ff(0.0f);
        attitude_control->get_angle_roll_p().kP(tune_roll_sp);
        break;
    case PITCH:
        attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
        attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*0.01f);
        attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
        attitude_control->get_rate_pitch_pid().ff(0.0f);
        attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
        break;
    case YAW:
        attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
        attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*0.01f);
        attitude_control->get_rate_yaw_pid().kD(0.0f);
        attitude_control->get_rate_yaw_pid().ff(0.0f);
        attitude_control->get_rate_yaw_pid().filt_hz(tune_yaw_rLPF);
        attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
        break;
    }
}

/*
  load a specified set of gains
 */
void AC_AutoTune::load_gains(enum GainType gain_type)
{
    if (current_gain_type == gain_type) {
        return;
    }
    switch (gain_type) {
    case GAIN_ORIGINAL:
        load_orig_gains();
        break;
    case GAIN_INTRA_TEST:
        load_intra_test_gains();
        break;
    case GAIN_TWITCH:
        load_twitch_gains();
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
        attitude_control->save_accel_roll_max(0.0f);
        attitude_control->save_accel_pitch_max(0.0f);
    }

    // sanity check the rate P values
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL) && roll_enabled() && !is_zero(tune_roll_rp)) {
        // rate roll gains
        attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
        attitude_control->get_rate_roll_pid().kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
        attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
        attitude_control->get_rate_roll_pid().ff(orig_roll_rff);
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
        orig_roll_rff = attitude_control->get_rate_roll_pid().ff();
        orig_roll_sp = attitude_control->get_angle_roll_p().kP();
        orig_roll_accel = attitude_control->get_accel_roll_max();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) && pitch_enabled() && !is_zero(tune_pitch_rp)) {
        // rate pitch gains
        attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
        attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
        attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
        attitude_control->get_rate_pitch_pid().ff(orig_pitch_rff);
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
        orig_pitch_rff = attitude_control->get_rate_pitch_pid().ff();
        orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
        orig_pitch_accel = attitude_control->get_accel_pitch_max();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_YAW) && yaw_enabled() && !is_zero(tune_yaw_rp)) {
        // rate yaw gains
        attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
        attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
        attitude_control->get_rate_yaw_pid().kD(0.0f);
        attitude_control->get_rate_yaw_pid().ff(orig_yaw_rff);
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
        orig_yaw_rff = attitude_control->get_rate_yaw_pid().ff();
        orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_hz();
        orig_yaw_sp = attitude_control->get_angle_yaw_p().kP();
        orig_yaw_accel = attitude_control->get_accel_pitch_max();
    }

    // update GCS and log save gains event
    update_gcs(AUTOTUNE_MESSAGE_SAVED_GAINS);
    Log_Write_Event(EVENT_AUTOTUNE_SAVEDGAINS);

    // reset Autotune so that gains are not saved again and autotune can be run again.
    mode = UNINITIALISED;
    axes_completed = 0;
}

// update_gcs - send message to ground station
void AC_AutoTune::update_gcs(uint8_t message_id)
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
    case AUTOTUNE_MESSAGE_SAVED_GAINS:
        gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: Saved gains for %s%s%s",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_ROLL)?"Roll ":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_PITCH)?"Pitch ":"",
                        (axes_completed&AUTOTUNE_AXIS_BITMASK_YAW)?"Yaw":"");
        break;
    }
}

// axis helper functions
inline bool AC_AutoTune::roll_enabled()
{
    return axis_bitmask & AUTOTUNE_AXIS_BITMASK_ROLL;
}

inline bool AC_AutoTune::pitch_enabled()
{
    return axis_bitmask & AUTOTUNE_AXIS_BITMASK_PITCH;
}

inline bool AC_AutoTune::yaw_enabled()
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
        if (is_equal(rate, meas_rate_min) && step_scaler > 0.5) {
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
void AC_AutoTune::twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max)
{
    if (rate_measurement_max < rate_measurement) {
        rate_measurement_max = rate_measurement;
        rate_of_change = (1000.0f*rate_measurement_max)/(AP_HAL::millis() - step_start_time_ms);
    }
}

// updating_rate_d_up - increase D and adjust P to optimize the D term for a little bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void AC_AutoTune::updating_rate_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max)
{
    if (meas_rate_max > rate_target) {
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
                Log_Write_Event(EVENT_AUTOTUNE_REACHED_LIMIT);
            }
        }
    } else if ((meas_rate_max < rate_target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            Log_Write_Event(EVENT_AUTOTUNE_REACHED_LIMIT);
        }
    } else {
        // we have a good measurement of bounce back
        if (meas_rate_max-meas_rate_min > meas_rate_max*aggressiveness) {
            // ignore the next result unless it is the same as this one
            ignore_next = true;
            // bounce back is bigger than our threshold so increment the success counter
            counter++;
        } else {
            if (ignore_next == false) {
                // bounce back is smaller than our threshold so decrement the success counter
                if (counter > 0) {
                    counter--;
                }
                // increase D gain (which should increase bounce back)
                tune_d += tune_d*tune_d_step_ratio*2.0f;
                // stop tuning if we hit maximum D
                if (tune_d >= tune_d_max) {
                    tune_d = tune_d_max;
                    counter = AUTOTUNE_SUCCESS_COUNT;
                    Log_Write_Event(EVENT_AUTOTUNE_REACHED_LIMIT);
                }
            } else {
                ignore_next = false;
            }
        }
    }
}

// updating_rate_d_down - decrease D and adjust P to optimize the D term for no bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void AC_AutoTune::updating_rate_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max)
{
    if (meas_rate_max > rate_target) {
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
                Log_Write_Event(EVENT_AUTOTUNE_REACHED_LIMIT);
            }
        }
    } else if ((meas_rate_max < rate_target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            Log_Write_Event(EVENT_AUTOTUNE_REACHED_LIMIT);
        }
    } else {
        // we have a good measurement of bounce back
        if (meas_rate_max-meas_rate_min < meas_rate_max*aggressiveness) {
            if (ignore_next == false) {
                // bounce back is less than our threshold so increment the success counter
                counter++;
            } else {
                ignore_next = false;
            }
        } else {
            // ignore the next result unless it is the same as this one
            ignore_next = true;
            // bounce back is larger than our threshold so decrement the success counter
            if (counter > 0) {
                counter--;
            }
            // decrease D gain (which should decrease bounce back)
            tune_d -= tune_d*tune_d_step_ratio;
            // stop tuning if we hit minimum D
            if (tune_d <= tune_d_min) {
                tune_d = tune_d_min;
                counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(EVENT_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }
}

// updating_rate_p_up_d_down - increase P to ensure the target is reached while checking bounce back isn't increasing
// P is increased until we achieve our target within a reasonable time while reducing D if bounce back increases above the threshold
void AC_AutoTune::updating_rate_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max)
{
    if (meas_rate_max > rate_target*(1+0.5f*aggressiveness)) {
        // ignore the next result unless it is the same as this one
        ignore_next = true;
        // if maximum measurement was greater than target so increment the success counter
        counter++;
    } else if ((meas_rate_max < rate_target) && (meas_rate_max > rate_target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (meas_rate_max-meas_rate_min > meas_rate_max*aggressiveness) && (tune_d > tune_d_min)) {
        // if bounce back was larger than the threshold so decrement the success counter
        if (counter > 0) {
            counter--;
        }
        // decrease D gain (which should decrease bounce back)
        tune_d -= tune_d*tune_d_step_ratio;
        // do not decrease the D term past the minimum
        if (tune_d <= tune_d_min) {
            tune_d = tune_d_min;
            Log_Write_Event(EVENT_AUTOTUNE_REACHED_LIMIT);
        }
        // decrease P gain to match D gain reduction
        tune_p -= tune_p*tune_p_step_ratio;
        // do not decrease the P term past the minimum
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            Log_Write_Event(EVENT_AUTOTUNE_REACHED_LIMIT);
        }
        // cancel change in direction
        positive_direction = !positive_direction;
    } else {
        if (ignore_next == false) {
            // if maximum measurement was lower than target so decrement the success counter
            if (counter > 0) {
                counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p*tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(EVENT_AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            ignore_next = false;
        }
    }
}

// updating_angle_p_down - decrease P until we don't reach the target before time out
// P is decreased to ensure we are not overshooting the target
void AC_AutoTune::updating_angle_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max)
{
    if (meas_angle_max < angle_target*(1+0.5f*aggressiveness)) {
        if (ignore_next == false) {
            // if maximum measurement was lower than target so increment the success counter
            counter++;
        } else {
            ignore_next = false;
        }
    } else {
        // ignore the next result unless it is the same as this one
        ignore_next = true;
        // if maximum measurement was higher than target so decrement the success counter
        if (counter > 0) {
            counter--;
        }
        // decrease P gain (which should decrease the maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        // stop tuning if we hit maximum P
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            counter = AUTOTUNE_SUCCESS_COUNT;
            Log_Write_Event(EVENT_AUTOTUNE_REACHED_LIMIT);
        }
    }
}

// updating_angle_p_up - increase P to ensure the target is reached
// P is increased until we achieve our target within a reasonable time
void AC_AutoTune::updating_angle_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max)
{
    if ((meas_angle_max > angle_target*(1+0.5f*aggressiveness)) ||
        ((meas_angle_max > angle_target) && (meas_rate_min < -meas_rate_max*aggressiveness))) {
        // ignore the next result unless it is the same as this one
        ignore_next = true;
        // if maximum measurement was greater than target so increment the success counter
        counter++;
    } else {
        if (ignore_next == false) {
            // if maximum measurement was lower than target so decrement the success counter
            if (counter > 0) {
                counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p*tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(EVENT_AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            ignore_next = false;
        }
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
        start_position = inertial_nav->get_position();
    }

    // don't go past 10 degrees, as autotune result would deteriorate too much
    const float angle_max_cd = 1000;

    // hit the 10 degree limit at 20 meters position error
    const float dist_limit_cm = 2000;

    // we only start adjusting yaw if we are more than 5m from the
    // target position. That corresponds to a lean angle of 2.5 degrees
    const float yaw_dist_limit_cm = 500;

    Vector3f pdiff = inertial_nav->get_position() - start_position;
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

// Write an Autotune data packet
void AC_AutoTune::Log_Write_AutoTune(uint8_t _axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt)
{
    AP::logger().Write(
        "ATUN",
        "TimeUS,Axis,TuneStep,Targ,Min,Max,RP,RD,SP,ddt",
        "s--ddd---o",
        "F--BBB---0",
        "QBBfffffff",
        AP_HAL::micros64(),
        axis,
        tune_step,
        meas_target*0.01f,
        meas_min*0.01f,
        meas_max*0.01f,
        new_gain_rp,
        new_gain_rd,
        new_gain_sp,
        new_ddt);
}

// Write an Autotune data packet
void AC_AutoTune::Log_Write_AutoTuneDetails(float angle_cd, float rate_cds)
{
    AP::logger().Write(
        "ATDE",
        "TimeUS,Angle,Rate",
        "sdk",
        "FBB",
        "Qff",
        AP_HAL::micros64(),
        angle_cd*0.01f,
        rate_cds*0.01f);
}
