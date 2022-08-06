#include "AC_AutoTune_Multi.h"

#include <AP_Logger/AP_Logger.h>

/*
 * autotune support for multicopters
 *
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

#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS   1000U     // timeout for tuning mode's testing step

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
#define AUTOTUNE_SP_MAX                   40.0f     // maximum Stab P value
#define AUTOTUNE_SP_MIN                    0.5f     // maximum Stab P value
#define AUTOTUNE_RP_ACCEL_MIN            4000.0f     // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_Y_ACCEL_MIN             1000.0f     // Minimum acceleration for Yaw
#define AUTOTUNE_Y_FILT_FREQ              10.0f     // Autotune filter frequency when testing Yaw
#define AUTOTUNE_D_UP_DOWN_MARGIN          0.2f     // The margin below the target that we tune D in
#define AUTOTUNE_RD_BACKOFF                1.0f     // Rate D gains are reduced to 50% of their maximum value discovered during tuning
#define AUTOTUNE_RP_BACKOFF                1.0f     // Rate P gains are reduced to 97.5% of their maximum value discovered during tuning
#define AUTOTUNE_SP_BACKOFF                0.9f     // Stab P gains are reduced to 90% of their maximum value discovered during tuning
#define AUTOTUNE_ACCEL_RP_BACKOFF          1.0f     // back off from maximum acceleration
#define AUTOTUNE_ACCEL_Y_BACKOFF           1.0f     // back off from maximum acceleration

// roll and pitch axes
#define AUTOTUNE_TARGET_RATE_RLLPIT_CDS     18000   // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS 4500    // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step

// yaw axis
#define AUTOTUNE_TARGET_RATE_YAW_CDS        9000    // target yaw rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_ANGLE_YAW_CD     500    // minimum target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_MIN_RATE_YAW_CDS    1500    // minimum target yaw rate during AUTOTUNE_STEP_TWITCHING step

// second table of user settable parameters for quadplanes, this
// allows us to go beyond the 64 parameter limit
const AP_Param::GroupInfo AC_AutoTune_Multi::var_info[] = {

    // @Param: AXES
    // @DisplayName: Autotune axis bitmask
    // @Description: 1-byte bitmap of axes to autotune
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw
    // @User: Standard
    AP_GROUPINFO("AXES", 1, AC_AutoTune_Multi, axis_bitmask,  7),  // AUTOTUNE_AXIS_BITMASK_DEFAULT

    // @Param: AGGR
    // @DisplayName: Autotune aggressiveness
    // @Description: Autotune aggressiveness. Defines the bounce back used to detect size of the D term.
    // @Range: 0.05 0.10
    // @User: Standard
    AP_GROUPINFO("AGGR", 2, AC_AutoTune_Multi, aggressiveness, 0.1f),

    // @Param: MIN_D
    // @DisplayName: AutoTune minimum D
    // @Description: Defines the minimum D gain
    // @Range: 0.001 0.006
    // @User: Standard
    AP_GROUPINFO("MIN_D", 3, AC_AutoTune_Multi, min_d,  0.001f),

    AP_GROUPEND
};

// constructor
AC_AutoTune_Multi::AC_AutoTune_Multi()
{
    tune_seq[0] = TUNE_COMPLETE;
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_AutoTune_Multi::do_gcs_announcements()
{
    const uint32_t now = AP_HAL::millis();
    if (now - announce_time < AUTOTUNE_ANNOUNCE_INTERVAL_MS) {
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: %s %s %u%%", axis_string(), type_string(), (counter * (100/AUTOTUNE_SUCCESS_COUNT)) );
    announce_time = now;
}

void AC_AutoTune_Multi::test_init()
{
    twitch_test_init();
}

void AC_AutoTune_Multi::test_run(AxisType test_axis, const float dir_sign)
{
    twitch_test_run(test_axis, dir_sign);
}

// backup_gains_and_initialise - store current gains as originals
//  called before tuning starts to backup original gains
void AC_AutoTune_Multi::backup_gains_and_initialise()
{
    AC_AutoTune::backup_gains_and_initialise();

    aggressiveness.set(constrain_float(aggressiveness, 0.05f, 0.2f));

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
    tune_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_E_hz();
    tune_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    tune_yaw_accel = attitude_control->get_accel_yaw_max_cdss();

    AP::logger().Write_Event(LogEvent::AUTOTUNE_INITIALISED);
}

// load_orig_gains - set gains to their original values
//  called by stop and failed functions
void AC_AutoTune_Multi::load_orig_gains()
{
    attitude_control->bf_feedforward(orig_bf_feedforward);
    if (roll_enabled()) {
        if (!is_zero(orig_roll_rp)) {
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
        if (!is_zero(orig_pitch_rp)) {
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
void AC_AutoTune_Multi::load_tuned_gains()
{
    if (!attitude_control->get_bf_feedforward()) {
        attitude_control->bf_feedforward(true);
        attitude_control->set_accel_roll_max_cdss(0.0f);
        attitude_control->set_accel_pitch_max_cdss(0.0f);
    }
    if (roll_enabled()) {
        if (!is_zero(tune_roll_rp)) {
            attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
            attitude_control->get_rate_roll_pid().kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
            attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
            attitude_control->get_rate_roll_pid().ff(orig_roll_rff);
            attitude_control->get_angle_roll_p().kP(tune_roll_sp);
            attitude_control->set_accel_roll_max_cdss(tune_roll_accel);
        }
    }
    if (pitch_enabled()) {
        if (!is_zero(tune_pitch_rp)) {
            attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
            attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
            attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
            attitude_control->get_rate_pitch_pid().ff(orig_pitch_rff);
            attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
            attitude_control->set_accel_pitch_max_cdss(tune_pitch_accel);
        }
    }
    if (yaw_enabled()) {
        if (!is_zero(tune_yaw_rp)) {
            attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
            attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
            attitude_control->get_rate_yaw_pid().kD(0.0f);
            attitude_control->get_rate_yaw_pid().ff(orig_yaw_rff);
            attitude_control->get_rate_yaw_pid().filt_E_hz(tune_yaw_rLPF);
            attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
            attitude_control->set_accel_yaw_max_cdss(tune_yaw_accel);
        }
    }
}

// load_intra_test_gains - gains used between tests
//  called during testing mode's update-gains step to set gains ahead of return-to-level step
void AC_AutoTune_Multi::load_intra_test_gains()
{
    // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
    // sanity check the gains
    attitude_control->bf_feedforward(true);
    if (roll_enabled()) {
        attitude_control->get_rate_roll_pid().kP(orig_roll_rp);
        attitude_control->get_rate_roll_pid().kI(orig_roll_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_roll_pid().kD(orig_roll_rd);
        attitude_control->get_rate_roll_pid().ff(orig_roll_rff);
        attitude_control->get_rate_roll_pid().filt_T_hz(orig_roll_fltt);
        attitude_control->get_rate_roll_pid().slew_limit(orig_roll_smax);
        attitude_control->get_angle_roll_p().kP(orig_roll_sp);
    }
    if (pitch_enabled()) {
        attitude_control->get_rate_pitch_pid().kP(orig_pitch_rp);
        attitude_control->get_rate_pitch_pid().kI(orig_pitch_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_pitch_pid().kD(orig_pitch_rd);
        attitude_control->get_rate_pitch_pid().ff(orig_pitch_rff);
        attitude_control->get_rate_pitch_pid().filt_T_hz(orig_pitch_fltt);
        attitude_control->get_rate_pitch_pid().slew_limit(orig_pitch_smax);
        attitude_control->get_angle_pitch_p().kP(orig_pitch_sp);
    }
    if (yaw_enabled()) {
        attitude_control->get_rate_yaw_pid().kP(orig_yaw_rp);
        attitude_control->get_rate_yaw_pid().kI(orig_yaw_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
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
void AC_AutoTune_Multi::load_test_gains()
{
    switch (axis) {
    case ROLL:
        attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
        attitude_control->get_rate_roll_pid().kI(tune_roll_rp*0.01f);
        attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
        attitude_control->get_rate_roll_pid().ff(0.0f);
        attitude_control->get_rate_roll_pid().filt_T_hz(0.0f);
        attitude_control->get_rate_roll_pid().slew_limit(0.0f);
        attitude_control->get_angle_roll_p().kP(tune_roll_sp);
        break;
    case PITCH:
        attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
        attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*0.01f);
        attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
        attitude_control->get_rate_pitch_pid().ff(0.0f);
        attitude_control->get_rate_pitch_pid().filt_T_hz(0.0f);
        attitude_control->get_rate_pitch_pid().slew_limit(0.0f);
        attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
        break;
    case YAW:
        attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
        attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*0.01f);
        attitude_control->get_rate_yaw_pid().kD(0.0f);
        attitude_control->get_rate_yaw_pid().ff(0.0f);
        attitude_control->get_rate_yaw_pid().filt_E_hz(tune_yaw_rLPF);
        attitude_control->get_rate_yaw_pid().filt_T_hz(0.0f);
        attitude_control->get_rate_yaw_pid().slew_limit(0.0f);
        attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
        break;
    }
}

// save_tuning_gains - save the final tuned gains for each axis
// save discovered gains to eeprom if autotuner is enabled (i.e. switch is in the high position)
void AC_AutoTune_Multi::save_tuning_gains()
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
        attitude_control->get_rate_roll_pid().kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
        attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
        attitude_control->get_rate_roll_pid().ff(orig_roll_rff);
        attitude_control->get_rate_roll_pid().filt_T_hz(orig_roll_fltt);
        attitude_control->get_rate_roll_pid().slew_limit(orig_roll_smax);
        attitude_control->get_rate_roll_pid().save_gains();

        // stabilize roll
        attitude_control->get_angle_roll_p().kP(tune_roll_sp);
        attitude_control->get_angle_roll_p().save_gains();

        // acceleration roll
        attitude_control->save_accel_roll_max_cdss(tune_roll_accel);

        // resave pids to originals in case the autotune is run again
        orig_roll_rp = attitude_control->get_rate_roll_pid().kP();
        orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
        orig_roll_rd = attitude_control->get_rate_roll_pid().kD();
        orig_roll_rff = attitude_control->get_rate_roll_pid().ff();
        orig_roll_sp = attitude_control->get_angle_roll_p().kP();
        orig_roll_accel = attitude_control->get_accel_roll_max_cdss();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) && pitch_enabled() && !is_zero(tune_pitch_rp)) {
        // rate pitch gains
        attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
        attitude_control->get_rate_pitch_pid().kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
        attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
        attitude_control->get_rate_pitch_pid().ff(orig_pitch_rff);
        attitude_control->get_rate_pitch_pid().filt_T_hz(orig_pitch_fltt);
        attitude_control->get_rate_pitch_pid().slew_limit(orig_pitch_smax);
        attitude_control->get_rate_pitch_pid().save_gains();

        // stabilize pitch
        attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
        attitude_control->get_angle_pitch_p().save_gains();

        // acceleration pitch
        attitude_control->save_accel_pitch_max_cdss(tune_pitch_accel);

        // resave pids to originals in case the autotune is run again
        orig_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
        orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
        orig_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
        orig_pitch_rff = attitude_control->get_rate_pitch_pid().ff();
        orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
        orig_pitch_accel = attitude_control->get_accel_pitch_max_cdss();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_YAW) && yaw_enabled() && !is_zero(tune_yaw_rp)) {
        // rate yaw gains
        attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
        attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
        attitude_control->get_rate_yaw_pid().kD(0.0f);
        attitude_control->get_rate_yaw_pid().ff(orig_yaw_rff);
        attitude_control->get_rate_yaw_pid().filt_T_hz(orig_yaw_fltt);
        attitude_control->get_rate_yaw_pid().slew_limit(orig_yaw_smax);
        attitude_control->get_rate_yaw_pid().filt_E_hz(tune_yaw_rLPF);
        attitude_control->get_rate_yaw_pid().save_gains();

        // stabilize yaw
        attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
        attitude_control->get_angle_yaw_p().save_gains();

        // acceleration yaw
        attitude_control->save_accel_yaw_max_cdss(tune_yaw_accel);

        // resave pids to originals in case the autotune is run again
        orig_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
        orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
        orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
        orig_yaw_rff = attitude_control->get_rate_yaw_pid().ff();
        orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_E_hz();
        orig_yaw_sp = attitude_control->get_angle_yaw_p().kP();
        orig_yaw_accel = attitude_control->get_accel_yaw_max_cdss();
    }

    // update GCS and log save gains event
    update_gcs(AUTOTUNE_MESSAGE_SAVED_GAINS);
    AP::logger().Write_Event(LogEvent::AUTOTUNE_SAVEDGAINS);

    reset();
}

// report final gains for a given axis to GCS
void AC_AutoTune_Multi::report_final_gains(AxisType test_axis) const
{
    switch (test_axis) {
        case ROLL:
            report_axis_gains("Roll", tune_roll_rp, tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL, tune_roll_rd, tune_roll_sp, tune_roll_accel);
            break;
        case PITCH:
            report_axis_gains("Pitch", tune_pitch_rp, tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL, tune_pitch_rd, tune_pitch_sp, tune_pitch_accel);
            break;
        case YAW:
            report_axis_gains("Yaw", tune_yaw_rp, tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL, 0, tune_yaw_sp, tune_yaw_accel);
            break;
    }
}

// report gain formating helper
void AC_AutoTune_Multi::report_axis_gains(const char* axis_string, float rate_P, float rate_I, float rate_D, float angle_P, float max_accel) const
{
    gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: %s complete", axis_string);
    gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: %s Rate: P:%0.3f, I:%0.3f, D:%0.4f",axis_string,rate_P,rate_I,rate_D);
    gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: %s Angle P:%0.3f, Max Accel:%0.0f",axis_string,angle_P,max_accel);
}

// twitching_test_rate - twitching tests
// update min and max and test for end conditions
void AC_AutoTune_Multi::twitching_test_rate(float rate, float rate_target_max, float &meas_rate_min, float &meas_rate_max)
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
void AC_AutoTune_Multi::twitching_abort_rate(float angle, float rate, float angle_max, float meas_rate_min)
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
void AC_AutoTune_Multi::twitching_test_angle(float angle, float rate, float angle_target_max, float &meas_angle_min, float &meas_angle_max, float &meas_rate_min, float &meas_rate_max)
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
void AC_AutoTune_Multi::twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max) const
{
    if (rate_measurement_max < rate_measurement) {
        rate_measurement_max = rate_measurement;
        rate_of_change = (1000.0f*rate_measurement_max)/(AP_HAL::millis() - step_start_time_ms);
    }
}

// update gains for the rate p up tune type
void AC_AutoTune_Multi::updating_rate_p_up_all(AxisType test_axis)
{
    switch (test_axis) {
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
}

// update gains for the rate d up tune type
void AC_AutoTune_Multi::updating_rate_d_up_all(AxisType test_axis)
{
    switch (test_axis) {
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
}

// update gains for the rate d down tune type
void AC_AutoTune_Multi::updating_rate_d_down_all(AxisType test_axis)
{
    switch (test_axis) {
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
}

// update gains for the angle p up tune type
void AC_AutoTune_Multi::updating_angle_p_up_all(AxisType test_axis)
{
    switch (test_axis) {
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
}

// update gains for the angle p down tune type
void AC_AutoTune_Multi::updating_angle_p_down_all(AxisType test_axis)
{
    switch (test_axis) {
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
}

// set gains post tune for the tune type
void AC_AutoTune_Multi::set_gains_post_tune(AxisType test_axis)
{
    switch (tune_type) {
    case RD_UP:
        break;
    case RD_DOWN:
        switch (test_axis) {
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
        switch (test_axis) {
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
        break;
    case SP_UP:
        switch (test_axis) {
        case ROLL:
            tune_roll_sp = MAX(AUTOTUNE_SP_MIN, tune_roll_sp * AUTOTUNE_SP_BACKOFF);
            tune_roll_accel = MAX(AUTOTUNE_RP_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
            break;
        case PITCH:
            tune_pitch_sp = MAX(AUTOTUNE_SP_MIN, tune_pitch_sp * AUTOTUNE_SP_BACKOFF);
            tune_pitch_accel = MAX(AUTOTUNE_RP_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
            break;
        case YAW:
            tune_yaw_sp = MAX(AUTOTUNE_SP_MIN, tune_yaw_sp * AUTOTUNE_SP_BACKOFF);
            tune_yaw_accel = MAX(AUTOTUNE_Y_ACCEL_MIN, test_accel_max * AUTOTUNE_ACCEL_Y_BACKOFF);
            break;
        }
        break;
    case RFF_UP:
    case MAX_GAINS:
        // this should never happen
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    case TUNE_COMPLETE:
        break;
    }
}

// updating_rate_d_up - increase D and adjust P to optimize the D term for a little bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void AC_AutoTune_Multi::updating_rate_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max)
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
                AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            }
        }
    } else if ((meas_rate_max < rate_target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
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
                    AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
                }
            } else {
                ignore_next = false;
            }
        }
    }
}

// updating_rate_d_down - decrease D and adjust P to optimize the D term for no bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void AC_AutoTune_Multi::updating_rate_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max)
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
                AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            }
        }
    } else if ((meas_rate_max < rate_target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
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
                AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            }
        }
    }
}

// updating_rate_p_up_d_down - increase P to ensure the target is reached while checking bounce back isn't increasing
// P is increased until we achieve our target within a reasonable time while reducing D if bounce back increases above the threshold
void AC_AutoTune_Multi::updating_rate_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max)
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
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
        }
        // decrease P gain to match D gain reduction
        tune_p -= tune_p*tune_p_step_ratio;
        // do not decrease the P term past the minimum
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
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
                AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            ignore_next = false;
        }
    }
}

// updating_angle_p_down - decrease P until we don't reach the target before time out
// P is decreased to ensure we are not overshooting the target
void AC_AutoTune_Multi::updating_angle_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max)
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
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
        }
    }
}

// updating_angle_p_up - increase P to ensure the target is reached
// P is increased until we achieve our target within a reasonable time
void AC_AutoTune_Multi::updating_angle_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max)
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
                AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            ignore_next = false;
        }
    }
}

void AC_AutoTune_Multi::Log_AutoTune()
{
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

}

void AC_AutoTune_Multi::Log_AutoTuneDetails()
{
    Log_Write_AutoTuneDetails(lean_angle, rotation_rate);
}

// @LoggerMessage: ATUN
// @Description: Copter/QuadPlane AutoTune
// @Vehicles: Copter, Plane
// @Field: TimeUS: Time since system startup
// @Field: Axis: which axis is currently being tuned
// @Field: TuneStep: step in autotune process
// @Field: Targ: target angle or rate, depending on tuning step
// @Field: Min: measured minimum target angle or rate
// @Field: Max: measured maximum target angle or rate
// @Field: RP: new rate gain P term
// @Field: RD: new rate gain D term
// @Field: SP: new angle P term
// @Field: ddt: maximum measured twitching acceleration

// Write an Autotune data packet
void AC_AutoTune_Multi::Log_Write_AutoTune(uint8_t _axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt)
{
    AP::logger().Write(
        "ATUN",
        "TimeUS,Axis,TuneStep,Targ,Min,Max,RP,RD,SP,ddt",
        "s--ddd---o",
        "F--000---0",
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
void AC_AutoTune_Multi::Log_Write_AutoTuneDetails(float angle_cd, float rate_cds)
{
    // @LoggerMessage: ATDE
    // @Description: AutoTune data packet
    // @Field: TimeUS: Time since system startup
    // @Field: Angle: current angle
    // @Field: Rate: current angular rate
    AP::logger().WriteStreaming(
        "ATDE",
        "TimeUS,Angle,Rate",
        "sdk",
        "F00",
        "Qff",
        AP_HAL::micros64(),
        angle_cd*0.01f,
        rate_cds*0.01f);
}

void AC_AutoTune_Multi::twitch_test_init()
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
void AC_AutoTune_Multi::twitch_test_run(AxisType test_axis, const float dir_sign)
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

    // capture this iteration's rotation rate and lean angle
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
    case RFF_UP:
    case MAX_GAINS:
        // this should never happen
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    default:
        break;
    }
}

// get_testing_step_timeout_ms accessor
uint32_t AC_AutoTune_Multi::get_testing_step_timeout_ms() const
{
    return AUTOTUNE_TESTING_STEP_TIMEOUT_MS;
}

