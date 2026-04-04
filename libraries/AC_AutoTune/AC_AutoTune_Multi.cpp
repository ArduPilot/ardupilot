#include "AC_AutoTune_config.h"

#if AC_AUTOTUNE_ENABLED

#include "AC_AutoTune_Multi.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

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

#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS   2000U     // timeout for tuning mode's testing step

#define AUTOTUNE_RD_STEP                  0.05     // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                  0.05     // minimum increment when increasing/decreasing Rate P term
#define AUTOTUNE_SP_STEP                  0.05     // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1     // I is set 10x smaller than P during testing
#define AUTOTUNE_PI_RATIO_FINAL            1.0     // I is set 1x P after testing
#define AUTOTUNE_YAW_PI_RATIO_FINAL        0.1     // I is set 1x P after testing
#define AUTOTUNE_RD_MAX                  0.200     // maximum Rate D value
#define AUTOTUNE_RLPF_MIN                  1.0     // minimum Rate Yaw filter value
#define AUTOTUNE_RLPF_MAX                  5.0     // maximum Rate Yaw filter value
#define AUTOTUNE_FLTE_MIN                  2.5     // minimum Rate Yaw error filter value
#define AUTOTUNE_RP_MIN                   0.01     // minimum Rate P value
#define AUTOTUNE_RP_MAX                    2.0     // maximum Rate P value

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 #define AUTOTUNE_SP_MAX                   10.0     // maximum Stab P value
#else
#define AUTOTUNE_SP_MAX                   40.0     // maximum Stab P value
#endif

#define AUTOTUNE_SP_MIN                    0.5     // maximum Stab P value
#define AUTOTUNE_RP_ACCEL_MIN            4000.0    // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_Y_ACCEL_MIN             1000.0    // Minimum acceleration for Yaw
#define AUTOTUNE_Y_FILT_FREQ              10.0     // Autotune filter frequency when testing Yaw
#define AUTOTUNE_D_UP_DOWN_MARGIN          0.2     // The margin below the target that we tune D in
#define AUTOTUNE_ACCEL_RP_BACKOFF          1.0     // back off from maximum acceleration
#define AUTOTUNE_ACCEL_Y_BACKOFF           1.0     // back off from maximum acceleration

// roll and pitch axes
#define AUTOTUNE_TARGET_RATE_RLLPIT_CDS     18000   // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS 4500    // target min roll/pitch rate during AUTOTUNE_STEP_TWITCHING step

// yaw axis
#define AUTOTUNE_TARGET_RATE_YAW_CDS        9000        // target yaw rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_RATE_YAW_CDS    1500        // minimum target yaw rate during AUTOTUNE_STEP_TWITCHING step

#define AUTOTUNE_TARGET_ANGLE_MAX_RP_SCALE  1.0 / 2.0   // minimum target angle, as a fraction of angle_max, during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_ANGLE_MAX_Y_SCALE   1.0         // minimum target angle, as a fraction of angle_max, during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_ANGLE_MIN_RP_SCALE  1.0 / 3.0   // minimum target angle, as a fraction of angle_max, during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_ANGLE_MIN_Y_SCALE   1.0 / 6.0   // minimum target angle, as a fraction of angle_max, during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_ANGLE_ABORT_RP_SCALE       2.5 / 3.0   // maximum allowable angle during testing, as a fraction of angle_max
#define AUTOTUNE_ANGLE_MAX_Y_SCALE          1.0         // maximum allowable angle during testing, as a fraction of angle_max
#define AUTOTUNE_ANGLE_NEG_RP_SCALE         1.0 / 5.0   // maximum allowable angle during testing, as a fraction of angle_max

// second table of user settable parameters for quadplanes, this
// allows us to go beyond the 64 parameter limit
const AP_Param::GroupInfo AC_AutoTune_Multi::var_info[] = {

    // @Param: AXES
    // @DisplayName: Autotune axis bitmask
    // @Description: 1-byte bitmap of axes to autotune
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw,3:YawD
    // @User: Standard
    AP_GROUPINFO("AXES", 1, AC_AutoTune_Multi, axis_bitmask,  7),  // AUTOTUNE_AXIS_BITMASK_DEFAULT

    // @Param: AGGR
    // @DisplayName: Autotune aggressiveness
    // @Description: Autotune aggressiveness. Defines the bounce back used to detect size of the D term.
    // @Range: 0.05 0.10
    // @User: Standard
    AP_GROUPINFO("AGGR", 2, AC_AutoTune_Multi, aggressiveness, 0.075f),

    // @Param: MIN_D
    // @DisplayName: AutoTune minimum D
    // @Description: Defines the minimum D gain
    // @Range: 0.0001 0.005
    // @User: Standard
    AP_GROUPINFO("MIN_D", 3, AC_AutoTune_Multi, min_d,  0.0005f),

    // @Param: GMBK
    // @DisplayName: AutoTune Gain Margin Backoff
    // @Description: Fraction by which tuned P and D gains are reduced after rate and angle AutoTune steps complete. This provides extra stability margin by reducing gains slightly from the optimal values found during tuning. A value of 0.0 applies no reduction. A value of 0.25 reduces tuned gains by 25%.
    // @Range: 0.0 0.5
    // @User: Standard
    AP_GROUPINFO("GMBK", 4, AC_AutoTune_Multi, gain_backoff,  0.25),

    AP_GROUPEND
};

// constructor
AC_AutoTune_Multi::AC_AutoTune_Multi()
{
    tune_seq[0] = TuneType::TUNE_COMPLETE;
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_AutoTune_Multi::do_gcs_announcements()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_announce_ms < AUTOTUNE_ANNOUNCE_INTERVAL_MS) {
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: %s %s %u%%", get_axis_name(), get_tune_type_name(), (success_counter * (100/AUTOTUNE_SUCCESS_COUNT)));
    last_announce_ms = now_ms;
}

// Prepares all tuning state variables and target values for a new twitch test.
void AC_AutoTune_Multi::test_init()
{
    float target_max_rate;
    switch (axis) {
    case AxisType::ROLL:
        angle_abort = target_angle_max_rp_cd();
        target_max_rate = MAX(AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, step_scaler * AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
        target_rate = constrain_float(degrees(attitude_control->max_rate_step_bf_roll()) * 100.0, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, target_max_rate);
        target_angle = constrain_float(degrees(attitude_control->max_angle_step_bf_roll()) * 100.0, target_angle_min_rp_cd(), target_angle_max_rp_cd());
        rotation_rate_filt.set_cutoff_frequency(attitude_control->get_rate_roll_pid().filt_D_hz() * 2.0);
        break;

    case AxisType::PITCH:
        angle_abort = target_angle_max_rp_cd();
        target_max_rate = MAX(AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, step_scaler * AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
        target_rate = constrain_float(degrees(attitude_control->max_rate_step_bf_pitch()) * 100.0, AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, target_max_rate);
        target_angle = constrain_float(degrees(attitude_control->max_angle_step_bf_pitch()) * 100.0, target_angle_min_rp_cd(), target_angle_max_rp_cd());
        rotation_rate_filt.set_cutoff_frequency(attitude_control->get_rate_pitch_pid().filt_D_hz() * 2.0);
        break;

    case AxisType::YAW:
    case AxisType::YAW_D:
        angle_abort = target_angle_max_y_cd();
        target_max_rate = MAX(AUTOTUNE_TARGET_MIN_RATE_YAW_CDS, step_scaler*AUTOTUNE_TARGET_RATE_YAW_CDS);
        target_rate = constrain_float(degrees(attitude_control->max_rate_step_bf_yaw() * 0.75) * 100.0, AUTOTUNE_TARGET_MIN_RATE_YAW_CDS, target_max_rate);
        target_angle = constrain_float(degrees(attitude_control->max_angle_step_bf_yaw() * 0.75) * 100.0, target_angle_min_y_cd(), target_angle_max_y_cd());
        if (axis == AxisType::YAW_D) {
            rotation_rate_filt.set_cutoff_frequency(attitude_control->get_rate_yaw_pid().filt_D_hz() * 2.0);
        } else {
            rotation_rate_filt.set_cutoff_frequency(AUTOTUNE_Y_FILT_FREQ);
        }
        break;
    }

    if ((tune_type == TuneType::ANGLE_P_DOWN) || (tune_type == TuneType::ANGLE_P_UP)) {
        // TODO: Evaluate whether initializing rotation_rate_filt with start_rate improves results for other axes
        rotation_rate_filt.reset(start_rate);
    } else {
        rotation_rate_filt.reset(0.0);
    }
    angle_step_commanded = false;
    test_rate_max = 0.0;
    test_rate_min = 0.0;
    test_angle_max = 0.0;
    test_angle_min = 0.0;
    accel_measure_rate_max = 0.0;
}

void AC_AutoTune_Multi::test_run(AxisType test_axis, const float dir_sign)
{
    // hold current attitude

    if ((tune_type == TuneType::ANGLE_P_DOWN) || (tune_type == TuneType::ANGLE_P_UP)) {
        // step angle targets on first iteration
        if (!angle_step_commanded) {
            angle_step_commanded = true;
            // Testing increasing stabilize P gain so will set lean angle target
            switch (test_axis) {
            case AxisType::ROLL:
                // request roll to 20deg
                attitude_control->input_angle_step_bf_roll_pitch_yaw_rad(dir_sign * cd_to_rad(target_angle), 0.0, 0.0);
                break;
            case AxisType::PITCH:
                // request pitch to 20deg
                attitude_control->input_angle_step_bf_roll_pitch_yaw_rad(0.0, dir_sign * cd_to_rad(target_angle), 0.0);
                break;
            case AxisType::YAW:
            case AxisType::YAW_D:
                // request yaw to 20deg
                attitude_control->input_angle_step_bf_roll_pitch_yaw_rad(0.0, 0.0, dir_sign * cd_to_rad(target_angle));
                break;
            } 
        } else {
            attitude_control->input_rate_bf_roll_pitch_yaw_rads(0.0, 0.0, 0.0);
        }
    } else {
        // Testing rate P and D gains so will set body-frame rate targets.
        // Rate controller will use existing body-frame rates and convert to motor outputs
        // for all axes except the one we override here.
        switch (test_axis) {
        case AxisType::ROLL:
            // override body-frame roll rate
            attitude_control->input_rate_step_bf_roll_pitch_yaw_rads(dir_sign * cd_to_rad(target_rate + start_rate), 0.0f, 0.0f);
            break;
        case AxisType::PITCH:
            // override body-frame pitch rate
            attitude_control->input_rate_step_bf_roll_pitch_yaw_rads(0.0f, dir_sign * cd_to_rad(target_rate + start_rate), 0.0f);
            break;
        case AxisType::YAW:
        case AxisType::YAW_D:
            // override body-frame yaw rate
            attitude_control->input_rate_step_bf_roll_pitch_yaw_rads(0.0f, 0.0f, dir_sign * cd_to_rad(target_rate + start_rate));
            break;
        }
    }

    // capture this iteration's rotation rate and lean angle
    float gyro_reading = 0;
    switch (test_axis) {
    case AxisType::ROLL:
        gyro_reading = ahrs_view->get_gyro().x;
        lean_angle = dir_sign * (ahrs_view->roll_sensor - (int32_t)start_angle);
        break;
    case AxisType::PITCH:
        gyro_reading = ahrs_view->get_gyro().y;
        lean_angle = dir_sign * (ahrs_view->pitch_sensor - (int32_t)start_angle);
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        gyro_reading = ahrs_view->get_gyro().z;
        lean_angle = dir_sign * wrap_180_cd(ahrs_view->yaw_sensor-(int32_t)start_angle);
        break;
    }

    // Add filter to measurements
    float filter_value;
    switch (tune_type) {
    case TuneType::ANGLE_P_DOWN:
    case TuneType::ANGLE_P_UP:
        filter_value = dir_sign * (degrees(gyro_reading) * 100.0);
        break;
    default:
        filter_value = dir_sign * (degrees(gyro_reading) * 100.0 - start_rate);
        break;
    }
    rotation_rate = rotation_rate_filt.apply(filter_value,
                    AP::scheduler().get_loop_period_s());

    switch (tune_type) {
    case TuneType::RATE_D_UP:
    case TuneType::RATE_D_DOWN:
        twitching_test_rate(lean_angle, rotation_rate, target_rate, test_rate_min, test_rate_max, test_angle_min);
        twitching_measure_acceleration(test_accel_max_cdss, rotation_rate, accel_measure_rate_max);
        twitching_abort_rate(lean_angle, rotation_rate, angle_abort, test_rate_min, test_angle_min);
        break;
    case TuneType::RATE_P_UP:
        twitching_test_rate(lean_angle, rotation_rate, target_rate * (1 + 0.5 * aggressiveness), test_rate_min, test_rate_max, test_angle_min);
        twitching_measure_acceleration(test_accel_max_cdss, rotation_rate, accel_measure_rate_max);
        twitching_abort_rate(lean_angle, rotation_rate, angle_abort, test_rate_min, test_angle_min);
        break;
    case TuneType::ANGLE_P_DOWN:
    case TuneType::ANGLE_P_UP:
        twitching_test_angle(lean_angle, rotation_rate, target_angle * (1 + 0.5 * aggressiveness), test_angle_min, test_angle_max, test_rate_min, test_rate_max);
        twitching_measure_acceleration(test_accel_max_cdss, rotation_rate - dir_sign * start_rate, accel_measure_rate_max);
        break;
    case TuneType::RATE_FF_UP:
    case TuneType::MAX_GAINS:
    case TuneType::TUNE_CHECK:
        // this should never happen
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    default:
        break;
    }
}

// backup_gains_and_initialise - store current gains as originals
//  called before tuning starts to backup original gains
void AC_AutoTune_Multi::backup_gains_and_initialise()
{
    AC_AutoTune::backup_gains_and_initialise();

    aggressiveness.set(constrain_float(aggressiveness, 0.05, 0.2));

    orig_bf_feedforward = attitude_control->get_bf_feedforward();

    // backup original pids and initialise tuned pid values
    orig_roll_rp = attitude_control->get_rate_roll_pid().kP();
    orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
    orig_roll_rd = attitude_control->get_rate_roll_pid().kD();
    orig_roll_rff = attitude_control->get_rate_roll_pid().ff();
    orig_roll_dff = attitude_control->get_rate_roll_pid().kDff();
    orig_roll_fltt = attitude_control->get_rate_roll_pid().filt_T_hz();
    orig_roll_smax = attitude_control->get_rate_roll_pid().slew_limit();
    orig_roll_sp = attitude_control->get_angle_roll_p().kP();
    orig_roll_accel_radss = attitude_control->get_accel_roll_max_radss();
    tune_roll_rp = attitude_control->get_rate_roll_pid().kP();
    tune_roll_rd = attitude_control->get_rate_roll_pid().kD();
    tune_roll_sp = attitude_control->get_angle_roll_p().kP();
    tune_roll_accel_radss = attitude_control->get_accel_roll_max_radss();

    orig_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
    orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
    orig_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
    orig_pitch_rff = attitude_control->get_rate_pitch_pid().ff();
    orig_pitch_dff = attitude_control->get_rate_pitch_pid().kDff();
    orig_pitch_fltt = attitude_control->get_rate_pitch_pid().filt_T_hz();
    orig_pitch_smax = attitude_control->get_rate_pitch_pid().slew_limit();
    orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
    orig_pitch_accel_radss = attitude_control->get_accel_pitch_max_radss();
    tune_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
    tune_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
    tune_pitch_sp = attitude_control->get_angle_pitch_p().kP();
    tune_pitch_accel_radss = attitude_control->get_accel_pitch_max_radss();

    orig_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
    orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
    orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
    orig_yaw_rff = attitude_control->get_rate_yaw_pid().ff();
    orig_yaw_dff = attitude_control->get_rate_yaw_pid().kDff();
    orig_yaw_fltt = attitude_control->get_rate_yaw_pid().filt_T_hz();
    orig_yaw_smax = attitude_control->get_rate_yaw_pid().slew_limit();
    orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_E_hz();
    orig_yaw_accel_radss = attitude_control->get_accel_yaw_max_radss();
    orig_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    tune_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
    tune_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
    tune_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_E_hz();
    if (yaw_d_enabled() && is_zero(tune_yaw_rd)) {
        tune_yaw_rd = min_d;
    }
    if (yaw_enabled() && is_zero(tune_yaw_rLPF)) {
        tune_yaw_rLPF = AUTOTUNE_FLTE_MIN;
    }
    tune_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    tune_yaw_accel_radss = attitude_control->get_accel_yaw_max_radss();

    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_INITIALISED);
}

// load_orig_gains - set gains to their original values
//  called by stop and failed functions
void AC_AutoTune_Multi::load_orig_gains()
{
    attitude_control->use_sqrt_controller(true);
    attitude_control->bf_feedforward(orig_bf_feedforward);
    if (roll_enabled()) {
        if (!is_zero(orig_roll_rp)) {
            attitude_control->get_rate_roll_pid().set_kP(orig_roll_rp);
            attitude_control->get_rate_roll_pid().set_kI(orig_roll_ri);
            attitude_control->get_rate_roll_pid().set_kD(orig_roll_rd);
            attitude_control->get_rate_roll_pid().set_ff(orig_roll_rff);
            attitude_control->get_rate_roll_pid().set_kDff(orig_roll_dff);
            attitude_control->get_rate_roll_pid().set_filt_T_hz(orig_roll_fltt);
            attitude_control->get_rate_roll_pid().set_slew_limit(orig_roll_smax);
            attitude_control->get_angle_roll_p().set_kP(orig_roll_sp);
            attitude_control->set_accel_roll_max_radss(orig_roll_accel_radss);
        }
    }
    if (pitch_enabled()) {
        if (!is_zero(orig_pitch_rp)) {
            attitude_control->get_rate_pitch_pid().set_kP(orig_pitch_rp);
            attitude_control->get_rate_pitch_pid().set_kI(orig_pitch_ri);
            attitude_control->get_rate_pitch_pid().set_kD(orig_pitch_rd);
            attitude_control->get_rate_pitch_pid().set_ff(orig_pitch_rff);
            attitude_control->get_rate_pitch_pid().set_kDff(orig_pitch_dff);
            attitude_control->get_rate_pitch_pid().set_filt_T_hz(orig_pitch_fltt);
            attitude_control->get_rate_pitch_pid().set_slew_limit(orig_pitch_smax);
            attitude_control->get_angle_pitch_p().set_kP(orig_pitch_sp);
            attitude_control->set_accel_pitch_max_radss(orig_pitch_accel_radss);
        }
    }
    if (yaw_enabled() || yaw_d_enabled()) {
        if (!is_zero(orig_yaw_rp)) {
            attitude_control->get_rate_yaw_pid().set_kP(orig_yaw_rp);
            attitude_control->get_rate_yaw_pid().set_kI(orig_yaw_ri);
            attitude_control->get_rate_yaw_pid().set_kD(orig_yaw_rd);
            attitude_control->get_rate_yaw_pid().set_ff(orig_yaw_rff);
            attitude_control->get_rate_yaw_pid().set_kDff(orig_yaw_dff);
            attitude_control->get_rate_yaw_pid().set_filt_E_hz(orig_yaw_rLPF);
            attitude_control->get_rate_yaw_pid().set_filt_T_hz(orig_yaw_fltt);
            attitude_control->get_rate_yaw_pid().set_slew_limit(orig_yaw_smax);
            attitude_control->get_angle_yaw_p().set_kP(orig_yaw_sp);
            attitude_control->set_accel_yaw_max_radss(orig_yaw_accel_radss);
        }
    }
}

// load_tuned_gains - load tuned gains
void AC_AutoTune_Multi::load_tuned_gains()
{
    attitude_control->use_sqrt_controller(true);
    if (!attitude_control->get_bf_feedforward()) {
        attitude_control->bf_feedforward(true);
        attitude_control->set_accel_roll_max_radss(0.0);
        attitude_control->set_accel_pitch_max_radss(0.0);
    }
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL) && roll_enabled() && !is_zero(tune_roll_rp)) {
        attitude_control->get_rate_roll_pid().set_kP(tune_roll_rp);
        attitude_control->get_rate_roll_pid().set_kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
        attitude_control->get_rate_roll_pid().set_kD(tune_roll_rd);
        attitude_control->get_rate_roll_pid().set_ff(orig_roll_rff);
        attitude_control->get_rate_roll_pid().set_kDff(orig_roll_dff);
        attitude_control->get_angle_roll_p().set_kP(tune_roll_sp);
        attitude_control->set_accel_roll_max_radss(tune_roll_accel_radss);
    }
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) && pitch_enabled() && !is_zero(tune_pitch_rp)) {
        attitude_control->get_rate_pitch_pid().set_kP(tune_pitch_rp);
        attitude_control->get_rate_pitch_pid().set_kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
        attitude_control->get_rate_pitch_pid().set_kD(tune_pitch_rd);
        attitude_control->get_rate_pitch_pid().set_ff(orig_pitch_rff);
        attitude_control->get_rate_pitch_pid().set_kDff(orig_pitch_dff);
        attitude_control->get_angle_pitch_p().set_kP(tune_pitch_sp);
        attitude_control->set_accel_pitch_max_radss(tune_pitch_accel_radss);
    }
    if ((((axes_completed & AUTOTUNE_AXIS_BITMASK_YAW) && yaw_enabled())
            || ((axes_completed & AUTOTUNE_AXIS_BITMASK_YAW_D) && yaw_d_enabled())) && !is_zero(tune_yaw_rp)) {
        attitude_control->get_rate_yaw_pid().set_kP(tune_yaw_rp);
        attitude_control->get_rate_yaw_pid().set_kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
        if (yaw_d_enabled()) {
            attitude_control->get_rate_yaw_pid().set_kD(tune_yaw_rd);
        }
        if (yaw_enabled()) {
            attitude_control->get_rate_yaw_pid().set_filt_E_hz(tune_yaw_rLPF);
        }
        attitude_control->get_rate_yaw_pid().set_ff(orig_yaw_rff);
        attitude_control->get_rate_yaw_pid().set_kDff(orig_yaw_dff);
        attitude_control->get_angle_yaw_p().set_kP(tune_yaw_sp);
        attitude_control->set_accel_yaw_max_radss(tune_yaw_accel_radss);
    }
}

// load_intra_test_gains - gains used between tests
//  called during testing mode's update-gains step to set gains ahead of return-to-level step
void AC_AutoTune_Multi::load_intra_test_gains()
{
    // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
    // sanity check the gains
    attitude_control->use_sqrt_controller(true);
    attitude_control->bf_feedforward(true);
    if (roll_enabled()) {
        attitude_control->get_rate_roll_pid().set_kP(orig_roll_rp);
        attitude_control->get_rate_roll_pid().set_kI(orig_roll_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_roll_pid().set_kD(orig_roll_rd);
        attitude_control->get_rate_roll_pid().set_ff(orig_roll_rff);
        attitude_control->get_rate_roll_pid().set_kDff(orig_roll_dff);
        attitude_control->get_rate_roll_pid().set_filt_T_hz(orig_roll_fltt);
        attitude_control->get_rate_roll_pid().set_slew_limit(orig_roll_smax);
        attitude_control->get_angle_roll_p().set_kP(orig_roll_sp);
    }
    if (pitch_enabled()) {
        attitude_control->get_rate_pitch_pid().set_kP(orig_pitch_rp);
        attitude_control->get_rate_pitch_pid().set_kI(orig_pitch_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_pitch_pid().set_kD(orig_pitch_rd);
        attitude_control->get_rate_pitch_pid().set_ff(orig_pitch_rff);
        attitude_control->get_rate_pitch_pid().set_kDff(orig_pitch_dff);
        attitude_control->get_rate_pitch_pid().set_filt_T_hz(orig_pitch_fltt);
        attitude_control->get_rate_pitch_pid().set_slew_limit(orig_pitch_smax);
        attitude_control->get_angle_pitch_p().set_kP(orig_pitch_sp);
    }
    if (yaw_enabled() || yaw_d_enabled()) {
        attitude_control->get_rate_yaw_pid().set_kP(orig_yaw_rp);
        attitude_control->get_rate_yaw_pid().set_kI(orig_yaw_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        attitude_control->get_rate_yaw_pid().set_kD(orig_yaw_rd);
        attitude_control->get_rate_yaw_pid().set_ff(orig_yaw_rff);
        attitude_control->get_rate_yaw_pid().set_kDff(orig_yaw_dff);
        attitude_control->get_rate_yaw_pid().set_filt_T_hz(orig_yaw_fltt);
        attitude_control->get_rate_yaw_pid().set_slew_limit(orig_yaw_smax);
        attitude_control->get_rate_yaw_pid().set_filt_E_hz(orig_yaw_rLPF);
        attitude_control->get_angle_yaw_p().set_kP(orig_yaw_sp);
    }
}

// load_test_gains - load the to-be-tested gains for a single axis
// called by control_attitude() just before it beings testing a gain (i.e. just before it twitches)
void AC_AutoTune_Multi::load_test_gains()
{
    attitude_control->use_sqrt_controller(false);
    switch (axis) {
    case AxisType::ROLL:
        attitude_control->get_rate_roll_pid().set_kP(tune_roll_rp);
        attitude_control->get_rate_roll_pid().set_kI(tune_roll_rp * 0.01);
        attitude_control->get_rate_roll_pid().set_kD(tune_roll_rd);
        attitude_control->get_rate_roll_pid().set_ff(0.0);
        attitude_control->get_rate_roll_pid().set_kDff(0.0);
        attitude_control->get_rate_roll_pid().set_filt_T_hz(0.0);
        attitude_control->get_rate_roll_pid().set_slew_limit(0.0);
        attitude_control->get_angle_roll_p().set_kP(tune_roll_sp);
        break;
    case AxisType::PITCH:
        attitude_control->get_rate_pitch_pid().set_kP(tune_pitch_rp);
        attitude_control->get_rate_pitch_pid().set_kI(tune_pitch_rp * 0.01);
        attitude_control->get_rate_pitch_pid().set_kD(tune_pitch_rd);
        attitude_control->get_rate_pitch_pid().set_ff(0.0);
        attitude_control->get_rate_pitch_pid().set_kDff(0.0);
        attitude_control->get_rate_pitch_pid().set_filt_T_hz(0.0);
        attitude_control->get_rate_pitch_pid().set_slew_limit(0.0);
        attitude_control->get_angle_pitch_p().set_kP(tune_pitch_sp);
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        attitude_control->get_rate_yaw_pid().set_kP(tune_yaw_rp);
        attitude_control->get_rate_yaw_pid().set_kI(tune_yaw_rp * 0.01);
        attitude_control->get_rate_yaw_pid().set_ff(0.0);
        attitude_control->get_rate_yaw_pid().set_kDff(0.0);
        if (axis == AxisType::YAW_D) {
            attitude_control->get_rate_yaw_pid().set_kD(tune_yaw_rd);
        } else {
            attitude_control->get_rate_yaw_pid().set_kD(0.0);
            attitude_control->get_rate_yaw_pid().set_filt_E_hz(tune_yaw_rLPF);
        }
        attitude_control->get_rate_yaw_pid().set_filt_T_hz(0.0);
        attitude_control->get_rate_yaw_pid().set_slew_limit(0.0);
        attitude_control->get_angle_yaw_p().set_kP(tune_yaw_sp);
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
        attitude_control->save_accel_roll_max_radss(0.0);
        attitude_control->save_accel_pitch_max_radss(0.0);
    }

    // sanity check the rate P values
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL) && roll_enabled() && !is_zero(tune_roll_rp)) {
        // rate roll gains
        attitude_control->get_rate_roll_pid().set_kP(tune_roll_rp);
        attitude_control->get_rate_roll_pid().set_kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
        attitude_control->get_rate_roll_pid().set_kD(tune_roll_rd);
        attitude_control->get_rate_roll_pid().set_ff(orig_roll_rff);
        attitude_control->get_rate_roll_pid().set_kDff(orig_roll_dff);
        attitude_control->get_rate_roll_pid().set_filt_T_hz(orig_roll_fltt);
        attitude_control->get_rate_roll_pid().set_slew_limit(orig_roll_smax);
        attitude_control->get_rate_roll_pid().save_gains();

        // stabilize roll
        attitude_control->get_angle_roll_p().set_kP(tune_roll_sp);
        attitude_control->get_angle_roll_p().save_gains();

        // acceleration roll
        attitude_control->save_accel_roll_max_radss(tune_roll_accel_radss);

        // resave pids to originals in case the autotune is run again
        orig_roll_rp = attitude_control->get_rate_roll_pid().kP();
        orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
        orig_roll_rd = attitude_control->get_rate_roll_pid().kD();
        orig_roll_rff = attitude_control->get_rate_roll_pid().ff();
        orig_roll_dff = attitude_control->get_rate_roll_pid().kDff();
        orig_roll_sp = attitude_control->get_angle_roll_p().kP();
        orig_roll_accel_radss = attitude_control->get_accel_roll_max_radss();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) && pitch_enabled() && !is_zero(tune_pitch_rp)) {
        // rate pitch gains
        attitude_control->get_rate_pitch_pid().set_kP(tune_pitch_rp);
        attitude_control->get_rate_pitch_pid().set_kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
        attitude_control->get_rate_pitch_pid().set_kD(tune_pitch_rd);
        attitude_control->get_rate_pitch_pid().set_ff(orig_pitch_rff);
        attitude_control->get_rate_pitch_pid().set_kDff(orig_pitch_dff);
        attitude_control->get_rate_pitch_pid().set_filt_T_hz(orig_pitch_fltt);
        attitude_control->get_rate_pitch_pid().set_slew_limit(orig_pitch_smax);
        attitude_control->get_rate_pitch_pid().save_gains();

        // stabilize pitch
        attitude_control->get_angle_pitch_p().set_kP(tune_pitch_sp);
        attitude_control->get_angle_pitch_p().save_gains();

        // acceleration pitch
        attitude_control->save_accel_pitch_max_radss(tune_pitch_accel_radss);

        // resave pids to originals in case the autotune is run again
        orig_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
        orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
        orig_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
        orig_pitch_rff = attitude_control->get_rate_pitch_pid().ff();
        orig_pitch_dff = attitude_control->get_rate_pitch_pid().kDff();
        orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
        orig_pitch_accel_radss = attitude_control->get_accel_pitch_max_radss();
    }

    if ((((axes_completed & AUTOTUNE_AXIS_BITMASK_YAW) && yaw_enabled())
            || ((axes_completed & AUTOTUNE_AXIS_BITMASK_YAW_D) && yaw_d_enabled())) && !is_zero(tune_yaw_rp)) {
        // rate yaw gains
        attitude_control->get_rate_yaw_pid().set_kP(tune_yaw_rp);
        attitude_control->get_rate_yaw_pid().set_kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
        attitude_control->get_rate_yaw_pid().set_ff(orig_yaw_rff);
        attitude_control->get_rate_yaw_pid().set_kDff(orig_yaw_dff);
        attitude_control->get_rate_yaw_pid().set_filt_T_hz(orig_yaw_fltt);
        attitude_control->get_rate_yaw_pid().set_slew_limit(orig_yaw_smax);
        if (yaw_d_enabled()) {
            attitude_control->get_rate_yaw_pid().set_kD(tune_yaw_rd);
        }
        if (yaw_enabled()) {
            attitude_control->get_rate_yaw_pid().set_filt_E_hz(tune_yaw_rLPF);
        }
        attitude_control->get_rate_yaw_pid().save_gains();

        // stabilize yaw
        attitude_control->get_angle_yaw_p().set_kP(tune_yaw_sp);
        attitude_control->get_angle_yaw_p().save_gains();

        // acceleration yaw
        attitude_control->save_accel_yaw_max_radss(tune_yaw_accel_radss);

        // resave pids to originals in case the autotune is run again
        orig_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
        orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
        orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
        orig_yaw_rff = attitude_control->get_rate_yaw_pid().ff();
        orig_yaw_dff = attitude_control->get_rate_yaw_pid().kDff();
        orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_E_hz();
        orig_yaw_sp = attitude_control->get_angle_yaw_p().kP();
        orig_yaw_accel_radss = attitude_control->get_accel_yaw_max_radss();
    }

    // update GCS and log save gains event
    update_gcs(AUTOTUNE_MESSAGE_SAVED_GAINS);
    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_SAVEDGAINS);

    reset();
}

// report final gains for a given axis to GCS
void AC_AutoTune_Multi::report_final_gains(AxisType test_axis) const
{
    switch (test_axis) {
        case AxisType::ROLL:
            report_axis_gains("Roll", tune_roll_rp, tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL, tune_roll_rd, tune_roll_sp, tune_roll_accel_radss);
            break;
        case AxisType::PITCH:
            report_axis_gains("Pitch", tune_pitch_rp, tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL, tune_pitch_rd, tune_pitch_sp, tune_pitch_accel_radss);
            break;
        case AxisType::YAW:
            report_axis_gains("Yaw(E)", tune_yaw_rp, tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL, 0, tune_yaw_sp, tune_yaw_accel_radss);
            break;
        case AxisType::YAW_D:
            report_axis_gains("Yaw(D)", tune_yaw_rp, tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL, tune_yaw_rd, tune_yaw_sp, tune_yaw_accel_radss);
            break;
    }
}

// report gain formatting helper
void AC_AutoTune_Multi::report_axis_gains(const char* axis_string, float rate_P, float rate_I, float rate_D, float angle_P, float max_accel_radss) const
{
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: %s complete", axis_string);
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: %s Rate: P:%0.3f, I:%0.3f, D:%0.4f", axis_string, rate_P, rate_I,rate_D);
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: %s Angle P:%0.3f, Max Accel:%0.0f", axis_string, angle_P, rad_to_cd(max_accel_radss));
}

// Measures peak angular rates and bounce-back behavior during rate tuning.
// Updates step timer and determines if tuning step is complete.
void AC_AutoTune_Multi::twitching_test_rate(float angle, float rate, float rate_target_max, float &meas_rate_min, float &meas_rate_max, float &meas_angle_min)
{
    const uint32_t now_ms = AP_HAL::millis();

    // capture maximum rate
    if (rate > meas_rate_max) {
        // the measurement is continuing to increase without stopping
        meas_rate_max = rate;
        meas_rate_min = rate;
        meas_angle_min = angle;
    }

    // capture minimum measurement after the measurement has peaked (aka "bounce back")
    if ((rate < meas_rate_min) && (meas_rate_max > rate_target_max * 0.25)) {
        // the measurement is bouncing back
        meas_rate_min = rate;
        meas_angle_min = angle;
    }

    // calculate early stopping time based on the time it takes to get to 63.21%
    if (meas_rate_max < rate_target_max * 0.6321) {
        // the measurement not reached the 63.21% threshold yet
        step_timeout_ms = (now_ms - step_start_time_ms) * 3;
        step_timeout_ms = MIN(step_timeout_ms, AUTOTUNE_TESTING_STEP_TIMEOUT_MS);
    }

    if (meas_rate_max > rate_target_max) {
        // the measured rate has passed the maximum target rate
        step = Step::UPDATE_GAINS;
    }

    if (meas_rate_max - meas_rate_min > meas_rate_max * aggressiveness) {
        // the measurement has passed 50% of the maximum rate and bounce back is larger than the threshold
        step = Step::UPDATE_GAINS;
    }

    if (now_ms - step_start_time_ms >= step_timeout_ms) {
        // we have passed the maximum stop time
        step = Step::UPDATE_GAINS;
    }
}

// twitching_test_rate - twitching tests
// update min and max and test for end conditions
void AC_AutoTune_Multi::twitching_abort_rate(float angle, float rate, float angle_max, float meas_rate_min, float angle_min)
{
    if (angle >= angle_max) {
        if (is_equal(rate, meas_rate_min) || (angle_min > 0.95 * angle_max)) {
            // we have reached the angle limit before completing the measurement of maximum and minimum
            // reduce the maximum target rate
            if (step_scaler > 0.2f) {
                step_scaler *= 0.9f;
            } else {
                LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AutoTune: Twitch Size Determination Failed");
                mode = TuneMode::FAILED;
                LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_FAILED);
            }
            // ignore result and start test again
            step = Step::ABORT;
        } else {
            step = Step::UPDATE_GAINS;
        }
    }
}

// twitching_test_angle - twitching tests
// update min and max and test for end conditions
void AC_AutoTune_Multi::twitching_test_angle(float angle, float rate, float angle_target_max, float &meas_angle_min, float &meas_angle_max, float &meas_rate_min, float &meas_rate_max)
{
    const uint32_t now_ms = AP_HAL::millis();

    // capture maximum angle
    if (angle > meas_angle_max) {
        // the angle still increasing
        meas_angle_max = angle;
        meas_angle_min = angle;
    }

    // capture minimum angle after we have reached a reasonable maximum angle
    if ((angle < meas_angle_min) && (meas_angle_max > angle_target_max * 0.25)) {
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

    // calculate early stopping time based on the time it takes to get to 63.21%
    if (meas_angle_max < angle_target_max * 0.6321) {
        // the measurement not reached the 63.21% threshold yet
        step_timeout_ms = (now_ms - step_start_time_ms) * 3;
        step_timeout_ms = MIN(step_timeout_ms, AUTOTUNE_TESTING_STEP_TIMEOUT_MS);
    }

    if (meas_angle_max > angle_target_max) {
        // the measurement has passed the maximum angle
        step = Step::UPDATE_GAINS;
    }

    if (meas_angle_max - meas_angle_min > meas_angle_max * aggressiveness) {
        // the measurement has passed 50% of the maximum angle and bounce back is larger than the threshold
        step = Step::UPDATE_GAINS;
    }

    if (now_ms - step_start_time_ms >= step_timeout_ms) {
        // we have passed the maximum stop time
        step = Step::UPDATE_GAINS;
    }
}

// twitching_measure_acceleration - measure rate of change of measurement
void AC_AutoTune_Multi::twitching_measure_acceleration(float &accel_average, float rate, float &rate_max) const
{
    if (rate_max < rate) {
        rate_max = rate;
        accel_average = (1000.0 * rate_max) / (AP_HAL::millis() - step_start_time_ms);
    }
}

// update gains for the rate p up tune type
void AC_AutoTune_Multi::updating_rate_p_up_all(AxisType test_axis)
{
    switch (test_axis) {
    case AxisType::ROLL:
        updating_rate_p_up_d_down(tune_roll_rd, min_d, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case AxisType::PITCH:
        updating_rate_p_up_d_down(tune_pitch_rd, min_d, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case AxisType::YAW:
        updating_rate_p_up_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max, false);
        break;
    case AxisType::YAW_D:
        updating_rate_p_up_d_down(tune_yaw_rd, min_d, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    }
}

// update gains for the rate d up tune type
void AC_AutoTune_Multi::updating_rate_d_up_all(AxisType test_axis)
{
    switch (test_axis) {
    case AxisType::ROLL:
        updating_rate_d_up(tune_roll_rd, min_d, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case AxisType::PITCH:
        updating_rate_d_up(tune_pitch_rd, min_d, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case AxisType::YAW:
        updating_rate_d_up(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RLPF_MAX, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case AxisType::YAW_D:
        updating_rate_d_up(tune_yaw_rd, min_d, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    }
}

// update gains for the rate d down tune type
void AC_AutoTune_Multi::updating_rate_d_down_all(AxisType test_axis)
{
    switch (test_axis) {
    case AxisType::ROLL:
        updating_rate_d_down(tune_roll_rd, min_d, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case AxisType::PITCH:
        updating_rate_d_down(tune_pitch_rd, min_d, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case AxisType::YAW:
        updating_rate_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    case AxisType::YAW_D:
        updating_rate_d_down(tune_yaw_rd, min_d, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, target_rate, test_rate_min, test_rate_max);
        break;
    }
}

// update gains for the angle p up tune type
void AC_AutoTune_Multi::updating_angle_p_up_all(AxisType test_axis)
{
    switch (test_axis) {
    case AxisType::ROLL:
        updating_angle_p_up(tune_roll_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
        break;
    case AxisType::PITCH:
        updating_angle_p_up(tune_pitch_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        updating_angle_p_up(tune_yaw_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
        break;
    }
}

// update gains for the angle p down tune type
void AC_AutoTune_Multi::updating_angle_p_down_all(AxisType test_axis)
{
    switch (test_axis) {
    case AxisType::ROLL:
        updating_angle_p_down(tune_roll_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
        break;
    case AxisType::PITCH:
        updating_angle_p_down(tune_pitch_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        updating_angle_p_down(tune_yaw_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, target_angle, test_angle_max, test_rate_min, test_rate_max);
        break;
    }
}

// set gains post tune for the tune type
void AC_AutoTune_Multi::set_tuning_gains_with_backoff(AxisType test_axis)
{
    // ensure gain_backoff has not been set outside limits
    gain_backoff.set_and_save_ifchanged(constrain_float(gain_backoff, 0.0f, 0.5f));

    switch (tune_type) {
    case TuneType::RATE_D_UP:
        break;
    case TuneType::RATE_D_DOWN:
        break;
    case TuneType::RATE_P_UP:
        switch (test_axis) {
        case AxisType::ROLL:
            tune_roll_rd = tune_roll_rd * (1.0f - gain_backoff);
            tune_roll_rp = tune_roll_rp * (1.0f - gain_backoff);
            break;
        case AxisType::PITCH:
            tune_pitch_rd = tune_pitch_rd * (1.0f - gain_backoff);
            tune_pitch_rp = tune_pitch_rp * (1.0f - gain_backoff);
            break;
        case AxisType::YAW:
            tune_yaw_rp = tune_yaw_rp * (1.0f - gain_backoff);
            break;
        case AxisType::YAW_D:
            tune_yaw_rd = tune_yaw_rd * (1.0f - gain_backoff);
            tune_yaw_rp = tune_yaw_rp * (1.0f - gain_backoff);
            break;
        }
        break;
    case TuneType::ANGLE_P_DOWN:
        break;
    case TuneType::ANGLE_P_UP:
        switch (test_axis) {
        case AxisType::ROLL:
            tune_roll_sp = tune_roll_sp * (1.0f - gain_backoff) * (1.0f - aggressiveness);
            tune_roll_accel_radss = cd_to_rad(MAX(AUTOTUNE_RP_ACCEL_MIN, test_accel_max_cdss * AUTOTUNE_ACCEL_RP_BACKOFF));
            break;
        case AxisType::PITCH:
            tune_pitch_sp = tune_pitch_sp * (1.0f - gain_backoff) * (1.0f - aggressiveness);
            tune_pitch_accel_radss = cd_to_rad(MAX(AUTOTUNE_RP_ACCEL_MIN, test_accel_max_cdss * AUTOTUNE_ACCEL_RP_BACKOFF));
            break;
        case AxisType::YAW:
        case AxisType::YAW_D:
            tune_yaw_sp = tune_yaw_sp * (1.0f - gain_backoff) * (1.0f - aggressiveness);
            tune_yaw_accel_radss = cd_to_rad(MAX(AUTOTUNE_Y_ACCEL_MIN, test_accel_max_cdss * AUTOTUNE_ACCEL_Y_BACKOFF));
            break;
        }
        break;
    case TuneType::RATE_FF_UP:
    case TuneType::MAX_GAINS:
    case TuneType::TUNE_CHECK:
        // this should never happen
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    case TuneType::TUNE_COMPLETE:
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
        tune_p -= tune_p * tune_p_step_ratio;
        if (tune_p < tune_p_min) {
            // P gain is at minimum so start reducing D
            tune_p = tune_p_min;
            tune_d -= tune_d * tune_d_step_ratio;
            if (tune_d <= tune_d_min) {
                // We have reached minimum D gain so stop tuning
                tune_d = tune_d_min;
                success_counter = AUTOTUNE_SUCCESS_COUNT;
                LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
                // This may be mean AGGR should be increased or MIN_D decreased
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Min Rate D limit reached");
            }
        }
    } else if ((meas_rate_max < rate_target * (1.0 - AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p * tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
        }
    } else {
        // we have a good measurement of bounce back
        if (meas_rate_max-meas_rate_min > meas_rate_max * aggressiveness) {
            // ignore the next result unless it is the same as this one
            ignore_next = true;
            // bounce back is bigger than our threshold so increment the success counter
            success_counter++;
        } else {
            if (ignore_next == false) {
                // bounce back is smaller than our threshold so decrement the success counter
                if (success_counter > 0) {
                    success_counter--;
                }
                // increase D gain (which should increase bounce back)
                tune_d += tune_d*tune_d_step_ratio * 2.0;
                // stop tuning if we hit maximum D
                if (tune_d >= tune_d_max) {
                    tune_d = tune_d_max;
                    success_counter = AUTOTUNE_SUCCESS_COUNT;
                    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
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
                success_counter = AUTOTUNE_SUCCESS_COUNT;
                LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
                // This may be mean AGGR should be increased or MIN_D decreased
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Min Rate D limit reached");
            }
        }
    } else if ((meas_rate_max < rate_target*(1.0 - AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p * tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
        }
    } else {
        // we have a good measurement of bounce back
        if (meas_rate_max - meas_rate_min < meas_rate_max * aggressiveness) {
            if (ignore_next == false) {
                // bounce back is less than our threshold so increment the success counter
                success_counter++;
            } else {
                ignore_next = false;
            }
        } else {
            // ignore the next result unless it is the same as this one
            ignore_next = true;
            // bounce back is larger than our threshold so decrement the success counter
            if (success_counter > 0) {
                success_counter--;
            }
            // decrease D gain (which should decrease bounce back)
            tune_d -= tune_d * tune_d_step_ratio;
            // stop tuning if we hit minimum D
            if (tune_d <= tune_d_min) {
                tune_d = tune_d_min;
                success_counter = AUTOTUNE_SUCCESS_COUNT;
                LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
                // This may be mean AGGR should be increased or MIN_D decreased
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Min Rate D limit reached");
            }
        }
    }
}

// updating_rate_p_up_d_down - increase P to ensure the target is reached while checking bounce back isn't increasing
// P is increased until we achieve our target within a reasonable time while reducing D if bounce back increases above the threshold
void AC_AutoTune_Multi::updating_rate_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max, bool fail_min_d)
{
    if (meas_rate_max > rate_target * (1.0 + 0.5 * aggressiveness)) {
        // ignore the next result unless it is the same as this one
        ignore_next = true;
        // if maximum measurement was greater than target so increment the success counter
        success_counter++;
    } else if ((meas_rate_max < rate_target) && (meas_rate_max > rate_target * (1.0 - AUTOTUNE_D_UP_DOWN_MARGIN)) && (meas_rate_max - meas_rate_min > meas_rate_max * aggressiveness) && (tune_d > tune_d_min)) {
        // if bounce back was larger than the threshold so decrement the success counter
        if (success_counter > 0) {
            success_counter--;
        }
        // Reduce D gain if bounce-back exceeds threshold.
        tune_d -= tune_d * tune_d_step_ratio;
        // do not decrease the D term past the minimum
        if (tune_d <= tune_d_min) {
            tune_d = tune_d_min;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
            if (fail_min_d) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AutoTune: Rate D Gain Determination Failed");
                mode = TuneMode::FAILED;
                LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_FAILED);
            }
        }
        // decrease P gain to match D gain reduction
        tune_p -= tune_p * tune_p_step_ratio;
        // do not decrease the P term past the minimum
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AutoTune: Rate P Gain Determination Failed");
            mode = TuneMode::FAILED;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_FAILED);
        }
    } else {
        if (ignore_next == false) {
            // if maximum measurement was lower than target so decrement the success counter
            if (success_counter > 0) {
                success_counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p * tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                success_counter = AUTOTUNE_SUCCESS_COUNT;
                LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
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
    if (meas_angle_max < angle_target * (1 + 0.5 * aggressiveness)) {
        if (ignore_next == false) {
            // if maximum measurement was lower than target so increment the success counter
            success_counter++;
        } else {
            ignore_next = false;
        }
    } else {
        // ignore the next result unless it is the same as this one
        ignore_next = true;
        // if maximum measurement was higher than target so decrement the success counter
        if (success_counter > 0) {
            success_counter--;
        }
        // decrease P gain (which should decrease the maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        // stop tuning if we hit maximum P
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AutoTune: Angle P Gain Determination Failed");
            mode = TuneMode::FAILED;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_FAILED);
       }
    }
}

// updating_angle_p_up - increase P to ensure the target is reached
// P is increased until we achieve our target within a reasonable time
void AC_AutoTune_Multi::updating_angle_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max)
{
    if ((meas_angle_max > angle_target * (1 + 0.5 * aggressiveness)) ||
        ((meas_angle_max > angle_target) && (meas_rate_min < -meas_rate_max * aggressiveness))) {
        // ignore the next result unless it is the same as this one
        ignore_next = true;
        // if maximum measurement was greater than target so increment the success counter
        success_counter++;
    } else {
        if (ignore_next == false) {
            // if maximum measurement was lower than target so decrement the success counter
            if (success_counter > 0) {
                success_counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p * tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                success_counter = AUTOTUNE_SUCCESS_COUNT;
                LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            ignore_next = false;
        }
    }
}

#if HAL_LOGGING_ENABLED
void AC_AutoTune_Multi::Log_AutoTune()
{
    if ((tune_type == TuneType::ANGLE_P_DOWN) || (tune_type == TuneType::ANGLE_P_UP)) {
        switch (axis) {
        case AxisType::ROLL:
            Log_Write_AutoTune(axis, tune_type, target_angle, test_angle_min, test_angle_max, tune_roll_rp, tune_roll_rd, tune_roll_sp, test_accel_max_cdss);
            break;
        case AxisType::PITCH:
            Log_Write_AutoTune(axis, tune_type, target_angle, test_angle_min, test_angle_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, test_accel_max_cdss);
            break;
        case AxisType::YAW:
            Log_Write_AutoTune(axis, tune_type, target_angle, test_angle_min, test_angle_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, test_accel_max_cdss);
            break;
        case AxisType::YAW_D:
            Log_Write_AutoTune(axis, tune_type, target_angle, test_angle_min, test_angle_max, tune_yaw_rp, tune_yaw_rd, tune_yaw_sp, test_accel_max_cdss);
            break;
        }
    } else {
        switch (axis) {
        case AxisType::ROLL:
            Log_Write_AutoTune(axis, tune_type, target_rate, test_rate_min, test_rate_max, tune_roll_rp, tune_roll_rd, tune_roll_sp, test_accel_max_cdss);
            break;
        case AxisType::PITCH:
            Log_Write_AutoTune(axis, tune_type, target_rate, test_rate_min, test_rate_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, test_accel_max_cdss);
            break;
        case AxisType::YAW:
            Log_Write_AutoTune(axis, tune_type, target_rate, test_rate_min, test_rate_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, test_accel_max_cdss);
            break;
        case AxisType::YAW_D:
            Log_Write_AutoTune(axis, tune_type, target_rate, test_rate_min, test_rate_max, tune_yaw_rp, tune_yaw_rd, tune_yaw_sp, test_accel_max_cdss);
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
void AC_AutoTune_Multi::Log_Write_AutoTune(AxisType _axis, TuneType tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt)
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
        meas_target*0.01,
        meas_min*0.01,
        meas_max*0.01,
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
        angle_cd*0.01,
        rate_cds*0.01);
}
#endif  // HAL_LOGGING_ENABLED

float AC_AutoTune_Multi::target_angle_max_rp_cd() const
{
    return attitude_control->lean_angle_max_cd() * AUTOTUNE_TARGET_ANGLE_MAX_RP_SCALE;
}

float AC_AutoTune_Multi::target_angle_max_y_cd() const
{
    // Aircraft with small lean angle will generally benefit from proportional smaller yaw twitch size
    return attitude_control->lean_angle_max_cd() * AUTOTUNE_TARGET_ANGLE_MAX_Y_SCALE;
}

float AC_AutoTune_Multi::target_angle_min_rp_cd() const
{
    return attitude_control->lean_angle_max_cd() * AUTOTUNE_TARGET_ANGLE_MIN_RP_SCALE;
}

float AC_AutoTune_Multi::target_angle_min_y_cd() const
{
    // Aircraft with small lean angle will generally benefit from proportional smaller yaw twitch size
    return attitude_control->lean_angle_max_cd() * AUTOTUNE_TARGET_ANGLE_MIN_Y_SCALE;
}

float AC_AutoTune_Multi::angle_lim_max_rp_cd() const
{
    return attitude_control->lean_angle_max_cd() * AUTOTUNE_ANGLE_ABORT_RP_SCALE;
}

float AC_AutoTune_Multi::angle_lim_neg_rpy_cd() const
{
    return attitude_control->lean_angle_max_cd() * AUTOTUNE_ANGLE_NEG_RP_SCALE;
}

// get_testing_step_timeout_ms accessor
uint32_t AC_AutoTune_Multi::get_testing_step_timeout_ms() const
{
    return AUTOTUNE_TESTING_STEP_TIMEOUT_MS;
}


#endif  // AC_AUTOTUNE_ENABLED
