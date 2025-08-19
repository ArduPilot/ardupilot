/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for autotune of helicopters. Based on original autotune code from ArduCopter, written by Leonard Hall
  Converted to a library by Andrew Tridgell, and rewritten to include helicopters by Bill Geyer
 */

#include "AC_AutoTune_config.h"

#if AC_AUTOTUNE_ENABLED

#include "AC_AutoTune_Heli.h"

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS   5000U     // timeout for tuning mode's testing step

#define AUTOTUNE_RD_STEP                  0.0005f     // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                  0.005f     // minimum increment when increasing/decreasing Rate P term
#define AUTOTUNE_SP_STEP                  0.05f     // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1f     // I is set 10x smaller than P during testing
#define AUTOTUNE_PI_RATIO_FINAL            1.0f     // I is set 1x P after testing
#define AUTOTUNE_YAW_PI_RATIO_FINAL        0.1f     // I is set 1x P after testing
#define AUTOTUNE_RD_MAX                  0.020f     // maximum Rate D value
#define AUTOTUNE_RP_MIN                   0.02f     // minimum Rate P value
#define AUTOTUNE_RP_MAX                   0.3f     // maximum Rate P value
#define AUTOTUNE_SP_MAX                    10.0f     // maximum Stab P value
#define AUTOTUNE_SP_MIN                    3.0f     // maximum Stab P value
#define AUTOTUNE_RFF_MAX                   0.5f     // maximum Stab P value
#define AUTOTUNE_RFF_MIN                   0.025f    // maximum Stab P value
#define AUTOTUNE_RD_BACKOFF                1.0f     // Rate D gains are reduced to 50% of their maximum value discovered during tuning
#define AUTOTUNE_RP_BACKOFF                1.0f     // Rate P gains are reduced to 97.5% of their maximum value discovered during tuning
#define AUTOTUNE_SP_BACKOFF                1.0f     // Stab P gains are reduced to 90% of their maximum value discovered during tuning
#define AUTOTUNE_ACCEL_RP_BACKOFF          1.0f     // back off from maximum acceleration
#define AUTOTUNE_ACCEL_Y_BACKOFF           1.0f     // back off from maximum acceleration

#define AUTOTUNE_HELI_TARGET_ANGLE_RLLPIT_CD     1500   // target roll/pitch angle during AUTOTUNE FeedForward rate test
#define AUTOTUNE_HELI_TARGET_RATE_RLLPIT_CDS     5000   // target roll/pitch rate during AUTOTUNE FeedForward rate test
#define AUTOTUNE_FFI_RATIO_FOR_TESTING     0.5f     // I is set 2x smaller than VFF during testing
#define AUTOTUNE_FFI_RATIO_FINAL           0.5f     // I is set 0.5x VFF after testing
#define AUTOTUNE_RP_ACCEL_MIN           20000.0f     // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_Y_ACCEL_MIN            10000.0f     // Minimum acceleration for Yaw

#define AUTOTUNE_SEQ_BITMASK_VFF             1
#define AUTOTUNE_SEQ_BITMASK_RATE_D          2
#define AUTOTUNE_SEQ_BITMASK_ANGLE_P         4
#define AUTOTUNE_SEQ_BITMASK_MAX_GAIN        8
#define AUTOTUNE_SEQ_BITMASK_TUNE_CHECK      16

// angle limits preserved from previous behaviour as Multi changed:
#define AUTOTUNE_ANGLE_TARGET_MAX_RP_CD     2000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_ANGLE_TARGET_MIN_RP_CD     1000    // minimum target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_ANGLE_TARGET_MAX_Y_CD      3000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_ANGLE_TARGET_MIN_Y_CD      500     // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_ANGLE_MAX_RP_CD            3000    // maximum allowable angle in degrees during testing
#define AUTOTUNE_ANGLE_NEG_RPY_CD           1000    // maximum allowable angle in degrees during testing

const AP_Param::GroupInfo AC_AutoTune_Heli::var_info[] = {

    // @Param: AXES
    // @DisplayName: Autotune axis bitmask
    // @Description: 1-byte bitmap of axes to autotune
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw
    // @User: Standard
    AP_GROUPINFO("AXES", 1, AC_AutoTune_Heli, axis_bitmask,  1),

    // @Param: SEQ
    // @DisplayName: AutoTune Sequence Bitmask
    // @Description: 2-byte bitmask to select what tuning should be performed.  Max gain automatically performed if Rate D is selected. Values: 7:All,1:VFF Only,2:Rate D/Rate P Only(incl max gain),4:Angle P Only,8:Max Gain Only,16:Tune Check,3:VFF and Rate D/Rate P(incl max gain),5:VFF and Angle P,6:Rate D/Rate P(incl max gain) and angle P
    // @Bitmask: 0:VFF,1:Rate D/Rate P(incl max gain),2:Angle P,3:Max Gain Only,4:Tune Check
    // @User: Standard
    AP_GROUPINFO("SEQ", 2, AC_AutoTune_Heli, seq_bitmask,  3),

    // @Param: FRQ_MIN
    // @DisplayName: AutoTune minimum sweep frequency
    // @Description: Defines the start frequency for sweeps and dwells
    // @Range: 10 30
    // @User: Standard
    AP_GROUPINFO("FRQ_MIN", 3, AC_AutoTune_Heli, min_sweep_freq,  10.0f),

    // @Param: FRQ_MAX
    // @DisplayName: AutoTune maximum sweep frequency
    // @Description: Defines the end frequency for sweeps and dwells
    // @Range: 50 120
    // @User: Standard
    AP_GROUPINFO("FRQ_MAX", 4, AC_AutoTune_Heli, max_sweep_freq,  70.0f),

    // @Param: GN_MAX
    // @DisplayName: AutoTune maximum response gain
    // @Description: Defines the response gain (output/input) to tune
    // @Range: 1 2.5
    // @User: Standard
    AP_GROUPINFO("GN_MAX", 5, AC_AutoTune_Heli, max_resp_gain,  1.0f),

    // @Param: VELXY_P
    // @DisplayName: AutoTune velocity xy P gain
    // @Description: Velocity xy P gain used to hold position during Max Gain, Rate P, and Rate D frequency sweeps
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("VELXY_P", 6, AC_AutoTune_Heli, vel_hold_gain,  0.1f),

    // @Param: ACC_MAX
    // @DisplayName: AutoTune maximum allowable angular acceleration
    // @Description: maximum angular acceleration in deg/s/s allowed during autotune maneuvers
    // @Range: 1 4000
    // @User: Standard
    // @Units: deg/s/s
    AP_GROUPINFO("ACC_MAX", 7, AC_AutoTune_Heli, accel_max_degss, 0.0f),

    // @Param: RAT_MAX
    // @DisplayName: Autotune maximum allowable angular rate
    // @Description: maximum angular rate in deg/s allowed during autotune maneuvers
    // @Range: 0 500
    // @User: Standard
    // @Units: deg/s
    AP_GROUPINFO("RAT_MAX", 8, AC_AutoTune_Heli, rate_max_degs, 0.0f),

    AP_GROUPEND
};

// constructor
AC_AutoTune_Heli::AC_AutoTune_Heli()
{
    tune_seq[0] = TuneType::TUNE_COMPLETE;
    AP_Param::setup_object_defaults(this, var_info);
}

// Prepares all tuning state variables and target values for a new test.
void AC_AutoTune_Heli::test_init()
{
    AC_AutoTune_FreqResp::ResponseType resp_type = AC_AutoTune_FreqResp::ResponseType::RATE;
    FreqRespCalcType calc_type = RATE;
    FreqRespInput freq_resp_input = TARGET;
    float freq_resp_amplitude = 5.0f;  // amplitude in deg
    float filter_freq = 10.0f;
    switch (tune_type) {
    case TuneType::RATE_FF_UP:
        if (!is_positive(next_test_freq)) {
            start_freq = 0.25f * M_2PI;
        } else {
            start_freq = next_test_freq;
        }
        stop_freq = start_freq;
        filter_freq = start_freq;
        
        attitude_control->bf_feedforward(false);

        // variables needed to initialize frequency response object and test method
        resp_type = AC_AutoTune_FreqResp::ResponseType::RATE;
        calc_type = RATE;
        freq_resp_input = TARGET;
        pre_calc_cycles = 1.0f;
        num_dwell_cycles = 3;
        break;
    case TuneType::MAX_GAINS:
        // initialize start frequency for sweep
        if (!is_positive(next_test_freq)) {
            start_freq = min_sweep_freq;
            stop_freq = max_sweep_freq;
            sweep_complete = true;
        } else {
            start_freq = next_test_freq;
            stop_freq = start_freq;
            test_accel_max_cdss = 0.0f;
        }
        filter_freq = start_freq;

        attitude_control->bf_feedforward(false);

        // variables needed to initialize frequency response object and test method
        resp_type = AC_AutoTune_FreqResp::ResponseType::RATE;
        calc_type = RATE;
        freq_resp_input = MOTOR;
        pre_calc_cycles = 6.25f;
        num_dwell_cycles = 6;
        break;
    case TuneType::RATE_P_UP:
    case TuneType::RATE_D_UP:
        // initialize start frequency
        if (!is_positive(next_test_freq)) {
            // continue using frequency where testing left off with RATE_D_UP completed
            if (curr_data.phase > 150.0f && curr_data.phase < 180.0f && tune_type == TuneType::RATE_P_UP) {
                start_freq = curr_data.freq;
            // start with freq found for sweep where phase was 180 deg
            } else if (!is_zero(sweep_tgt.ph180.freq)) {
                start_freq = sweep_tgt.ph180.freq;
            // otherwise start at min freq to step up in dwell frequency until phase > 160 deg
            } else {
                start_freq = min_sweep_freq;
            }
        } else {
            start_freq = next_test_freq;
        }
        stop_freq = start_freq;
        filter_freq = start_freq;

        attitude_control->bf_feedforward(false);

        // variables needed to initialize frequency response object and test method
        resp_type = AC_AutoTune_FreqResp::ResponseType::RATE;
        calc_type = RATE;
        freq_resp_input = TARGET;
        pre_calc_cycles = 6.25f;
        num_dwell_cycles = 6;
        break;
    case TuneType::ANGLE_P_UP:
        // initialize start frequency for sweep
        if (!is_positive(next_test_freq)) {
            start_freq = min_sweep_freq;
            stop_freq = max_sweep_freq;
            sweep_complete = true;
        } else {
            start_freq = next_test_freq;
            stop_freq = start_freq;
            test_accel_max_cdss = 0.0f;
        }
        filter_freq = start_freq;
        attitude_control->bf_feedforward(false);

        // variables needed to initialize frequency response object and test method
        resp_type = AC_AutoTune_FreqResp::ResponseType::ANGLE;
        calc_type = DRB;
        freq_resp_input = TARGET;
        pre_calc_cycles = 6.25f;
        num_dwell_cycles = 6;
        break;
    case TuneType::TUNE_CHECK:
        // initialize start frequency
        start_freq = min_sweep_freq;
        stop_freq = max_sweep_freq;
        test_accel_max_cdss = 0.0f;
        filter_freq = start_freq;

        // variables needed to initialize frequency response object and test method
        resp_type = AC_AutoTune_FreqResp::ResponseType::ANGLE;
        calc_type = ANGLE;
        freq_resp_input = TARGET;
        break;
    default:
        break;
    }

    if (!is_equal(start_freq,stop_freq)) {
        input_type = AC_AutoTune_FreqResp::InputType::SWEEP;
    } else {
        input_type = AC_AutoTune_FreqResp::InputType::DWELL;
    }


    // initialize dwell test method
    dwell_test_init(start_freq, stop_freq, freq_resp_amplitude, filter_freq, freq_resp_input, calc_type, resp_type, input_type);
}

// run tests for each tune type
void AC_AutoTune_Heli::test_run(AxisType test_axis, const float dir_sign)
{
    // if tune complete or beyond frequency range or no max allowed gains then exit testing
    if (tune_type == TuneType::TUNE_COMPLETE ||
       ((tune_type == TuneType::RATE_P_UP || tune_type == TuneType::RATE_D_UP) && (max_rate_p.max_allowed <= 0.0f || max_rate_d.max_allowed <= 0.0f)) ||
       ((tune_type == TuneType::MAX_GAINS || tune_type == TuneType::RATE_P_UP || tune_type == TuneType::RATE_D_UP || tune_type == TuneType::ANGLE_P_UP) && exceeded_freq_range(start_freq))){

        load_gains(GainType::ORIGINAL);

        // hold level attitude
        attitude_control->input_euler_angle_roll_pitch_yaw_rad(desired_roll_rad, desired_pitch_rad, desired_yaw_rad, true);

        if ((tune_type == TuneType::RATE_P_UP || tune_type == TuneType::RATE_D_UP) && (max_rate_p.max_allowed <= 0.0f || max_rate_d.max_allowed <= 0.0f)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Max Gain Determination Failed");
            mode = TuneMode::FAILED;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_FAILED);
            update_gcs(AUTOTUNE_MESSAGE_FAILED);
        } else if ((tune_type == TuneType::MAX_GAINS || tune_type == TuneType::RATE_P_UP || tune_type == TuneType::RATE_D_UP || tune_type == TuneType::ANGLE_P_UP) && exceeded_freq_range(start_freq)){
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Exceeded frequency range");
            mode = TuneMode::FAILED;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_FAILED);
            update_gcs(AUTOTUNE_MESSAGE_FAILED);
        } else if (tune_type == TuneType::TUNE_COMPLETE) {
            success_counter = AUTOTUNE_SUCCESS_COUNT;
            step = Step::UPDATE_GAINS;
        }
        return;
    }

    dwell_test_run(curr_data);

}

// heli specific gcs announcements
void AC_AutoTune_Heli::do_gcs_announcements()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_announce_ms < AUTOTUNE_ANNOUNCE_INTERVAL_MS) {
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: %s %s", get_axis_name(), get_tune_type_name());
    send_step_string();
    switch (tune_type) {
    case TuneType::RATE_FF_UP:
    case TuneType::RATE_D_UP:
    case TuneType::RATE_P_UP:
    case TuneType::MAX_GAINS:
    case TuneType::ANGLE_P_UP:
    case TuneType::TUNE_CHECK:
        if (is_equal(start_freq,stop_freq)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Dwell");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Sweep");
            if (settle_time == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f phase=%f", (double)(curr_test.freq), (double)(curr_test.gain), (double)(curr_test.phase));
            }
        }
        break;
    default:
        break;
    }

    last_announce_ms = now_ms;
}

// send post test updates to user
void AC_AutoTune_Heli::do_post_test_gcs_announcements() {
    float tune_rp = 0.0f;
    float tune_rd = 0.0f;
    float tune_rff = 0.0f;
    float tune_sp = 0.0f;
    float tune_accel_radss = 0.0f;

    switch (axis) {
    case AxisType::ROLL:
        tune_rp = tune_roll_rp;
        tune_rd = tune_roll_rd;
        tune_rff = tune_roll_rff;
        tune_sp = tune_roll_sp;
        tune_accel_radss = tune_roll_accel_radss;
        break;
    case AxisType::PITCH:
        tune_rp = tune_pitch_rp;
        tune_rd = tune_pitch_rd;
        tune_rff = tune_pitch_rff;
        tune_sp = tune_pitch_sp;
        tune_accel_radss =  tune_pitch_accel_radss;
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        tune_rp = tune_yaw_rp;
        tune_rd = tune_yaw_rd;
        tune_rff = tune_yaw_rff;
        tune_sp = tune_yaw_sp;
        tune_accel_radss =  tune_yaw_accel_radss;
        break;
    }

    if (step == Step::UPDATE_GAINS) {
        switch (tune_type) {
        case TuneType::RATE_FF_UP:
        case TuneType::RATE_P_UP:
        case TuneType::RATE_D_UP:
        case TuneType::ANGLE_P_UP:
        case TuneType::MAX_GAINS:
            if (is_equal(start_freq,stop_freq)) {
                // announce results of dwell
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f", (double)(curr_data.freq), (double)(curr_data.gain));
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: ph=%f", (double)(curr_data.phase));
               if (tune_type == TuneType::RATE_P_UP) {
                   GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: rate_p=%f", (double)(tune_rp));
               } else if (tune_type == TuneType::RATE_D_UP) {
                   GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: rate_d=%f", (double)(tune_rd));
               } else if (tune_type == TuneType::RATE_FF_UP) {
                   GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: rate_ff=%f", (double)(tune_rff));
               } else if (tune_type == TuneType::ANGLE_P_UP) {
                   GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: angle_p=%f tune_accel=%f max_accel=%f", (double)(tune_sp), (double)(rad_to_cd(tune_accel_radss)), (double)(test_accel_max_cdss));
               }
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: max_freq=%f max_gain=%f", (double)(sweep_tgt.maxgain.freq), (double)(sweep_tgt.maxgain.gain));
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: ph180_freq=%f ph180_gain=%f", (double)(sweep_tgt.ph180.freq), (double)(sweep_tgt.ph180.gain));
            }
            break;
        default:
            break;
        }
    }
}

// backup_gains_and_initialise - store current gains as originals
//  called before tuning starts to backup original gains
void AC_AutoTune_Heli::backup_gains_and_initialise()
{
    AC_AutoTune::backup_gains_and_initialise();

    // initializes dwell test sequence for rate_p_up and rate_d_up tests for tradheli
    next_test_freq = 0.0f;
    start_freq = 0.0f;
    stop_freq = 0.0f;

    orig_bf_feedforward = attitude_control->get_bf_feedforward();

    // backup original pids and initialise tuned pid values
    orig_roll_rp = attitude_control->get_rate_roll_pid().kP();
    orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
    orig_roll_rd = attitude_control->get_rate_roll_pid().kD();
    orig_roll_rff = attitude_control->get_rate_roll_pid().ff();
    orig_roll_fltt = attitude_control->get_rate_roll_pid().filt_T_hz();
    orig_roll_smax = attitude_control->get_rate_roll_pid().slew_limit();
    orig_roll_sp = attitude_control->get_angle_roll_p().kP();
    orig_roll_accel_radss = attitude_control->get_accel_roll_max_radss();
    orig_roll_rate_rads = attitude_control->get_ang_vel_roll_max_rads();
    tune_roll_rp = attitude_control->get_rate_roll_pid().kP();
    tune_roll_rd = attitude_control->get_rate_roll_pid().kD();
    tune_roll_rff = attitude_control->get_rate_roll_pid().ff();
    tune_roll_sp = attitude_control->get_angle_roll_p().kP();
    tune_roll_accel_radss = attitude_control->get_accel_roll_max_radss();

    orig_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
    orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
    orig_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
    orig_pitch_rff = attitude_control->get_rate_pitch_pid().ff();
    orig_pitch_fltt = attitude_control->get_rate_pitch_pid().filt_T_hz();
    orig_pitch_smax = attitude_control->get_rate_pitch_pid().slew_limit();
    orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
    orig_pitch_accel_radss = attitude_control->get_accel_pitch_max_radss();
    orig_pitch_rate_rads = attitude_control->get_ang_vel_pitch_max_rads();
    tune_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
    tune_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
    tune_pitch_rff = attitude_control->get_rate_pitch_pid().ff();
    tune_pitch_sp = attitude_control->get_angle_pitch_p().kP();
    tune_pitch_accel_radss = attitude_control->get_accel_pitch_max_radss();

    orig_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
    orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
    orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
    orig_yaw_rff = attitude_control->get_rate_yaw_pid().ff();
    orig_yaw_fltt = attitude_control->get_rate_yaw_pid().filt_T_hz();
    orig_yaw_smax = attitude_control->get_rate_yaw_pid().slew_limit();
    orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_E_hz();
    orig_yaw_accel_radss = attitude_control->get_accel_yaw_max_radss();
    orig_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    orig_yaw_rate_rads = attitude_control->get_ang_vel_yaw_max_rads();
    tune_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
    tune_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
    tune_yaw_rff = attitude_control->get_rate_yaw_pid().ff();
    tune_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_E_hz();
    tune_yaw_sp = attitude_control->get_angle_yaw_p().kP();
    tune_yaw_accel_radss = attitude_control->get_accel_yaw_max_radss();

    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_INITIALISED);
}

// load_orig_gains - set gains to their original values
//  called by stop and failed functions
void AC_AutoTune_Heli::load_orig_gains()
{
    attitude_control->use_sqrt_controller(true);
    attitude_control->bf_feedforward(orig_bf_feedforward);
    if (roll_enabled()) {
        load_gain_set(AxisType::ROLL, orig_roll_rp, orig_roll_ri, orig_roll_rd, orig_roll_rff, orig_roll_sp, orig_roll_accel_radss, orig_roll_fltt, 0.0f, orig_roll_smax, orig_roll_rate_rads);
    }
    if (pitch_enabled()) {
        load_gain_set(AxisType::PITCH, orig_pitch_rp, orig_pitch_ri, orig_pitch_rd, orig_pitch_rff, orig_pitch_sp, orig_pitch_accel_radss, orig_pitch_fltt, 0.0f, orig_pitch_smax, orig_pitch_rate_rads);
    }
    if (yaw_enabled()) {
        load_gain_set(AxisType::YAW, orig_yaw_rp, orig_yaw_ri, orig_yaw_rd, orig_yaw_rff, orig_yaw_sp, orig_yaw_accel_radss, orig_yaw_fltt, orig_yaw_rLPF, orig_yaw_smax, orig_yaw_rate_rads);
    }
}

// load_tuned_gains - load tuned gains
void AC_AutoTune_Heli::load_tuned_gains()
{
    attitude_control->use_sqrt_controller(true);
    if (!attitude_control->get_bf_feedforward()) {
        attitude_control->bf_feedforward(true);
        attitude_control->set_accel_roll_max_radss(0.0f);
        attitude_control->set_accel_pitch_max_radss(0.0f);
    }
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL) && roll_enabled()) {
        load_gain_set(AxisType::ROLL, tune_roll_rp, tune_roll_rff*AUTOTUNE_FFI_RATIO_FINAL, tune_roll_rd, tune_roll_rff, tune_roll_sp, tune_roll_accel_radss, orig_roll_fltt, 0.0f, orig_roll_smax, orig_roll_rate_rads);
    }
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) && pitch_enabled()) {
        load_gain_set(AxisType::PITCH, tune_pitch_rp, tune_pitch_rff*AUTOTUNE_FFI_RATIO_FINAL, tune_pitch_rd, tune_pitch_rff, tune_pitch_sp, tune_pitch_accel_radss, orig_pitch_fltt, 0.0f, orig_pitch_smax, orig_pitch_rate_rads);
    }
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_YAW) && yaw_enabled() && !is_zero(tune_yaw_rp)) {
        load_gain_set(AxisType::YAW, tune_yaw_rp, tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL, tune_yaw_rd, tune_yaw_rff, tune_yaw_sp, tune_yaw_accel_radss, orig_yaw_fltt, tune_yaw_rLPF, orig_yaw_smax, orig_yaw_rate_rads);
    }
}

// load_intra_test_gains - gains used between tests
//  called during testing mode's update-gains step to set gains ahead of return-to-level step
void AC_AutoTune_Heli::load_intra_test_gains()
{
    // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
    // sanity check the gains
    attitude_control->use_sqrt_controller(true);
    attitude_control->bf_feedforward(true);
    if (roll_enabled()) {
        load_gain_set(AxisType::ROLL, orig_roll_rp, orig_roll_rff * AUTOTUNE_FFI_RATIO_FOR_TESTING, orig_roll_rd, orig_roll_rff, orig_roll_sp, orig_roll_accel_radss, orig_roll_fltt, 0.0f, orig_roll_smax, orig_roll_rate_rads);
    }
    if (pitch_enabled()) {
        load_gain_set(AxisType::PITCH, orig_pitch_rp, orig_pitch_rff * AUTOTUNE_FFI_RATIO_FOR_TESTING, orig_pitch_rd, orig_pitch_rff, orig_pitch_sp, orig_pitch_accel_radss, orig_pitch_fltt, 0.0f, orig_pitch_smax, orig_pitch_rate_rads);
    }
    if (yaw_enabled()) {
        load_gain_set(AxisType::YAW, orig_yaw_rp, orig_yaw_rp*AUTOTUNE_PI_RATIO_FOR_TESTING, orig_yaw_rd, orig_yaw_rff, orig_yaw_sp, orig_yaw_accel_radss, orig_yaw_fltt, orig_yaw_rLPF, orig_yaw_smax, orig_yaw_rate_rads);
    }
}

// load_test_gains - load the to-be-tested gains for a single axis
// called by control_attitude() just before it beings testing a gain
void AC_AutoTune_Heli::load_test_gains()
{
    attitude_control->use_sqrt_controller(true);
    float rate_p, rate_i, rate_d, rate_test_max_rads, accel_test_max_radss;
    switch (axis) {
    case AxisType::ROLL:

        if (tune_type == TuneType::TUNE_CHECK) {
            rate_test_max_rads = orig_roll_rate_rads;
            accel_test_max_radss = tune_roll_accel_radss;
        } else {
            // have attitude controller not perform rate limiting and angle P scaling based on accel max
            rate_test_max_rads = 0.0;
            accel_test_max_radss = 0.0;
        }
        if (tune_type == TuneType::ANGLE_P_UP || tune_type == TuneType::TUNE_CHECK) {
            rate_i = tune_roll_rff*AUTOTUNE_FFI_RATIO_FINAL;
        } else {
            // freeze integrator to hold trim by making i term small during rate controller tuning
            rate_i = 0.01f * orig_roll_ri;
        }
        if (tune_type == TuneType::MAX_GAINS && !is_zero(tune_roll_rff)) {
            rate_p = 0.0f;
            rate_d = 0.0f;
        } else {
            rate_p = tune_roll_rp;
            rate_d = tune_roll_rd;
        }
        load_gain_set(AxisType::ROLL, rate_p, rate_i, rate_d, tune_roll_rff, tune_roll_sp, accel_test_max_radss, orig_roll_fltt, 0.0f, 0.0f, rate_test_max_rads);
        break;
    case AxisType::PITCH:
        if (tune_type == TuneType::TUNE_CHECK) {
            rate_test_max_rads = orig_pitch_rate_rads;
            accel_test_max_radss = tune_pitch_accel_radss;
        } else {
            // have attitude controller not perform rate limiting and angle P scaling based on accel max
            rate_test_max_rads = 0.0;
            accel_test_max_radss = 0.0;
        }
        if (tune_type == TuneType::ANGLE_P_UP || tune_type == TuneType::TUNE_CHECK) {
            rate_i = tune_pitch_rff*AUTOTUNE_FFI_RATIO_FINAL;
        } else {
            // freeze integrator to hold trim by making i term small during rate controller tuning
            rate_i = 0.01f * orig_pitch_ri;
        }
        if (tune_type == TuneType::MAX_GAINS && !is_zero(tune_pitch_rff)) {
            rate_p = 0.0f;
            rate_d = 0.0f;
        } else {
            rate_p = tune_pitch_rp;
            rate_d = tune_pitch_rd;
        }
        load_gain_set(AxisType::PITCH, rate_p, rate_i, rate_d, tune_pitch_rff, tune_pitch_sp, accel_test_max_radss, orig_pitch_fltt, 0.0f, 0.0f, rate_test_max_rads);
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        if (tune_type == TuneType::TUNE_CHECK) {
            rate_test_max_rads = orig_yaw_rate_rads;
            accel_test_max_radss = tune_yaw_accel_radss;
        } else {
            // have attitude controller not perform rate limiting and angle P scaling based on accel max
            rate_test_max_rads = 0.0;
            accel_test_max_radss = 0.0;
        }
        if (tune_type == TuneType::ANGLE_P_UP || tune_type == TuneType::TUNE_CHECK) {
            rate_i = tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL;
        } else {
            // freeze integrator to hold trim by making i term small during rate controller tuning
            rate_i = 0.01f * orig_yaw_ri;
        }
        load_gain_set(AxisType::YAW, tune_yaw_rp, rate_i, tune_yaw_rd, tune_yaw_rff, tune_yaw_sp, accel_test_max_radss, orig_yaw_fltt, tune_yaw_rLPF, 0.0f, rate_test_max_rads);
        break;
    }
}

// load gains
void AC_AutoTune_Heli::load_gain_set(AxisType s_axis, float rate_p, float rate_i, float rate_d, float rate_ff, float angle_p, float max_accel_radss, float rate_fltt, float rate_flte, float smax, float max_rate_rads)
{
    switch (s_axis) {
    case AxisType::ROLL:
        attitude_control->get_rate_roll_pid().set_kP(rate_p);
        attitude_control->get_rate_roll_pid().set_kI(rate_i);
        attitude_control->get_rate_roll_pid().set_kD(rate_d);
        attitude_control->get_rate_roll_pid().set_ff(rate_ff);
        attitude_control->get_rate_roll_pid().set_filt_T_hz(rate_fltt);
        attitude_control->get_rate_roll_pid().set_slew_limit(smax);
        attitude_control->get_angle_roll_p().set_kP(angle_p);
        attitude_control->set_accel_roll_max_radss(max_accel_radss);
        attitude_control->set_ang_vel_roll_max_rads(max_rate_rads);
        break;
    case AxisType::PITCH:
        attitude_control->get_rate_pitch_pid().set_kP(rate_p);
        attitude_control->get_rate_pitch_pid().set_kI(rate_i);
        attitude_control->get_rate_pitch_pid().set_kD(rate_d);
        attitude_control->get_rate_pitch_pid().set_ff(rate_ff);
        attitude_control->get_rate_pitch_pid().set_filt_T_hz(rate_fltt);
        attitude_control->get_rate_pitch_pid().set_slew_limit(smax);
        attitude_control->get_angle_pitch_p().set_kP(angle_p);
        attitude_control->set_accel_pitch_max_radss(max_accel_radss);
        attitude_control->set_ang_vel_pitch_max_rads(max_rate_rads);
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        attitude_control->get_rate_yaw_pid().set_kP(rate_p);
        attitude_control->get_rate_yaw_pid().set_kI(rate_i);
        attitude_control->get_rate_yaw_pid().set_kD(rate_d);
        attitude_control->get_rate_yaw_pid().set_ff(rate_ff);
        attitude_control->get_rate_yaw_pid().set_filt_T_hz(rate_fltt);
        attitude_control->get_rate_yaw_pid().set_slew_limit(smax);
        attitude_control->get_rate_yaw_pid().set_filt_E_hz(rate_flte);
        attitude_control->get_angle_yaw_p().set_kP(angle_p);
        attitude_control->set_accel_yaw_max_radss(max_accel_radss);
        attitude_control->set_ang_vel_yaw_max_rads(max_rate_rads);
        break;
    }
}

// save_tuning_gains - save the final tuned gains for each axis
// save discovered gains to eeprom if autotuner is enabled (i.e. switch is in the high position)
void AC_AutoTune_Heli::save_tuning_gains()
{
    // see if we successfully completed tuning of at least one axis
    if (axes_completed == 0) {
        return;
    }

    if (!attitude_control->get_bf_feedforward()) {
        attitude_control->bf_feedforward_save(true);
        attitude_control->save_accel_roll_max_radss(0.0f);
        attitude_control->save_accel_pitch_max_radss(0.0f);
    }

    // sanity check the rate P values
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL) && roll_enabled()) {
        load_gain_set(AxisType::ROLL, tune_roll_rp, tune_roll_rff*AUTOTUNE_FFI_RATIO_FINAL, tune_roll_rd, tune_roll_rff, tune_roll_sp, tune_roll_accel_radss, orig_roll_fltt, 0.0f, orig_roll_smax, orig_roll_rate_rads);
        // save rate roll gains
        attitude_control->get_rate_roll_pid().save_gains();

        // save stabilize roll
        attitude_control->get_angle_roll_p().save_gains();

        // resave pids to originals in case the autotune is run again
        orig_roll_rp = attitude_control->get_rate_roll_pid().kP();
        orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
        orig_roll_rd = attitude_control->get_rate_roll_pid().kD();
        orig_roll_rff = attitude_control->get_rate_roll_pid().ff();
        orig_roll_sp = attitude_control->get_angle_roll_p().kP();
        orig_roll_accel_radss = attitude_control->get_accel_roll_max_radss();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) && pitch_enabled()) {
        load_gain_set(AxisType::PITCH, tune_pitch_rp, tune_pitch_rff*AUTOTUNE_FFI_RATIO_FINAL, tune_pitch_rd, tune_pitch_rff, tune_pitch_sp, tune_pitch_accel_radss, orig_pitch_fltt, 0.0f, orig_pitch_smax, orig_pitch_rate_rads);
        // save rate pitch gains
        attitude_control->get_rate_pitch_pid().save_gains();

        // save stabilize pitch
        attitude_control->get_angle_pitch_p().save_gains();

        // resave pids to originals in case the autotune is run again
        orig_pitch_rp = attitude_control->get_rate_pitch_pid().kP();
        orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
        orig_pitch_rd = attitude_control->get_rate_pitch_pid().kD();
        orig_pitch_rff = attitude_control->get_rate_pitch_pid().ff();
        orig_pitch_sp = attitude_control->get_angle_pitch_p().kP();
        orig_pitch_accel_radss = attitude_control->get_accel_pitch_max_radss();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_YAW) && yaw_enabled() && !is_zero(tune_yaw_rp)) {
        load_gain_set(AxisType::YAW, tune_yaw_rp, tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL, tune_yaw_rd, tune_yaw_rff, tune_yaw_sp, tune_yaw_accel_radss, orig_yaw_fltt, orig_yaw_rLPF, orig_yaw_smax, orig_yaw_rate_rads);
        // save rate yaw gains
        attitude_control->get_rate_yaw_pid().save_gains();

        // save stabilize yaw
        attitude_control->get_angle_yaw_p().save_gains();

        // resave pids to originals in case the autotune is run again
        orig_yaw_rp = attitude_control->get_rate_yaw_pid().kP();
        orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
        orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
        orig_yaw_rff = attitude_control->get_rate_yaw_pid().ff();
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
void AC_AutoTune_Heli::report_final_gains(AxisType test_axis) const
{
    switch (test_axis) {
        case AxisType::ROLL:
            report_axis_gains("Roll", tune_roll_rp, tune_roll_rff*AUTOTUNE_FFI_RATIO_FINAL, tune_roll_rd, tune_roll_rff, tune_roll_sp, tune_roll_accel_radss);
            break;
        case AxisType::PITCH:
            report_axis_gains("Pitch", tune_pitch_rp, tune_pitch_rff*AUTOTUNE_FFI_RATIO_FINAL, tune_pitch_rd, tune_pitch_rff, tune_pitch_sp, tune_pitch_accel_radss);
            break;
        case AxisType::YAW:
        case AxisType::YAW_D:
            report_axis_gains("Yaw", tune_yaw_rp, tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL, tune_yaw_rd, tune_yaw_rff, tune_yaw_sp, tune_yaw_accel_radss);
            break;
    }
}

// report gain formatting helper
void AC_AutoTune_Heli::report_axis_gains(const char* axis_string, float rate_P, float rate_I, float rate_D, float rate_ff, float angle_P, float max_accel_radss) const
{
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: %s complete", axis_string);
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: %s Rate: P:%0.4f, I:%0.4f, D:%0.5f, FF:%0.4f", axis_string, rate_P, rate_I, rate_D, rate_ff);
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AutoTune: %s Angle P:%0.2f, Max Accel:%0.0f", axis_string, angle_P, rad_to_cd(max_accel_radss));
}

void AC_AutoTune_Heli::dwell_test_init(float start_frq, float stop_frq, float amplitude, float filt_freq, FreqRespInput freq_resp_input, FreqRespCalcType calc_type, AC_AutoTune_FreqResp::ResponseType resp_type, AC_AutoTune_FreqResp::InputType waveform_input_type)
{
    test_input_type = waveform_input_type;
    test_freq_resp_input = freq_resp_input;
    test_calc_type = calc_type;
    test_start_freq = start_frq;
    //target attitude magnitude
    tgt_attitude = radians(amplitude);

    // initialize frequency response object
    if (test_input_type == AC_AutoTune_FreqResp::InputType::SWEEP) {
        step_timeout_ms = sweep_time_ms + 500;
        reset_sweep_variables();
        curr_test.gain = 0.0f;
        curr_test.phase = 0.0f;
        chirp_input.init(0.001f * sweep_time_ms, start_frq / M_2PI, stop_frq / M_2PI, 0.0f, 0.0001f * sweep_time_ms, 0.0f);
    } else {
        if (!is_zero(start_frq)) {
            // time limit set by adding the pre calc cycles with the dwell cycles.  500 ms added to account for settling with buffer.
            step_timeout_ms = (uint32_t) (2000 + ((float)num_dwell_cycles + pre_calc_cycles + 2.0f) * 1000.0f * M_2PI / start_frq);
        }
        chirp_input.init(0.001f * step_timeout_ms, start_frq / M_2PI, stop_frq / M_2PI, 0.0f, 0.0001f * step_timeout_ms, 0.0f);
    }

    freqresp_tgt.init(test_input_type, resp_type, num_dwell_cycles);
    freqresp_mtr.init(test_input_type, resp_type, num_dwell_cycles);
    
    dwell_start_time_ms = 0.0f;
    settle_time = 200;

    rotation_rate_filt.set_cutoff_frequency(filt_freq);
    command_filt.set_cutoff_frequency(filt_freq);
    target_rate_filt.set_cutoff_frequency(filt_freq);

    rotation_rate_filt.reset(0);
    command_filt.reset(0);
    target_rate_filt.reset(0);
    rotation_rate = 0.0f;
    command_out = 0.0f;
    filt_target_rate = 0.0f;

    // filter at lower frequency to remove steady state
    filt_command_reading.set_cutoff_frequency(0.2f * filt_freq);
    filt_gyro_reading.set_cutoff_frequency(0.05f * filt_freq);
    filt_tgt_rate_reading.set_cutoff_frequency(0.05f * filt_freq);
    filt_att_fdbk_from_velxy_cd.set_cutoff_frequency(0.2f * filt_freq);

    curr_test_mtr = {};
    curr_test_tgt = {};
    cycle_complete_tgt = false;
    cycle_complete_mtr = false;
    sweep_complete = false;

}

void AC_AutoTune_Heli::dwell_test_run(sweep_info &test_data)
{
    float gyro_reading = 0.0f;
    float command_reading = 0.0f;
    float tgt_rate_reading = 0.0f;
    const uint32_t now_ms = AP_HAL::millis();
    float target_angle_cd = 0.0f;
    float dwell_freq = test_start_freq;

    float cycle_time_ms = 0;
    if (!is_zero(dwell_freq)) {
        cycle_time_ms = 1000.0f * M_2PI / dwell_freq;
    }

    // body frame calculation of velocity
    Vector3f velocity_ned, velocity_bf;
    if (ahrs_view->get_velocity_NED(velocity_ned)) {
        velocity_bf.x = velocity_ned.x * ahrs_view->cos_yaw() + velocity_ned.y * ahrs_view->sin_yaw();
        velocity_bf.y = -velocity_ned.x * ahrs_view->sin_yaw() + velocity_ned.y * ahrs_view->cos_yaw();
    }

    if (settle_time == 0) {
        dwell_freq = chirp_input.get_frequency_rads();
        float tgt_att_limited = tgt_attitude;
        if (is_positive(dwell_freq)) {
            float tgt_att_temp = tgt_attitude;
            if (is_positive(rate_max_degs)) {
                float ang_limit_rate = radians(rate_max_degs) / dwell_freq;
                tgt_att_temp = MIN(ang_limit_rate, tgt_attitude);
            }
            if (is_positive(accel_max_degss)) {
                float ang_limit_accel_radss = radians(accel_max_degss) / sq(dwell_freq);
                tgt_att_limited = MIN(ang_limit_accel_radss, tgt_att_temp);
            } else {
                tgt_att_limited = tgt_att_temp;
            }
        }
        target_angle_cd = -chirp_input.update((now_ms - dwell_start_time_ms) * 0.001, degrees(tgt_att_limited) * 100.0f);
        dwell_freq = chirp_input.get_frequency_rads();
        const Vector2f att_fdbk {
            -5730.0f * vel_hold_gain * velocity_bf.y,
            5730.0f * vel_hold_gain * velocity_bf.x
        };
        filt_att_fdbk_from_velxy_cd.apply(att_fdbk, AP::scheduler().get_loop_period_s());
    } else {
        target_angle_cd = 0.0f;
        trim_yaw_tgt_reading_cd = (float)attitude_control->get_att_target_euler_cd().z;
        trim_yaw_heading_reading_cd = (float)ahrs_view->yaw_sensor;
        dwell_start_time_ms = now_ms;
        filt_att_fdbk_from_velxy_cd.reset(Vector2f(0.0f,0.0f));
        settle_time--;
    }

    const Vector2f trim_angle_cd {
        constrain_float(filt_att_fdbk_from_velxy_cd.get().x, -2000.0f, 2000.0f),
        constrain_float(filt_att_fdbk_from_velxy_cd.get().y, -2000.0f, 2000.0f)
    };

    switch (axis) {
    case AxisType::ROLL:
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(cd_to_rad(target_angle_cd + trim_angle_cd.x), cd_to_rad(trim_angle_cd.y), 0.0f);
        command_reading = motors->get_roll();
        if (test_calc_type == DRB) {
            tgt_rate_reading = cd_to_rad(target_angle_cd);
            gyro_reading = cd_to_rad((float)ahrs_view->roll_sensor + trim_angle_cd.x - target_angle_cd);
        } else if (test_calc_type == RATE) {
            tgt_rate_reading = attitude_control->rate_bf_targets().x;
            gyro_reading = ahrs_view->get_gyro().x;
        } else {
            tgt_rate_reading = cd_to_rad((float)attitude_control->get_att_target_euler_cd().x);
            gyro_reading = cd_to_rad((float)ahrs_view->roll_sensor);
        }
        break;
    case AxisType::PITCH:
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(cd_to_rad(trim_angle_cd.x), cd_to_rad(target_angle_cd + trim_angle_cd.y), 0.0f);
        command_reading = motors->get_pitch();
        if (test_calc_type == DRB) {
            tgt_rate_reading = cd_to_rad(target_angle_cd);
            gyro_reading = cd_to_rad((float)ahrs_view->pitch_sensor + trim_angle_cd.y - target_angle_cd);
        } else if (test_calc_type == RATE) {
            tgt_rate_reading = attitude_control->rate_bf_targets().y;
            gyro_reading = ahrs_view->get_gyro().y;
        } else {
            tgt_rate_reading = cd_to_rad((float)attitude_control->get_att_target_euler_cd().y);
            gyro_reading = cd_to_rad((float)ahrs_view->pitch_sensor);
        }
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        attitude_control->input_euler_angle_roll_pitch_yaw_rad(cd_to_rad(trim_angle_cd.x), cd_to_rad(trim_angle_cd.y), cd_to_rad(wrap_180_cd(trim_yaw_tgt_reading_cd + target_angle_cd)), false);
        command_reading = motors->get_yaw();
        if (test_calc_type == DRB) {
            tgt_rate_reading = cd_to_rad(target_angle_cd);
            gyro_reading = cd_to_rad(wrap_180_cd((float)ahrs_view->yaw_sensor - trim_yaw_heading_reading_cd - target_angle_cd));
        } else if (test_calc_type == RATE) {
            tgt_rate_reading = attitude_control->rate_bf_targets().z;
            gyro_reading = ahrs_view->get_gyro().z;
        } else {
            tgt_rate_reading = cd_to_rad(wrap_180_cd((float)attitude_control->get_att_target_euler_cd().z - trim_yaw_tgt_reading_cd));
            gyro_reading = cd_to_rad((wrap_180_cd((float)ahrs_view->yaw_sensor - trim_yaw_heading_reading_cd)));
        }
        break;
    }

    if (settle_time == 0) {
        filt_command_reading.apply(command_reading, AP::scheduler().get_loop_period_s());
        filt_gyro_reading.apply(gyro_reading, AP::scheduler().get_loop_period_s());
        filt_tgt_rate_reading.apply(tgt_rate_reading, AP::scheduler().get_loop_period_s());
    } else {
        filt_command_reading.reset(command_reading);
        filt_gyro_reading.reset(gyro_reading);
        filt_tgt_rate_reading.reset(tgt_rate_reading);
    }

    // looks at gain and phase of input rate to output rate
    rotation_rate = rotation_rate_filt.apply((gyro_reading - filt_gyro_reading.get()),
                AP::scheduler().get_loop_period_s());
    filt_target_rate = target_rate_filt.apply((tgt_rate_reading - filt_tgt_rate_reading.get()),
                AP::scheduler().get_loop_period_s());
    command_out = command_filt.apply((command_reading - filt_command_reading.get()),
                AP::scheduler().get_loop_period_s());

    float dwell_gain_mtr = 0.0f; 
    float dwell_phase_mtr = 0.0f;
    float dwell_gain_tgt = 0.0f;
    float dwell_phase_tgt = 0.0f;
    // wait for dwell to start before determining gain and phase
    if ((float)(now_ms - dwell_start_time_ms) > pre_calc_cycles * cycle_time_ms || (test_input_type == AC_AutoTune_FreqResp::InputType::SWEEP && settle_time == 0)) {
        freqresp_mtr.update(command_out,command_out,rotation_rate, dwell_freq);
        freqresp_tgt.update(command_out,filt_target_rate,rotation_rate, dwell_freq);

        if (freqresp_mtr.is_cycle_complete()) {
            if (test_input_type == AC_AutoTune_FreqResp::InputType::SWEEP) {
                if (is_zero(curr_test_mtr.freq) && freqresp_mtr.get_freq() < test_start_freq) {
                    // don't set data since captured frequency is below the start frequency
                } else {
                    curr_test_mtr.freq = freqresp_mtr.get_freq();
                    curr_test_mtr.gain = freqresp_mtr.get_gain();
                    curr_test_mtr.phase = freqresp_mtr.get_phase();
                }
                // reset cycle_complete to allow indication of next cycle
                freqresp_mtr.reset_cycle_complete();
#if HAL_LOGGING_ENABLED
                // log sweep data
                Log_AutoTuneSweep();
#endif
            } else {
                dwell_gain_mtr = freqresp_mtr.get_gain();
                dwell_phase_mtr = freqresp_mtr.get_phase();
                cycle_complete_mtr = true;
            }
        }

        if (freqresp_tgt.is_cycle_complete()) {
            if (test_input_type == AC_AutoTune_FreqResp::InputType::SWEEP) {
                if (is_zero(curr_test_tgt.freq) && freqresp_tgt.get_freq() < test_start_freq) {
                    // don't set data since captured frequency is below the start frequency
                } else {
                    curr_test_tgt.freq = freqresp_tgt.get_freq();
                    curr_test_tgt.gain = freqresp_tgt.get_gain();
                    curr_test_tgt.phase = freqresp_tgt.get_phase();
                    if (test_calc_type == DRB) {test_accel_max_cdss = freqresp_tgt.get_accel_max();}
                }
                // reset cycle_complete to allow indication of next cycle
                freqresp_tgt.reset_cycle_complete();
#if HAL_LOGGING_ENABLED
                // log sweep data
                Log_AutoTuneSweep();
#endif
            } else {
                dwell_gain_tgt = freqresp_tgt.get_gain();
                dwell_phase_tgt = freqresp_tgt.get_phase();
                if (test_calc_type == DRB) {test_accel_max_cdss = freqresp_tgt.get_accel_max();}
                cycle_complete_tgt = true;
            }
        }

        if (test_freq_resp_input == TARGET) {
            if (test_input_type == AC_AutoTune_FreqResp::InputType::SWEEP) {
                curr_test = curr_test_tgt;
            } else {
                test_data.freq = test_start_freq;
                test_data.gain = dwell_gain_tgt;
                test_data.phase = dwell_phase_tgt;
            }
        } else {
            if (test_input_type == AC_AutoTune_FreqResp::InputType::SWEEP) {
                curr_test = curr_test_mtr;
            } else {
                test_data.freq = test_start_freq;
                test_data.gain = dwell_gain_mtr;
                test_data.phase = dwell_phase_mtr;
            }
        }
    }

    // set sweep data if a frequency sweep is being conducted
    if (test_input_type == AC_AutoTune_FreqResp::InputType::SWEEP && (float)(now_ms - dwell_start_time_ms) > 2.5f * cycle_time_ms) {
        // track sweep phase to prevent capturing 180 deg and 270 deg data after phase has wrapped.
        if (curr_test_tgt.phase > 180.0f && sweep_tgt.progress == 0) {
            sweep_tgt.progress = 1;
        } else if (curr_test_tgt.phase > 270.0f && sweep_tgt.progress == 1) {
            sweep_tgt.progress = 2;
        }
        if (curr_test_tgt.phase <= 160.0f && curr_test_tgt.phase >= 150.0f && sweep_tgt.progress == 0) {
            sweep_tgt.ph180 = curr_test_tgt;
        }
        if (curr_test_tgt.phase <= 250.0f && curr_test_tgt.phase >= 240.0f && sweep_tgt.progress == 1) {
            sweep_tgt.ph270 = curr_test_tgt;
        }
        if (curr_test_tgt.gain > sweep_tgt.maxgain.gain) {
            sweep_tgt.maxgain = curr_test_tgt;
        }
        // Determine sweep info for motor input to response output
        if (curr_test_mtr.phase > 180.0f && sweep_mtr.progress == 0) {
            sweep_mtr.progress = 1;
        } else if (curr_test_mtr.phase > 270.0f && sweep_mtr.progress == 1) {
            sweep_mtr.progress = 2;
        }
        if (curr_test_mtr.phase <= 160.0f && curr_test_mtr.phase >= 150.0f && sweep_mtr.progress == 0) {
            sweep_mtr.ph180 = curr_test_mtr;
        }
        if (curr_test_mtr.phase <= 250.0f && curr_test_mtr.phase >= 240.0f && sweep_mtr.progress == 1) {
            sweep_mtr.ph270 = curr_test_mtr;
        }
        if (curr_test_mtr.gain > sweep_mtr.maxgain.gain) {
            sweep_mtr.maxgain = curr_test_mtr;
        }

        if (now_ms - step_start_time_ms >= sweep_time_ms + 200) {
            // we have passed the maximum stop time
            sweep_complete = true;
            step = Step::UPDATE_GAINS;
        }
    } else {
        if (now_ms - step_start_time_ms >= step_timeout_ms || (freqresp_tgt.is_cycle_complete() && freqresp_mtr.is_cycle_complete())) {
            if (now_ms - step_start_time_ms >= step_timeout_ms) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Step time limit exceeded");
            }
            cycle_complete_tgt = false;
            cycle_complete_tgt = false;
            // we have passed the maximum stop time
            step = Step::UPDATE_GAINS;
        }
    }
}

// update gains for the rate p up tune type
void AC_AutoTune_Heli::updating_rate_p_up_all(AxisType test_axis)
{
    switch (test_axis) {
    case AxisType::ROLL:
        updating_rate_p_up(tune_roll_rp, curr_data, next_test_freq, max_rate_p);
        break;
    case AxisType::PITCH:
        updating_rate_p_up(tune_pitch_rp, curr_data, next_test_freq, max_rate_p);
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        updating_rate_p_up(tune_yaw_rp, curr_data, next_test_freq, max_rate_p);
        break;
    }
}

// update gains for the rate d up tune type
void AC_AutoTune_Heli::updating_rate_d_up_all(AxisType test_axis)
{
    switch (test_axis) {
    case AxisType::ROLL:
        updating_rate_d_up(tune_roll_rd, curr_data, next_test_freq, max_rate_d);
        break;
    case AxisType::PITCH:
        updating_rate_d_up(tune_pitch_rd, curr_data, next_test_freq, max_rate_d);
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        updating_rate_d_up(tune_yaw_rd, curr_data, next_test_freq, max_rate_d);
        break;
    }
}

// update gains for the rate ff up tune type
void AC_AutoTune_Heli::updating_rate_ff_up_all(AxisType test_axis)
{
    switch (test_axis) {
    case AxisType::ROLL:
        updating_rate_ff_up(tune_roll_rff, curr_data, next_test_freq);
        break;
    case AxisType::PITCH:
        updating_rate_ff_up(tune_pitch_rff, curr_data, next_test_freq);
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        updating_rate_ff_up(tune_yaw_rff, curr_data, next_test_freq);
        // TODO make FF updating routine determine when to set rff gain to zero based on A/C response
        if (tune_yaw_rff <= AUTOTUNE_RFF_MIN && success_counter == AUTOTUNE_SUCCESS_COUNT) {
            tune_yaw_rff = 0.0f;
        }
        break;
    }
}

// update gains for the angle p up tune type
void AC_AutoTune_Heli::updating_angle_p_up_all(AxisType test_axis)
{
    attitude_control->bf_feedforward(orig_bf_feedforward);

    // sweep doesn't require gain update so return immediately after setting next test freq
    // determine next_test_freq for dwell testing
    if (sweep_complete && input_type == AC_AutoTune_FreqResp::InputType::SWEEP){
        // if a max gain frequency was found then set the start of the dwells to that freq otherwise start at min frequency
        if (!is_zero(sweep_tgt.maxgain.freq)) {
            next_test_freq = constrain_float(sweep_tgt.maxgain.freq, min_sweep_freq, max_sweep_freq);
            freq_max = next_test_freq;
            sp_prev_gain = sweep_tgt.maxgain.gain;
            phase_max = sweep_tgt.maxgain.phase;
            found_max_gain_freq = true;            
        } else {
            next_test_freq = min_sweep_freq;            
        }
        return;
    }

    switch (test_axis) {
    case AxisType::ROLL:
        updating_angle_p_up(tune_roll_sp, curr_data, next_test_freq);
        break;
    case AxisType::PITCH:
        updating_angle_p_up(tune_pitch_sp, curr_data, next_test_freq);
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        updating_angle_p_up(tune_yaw_sp, curr_data, next_test_freq);
        break;
    }
}

// update gains for the max gain tune type
void AC_AutoTune_Heli::updating_max_gains_all(AxisType test_axis)
{
    // sweep doesn't require gain update so return immediately after setting next test freq
    // determine next_test_freq for dwell testing
    if (sweep_complete && input_type == AC_AutoTune_FreqResp::InputType::SWEEP) {
        // if a max gain frequency was found then set the start of the dwells to that freq otherwise start at min frequency
        if (!is_zero(sweep_mtr.ph180.freq)) {
            next_test_freq = constrain_float(sweep_mtr.ph180.freq, min_sweep_freq, max_sweep_freq);
            reset_maxgains_update_gain_variables();
        } else {
            next_test_freq = min_sweep_freq;
        }
        return;
    }

    switch (test_axis) {
    case AxisType::ROLL:
        updating_max_gains(curr_data, next_test_freq, max_rate_p, max_rate_d, tune_roll_rp, tune_roll_rd);
        break;
    case AxisType::PITCH:
        updating_max_gains(curr_data, next_test_freq, max_rate_p, max_rate_d, tune_pitch_rp, tune_pitch_rd);
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        updating_max_gains(curr_data, next_test_freq, max_rate_p, max_rate_d, tune_yaw_rp, tune_yaw_rd);
        // rate P and rate D can be non zero for yaw and need to be included in the max allowed gain
        if (!is_zero(max_rate_p.max_allowed) && success_counter == AUTOTUNE_SUCCESS_COUNT) {
            max_rate_p.max_allowed += tune_yaw_rp;
        }
        if (!is_zero(max_rate_d.max_allowed) && success_counter == AUTOTUNE_SUCCESS_COUNT) {
            max_rate_d.max_allowed += tune_yaw_rd;
        }
        break;
    }
}

// set gains post tune for the tune type
void AC_AutoTune_Heli::set_tuning_gains_with_backoff(AxisType test_axis)
{
    switch (tune_type) {
    case TuneType::RATE_D_UP:
        switch (test_axis) {
        case AxisType::ROLL:
            tune_roll_rd = MAX(0.0f, tune_roll_rd * AUTOTUNE_RD_BACKOFF);
            break;
        case AxisType::PITCH:
            tune_pitch_rd = MAX(0.0f, tune_pitch_rd * AUTOTUNE_RD_BACKOFF);
            break;
        case AxisType::YAW:
        case AxisType::YAW_D:
            tune_yaw_rd = MAX(0.0f, tune_yaw_rd * AUTOTUNE_RD_BACKOFF);
            break;
        }
        break;
    case TuneType::RATE_P_UP:
        switch (test_axis) {
        case AxisType::ROLL:
            tune_roll_rp = MAX(0.0f, tune_roll_rp * AUTOTUNE_RP_BACKOFF);
            break;
        case AxisType::PITCH:
            tune_pitch_rp = MAX(0.0f, tune_pitch_rp * AUTOTUNE_RP_BACKOFF);
            break;
        case AxisType::YAW:
        case AxisType::YAW_D:
            tune_yaw_rp = MAX(AUTOTUNE_RP_MIN, tune_yaw_rp * AUTOTUNE_RP_BACKOFF);
            break;
        }
        break;
    case TuneType::ANGLE_P_UP:
        switch (test_axis) {
        case AxisType::ROLL:
            tune_roll_sp = MAX(AUTOTUNE_SP_MIN, tune_roll_sp * AUTOTUNE_SP_BACKOFF);
            break;
        case AxisType::PITCH:
            tune_pitch_sp = MAX(AUTOTUNE_SP_MIN, tune_pitch_sp * AUTOTUNE_SP_BACKOFF);
            break;
        case AxisType::YAW:
        case AxisType::YAW_D:
            tune_yaw_sp = MAX(AUTOTUNE_SP_MIN, tune_yaw_sp * AUTOTUNE_SP_BACKOFF);
            break;
        }
        break;
    case TuneType::RATE_FF_UP:
        break;
    default:
        break;
    }
}

// updating_rate_ff_up - adjust FF to ensure the target is reached
// FF is adjusted until rate requested is achieved
void AC_AutoTune_Heli::updating_rate_ff_up(float &tune_ff, sweep_info &test_data, float &next_freq)
{
    float tune_tgt = 0.95;
    float tune_tol = 0.025;
    next_freq = test_data.freq;

    // handle axes where FF gain is initially zero
    if (test_data.gain < tune_tgt - tune_tol && !is_positive(tune_ff)) {
        tune_ff = 0.03f;
        return;
    }

    if (test_data.gain < tune_tgt - 0.2 || test_data.gain > tune_tgt + 0.2) {
        tune_ff =  tune_ff * constrain_float(tune_tgt / test_data.gain, 0.75, 1.25);  //keep changes less than 25%
    } else if (test_data.gain < tune_tgt - 0.1 || test_data.gain > tune_tgt + 0.1) {
        if (test_data.gain < tune_tgt - 0.1) {
            tune_ff *= 1.05;
        } else {
            tune_ff *= 0.95;
        }
    } else if (test_data.gain < tune_tgt - tune_tol || test_data.gain > tune_tgt + tune_tol) {
        if (test_data.gain < tune_tgt - tune_tol) {
            tune_ff *= 1.02;
        } else {
            tune_ff *= 0.98;
        }
    } else if (test_data.gain >= tune_tgt - tune_tol && test_data.gain <= tune_tgt + tune_tol) {
        success_counter = AUTOTUNE_SUCCESS_COUNT;
        // reset next_freq for next test
        next_freq = 0.0f;
        tune_ff = constrain_float(tune_ff,0.0f,1.0f);
    }
}

// updating_rate_p_up - uses maximum allowable gain determined from max_gain test to determine rate p gain that does not
// exceed max response gain.  A phase of 161 deg is used to conduct the tuning as this phase is where analytically
// max gain to 6db gain margin is determined for a unity feedback controller.
void AC_AutoTune_Heli::updating_rate_p_up(float &tune_p, sweep_info &test_data, float &next_freq, max_gain_data &max_gain_p)
{
    float test_freq_incr = 0.25f * M_2PI;
    next_freq = test_data.freq;

    sweep_info data_at_ph161;
    float sugg_freq;
    if (freq_search_for_phase(test_data, 161.0f, test_freq_incr, data_at_ph161, sugg_freq)) {
        if (data_at_ph161.gain < max_resp_gain && tune_p < 0.6f * max_gain_p.max_allowed) {
            tune_p += 0.05f * max_gain_p.max_allowed;
            next_freq = data_at_ph161.freq;
        } else {
            success_counter = AUTOTUNE_SUCCESS_COUNT;
            // reset next_freq for next test
            next_freq = 0.0f;
            tune_p -= 0.05f * max_gain_p.max_allowed;
            tune_p = constrain_float(tune_p,0.0f,0.6f * max_gain_p.max_allowed);
        }
    } else {
        next_freq = sugg_freq;
    }
}

// updating_rate_d_up - uses maximum allowable gain determined from max_gain test to determine rate d gain where the response
// gain is at a minimum.  A phase of 161 deg is used to conduct the tuning as this phase is where analytically
// max gain to 6db gain margin is determined for a unity feedback controller.
void AC_AutoTune_Heli::updating_rate_d_up(float &tune_d, sweep_info &test_data, float &next_freq, max_gain_data &max_gain_d)
{
    float test_freq_incr = 0.25f * M_2PI;  // set for 1/4 hz increments
    next_freq = test_data.freq;

    sweep_info data_at_ph161;
    float sugg_freq;
    if (freq_search_for_phase(test_data, 161.0f, test_freq_incr, data_at_ph161, sugg_freq)) {
        if ((data_at_ph161.gain < rd_prev_gain || is_zero(rd_prev_gain)) && tune_d < 0.6f * max_gain_d.max_allowed) {
            tune_d += 0.05f * max_gain_d.max_allowed;
            rd_prev_gain = data_at_ph161.gain;
            next_freq = data_at_ph161.freq;
        } else {
            success_counter = AUTOTUNE_SUCCESS_COUNT;
            // reset next freq and rd_prev_gain for next test
            next_freq = 0;
            rd_prev_gain = 0.0f;
            tune_d -= 0.05f * max_gain_d.max_allowed;
            tune_d = constrain_float(tune_d,0.0f,0.6f * max_gain_d.max_allowed);
        }
    } else {
        next_freq = sugg_freq;
    }
}

// updating_angle_p_up - determines maximum angle p gain for pitch and roll.  This is accomplished by determining the frequency
// for the maximum response gain that is the disturbance rejection peak.
void AC_AutoTune_Heli::updating_angle_p_up(float &tune_p, sweep_info &test_data, float &next_freq)
{
    float test_freq_incr = 0.5f * M_2PI;
    float gain_incr = 0.5f;

    if (is_zero(test_data.phase)) {
        // bad test point. increase slightly in hope of getting better result
        next_freq += 0.5f * test_freq_incr;
        return;
    }

    if (!found_max_gain_freq) {
        if (test_data.gain > max_resp_gain && tune_p > AUTOTUNE_SP_MIN) {
            // exceeded max response gain already, reduce tuning gain to remain under max response gain
            tune_p -= gain_incr;
            // force counter to stay on frequency
            next_freq = test_data.freq;
            return;
        } else if (test_data.gain > max_resp_gain && tune_p <= AUTOTUNE_SP_MIN) {
            // exceeded max response gain at the minimum allowable tuning gain. terminate testing.
            tune_p = AUTOTUNE_SP_MIN;
            success_counter = AUTOTUNE_SUCCESS_COUNT;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
        } else if (test_data.gain > sp_prev_gain) {
            freq_max = test_data.freq;
            phase_max = test_data.phase;
            sp_prev_gain = test_data.gain;
            next_freq = test_data.freq + test_freq_incr;
            return;
        // Gain is expected to continue decreasing past gain peak. declare max gain freq found and refine search.
        } else if (test_data.gain < 0.95f * sp_prev_gain) {
            found_max_gain_freq = true;
            next_freq = freq_max + 0.5 * test_freq_incr;
            return;
        } else {
            next_freq = test_data.freq + test_freq_incr;
            return;
        }
    }

    // refine peak 
    if (!found_peak) {
        // look at frequency above max gain freq found
        if (test_data.freq > freq_max && test_data.gain > sp_prev_gain) {
            // found max at frequency greater than initial max gain frequency
            found_peak = true;
        } else if (test_data.freq > freq_max && test_data.gain < sp_prev_gain) {
            // look at frequency below initial max gain frequency
            next_freq = test_data.freq - 0.5 * test_freq_incr;
            return;
        } else if (test_data.freq < freq_max && test_data.gain > sp_prev_gain) {
            // found max at frequency less than initial max gain frequency
            found_peak = true;
        } else {
            found_peak = true;
            test_data.freq = freq_max;
            test_data.gain = sp_prev_gain;
        }
        sp_prev_gain = test_data.gain;
    }

    // start increasing gain
    if (found_max_gain_freq && found_peak) {
        if (test_data.gain < max_resp_gain && tune_p < AUTOTUNE_SP_MAX) {
            tune_p += gain_incr;
            next_freq = test_data.freq;
            if (tune_p >= AUTOTUNE_SP_MAX) {
                tune_p = AUTOTUNE_SP_MAX;
                success_counter = AUTOTUNE_SUCCESS_COUNT;
                LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
            }
            sp_prev_gain = test_data.gain;
        } else if (test_data.gain > 1.1f * max_resp_gain && tune_p > AUTOTUNE_SP_MIN) {
            tune_p -= gain_incr;
        } else {
            // adjust tuning gain so max response gain is not exceeded
            if (sp_prev_gain < max_resp_gain && test_data.gain > max_resp_gain) {
                float adj_factor = (max_resp_gain - test_data.gain) / (test_data.gain - sp_prev_gain);
                tune_p = tune_p + gain_incr * adj_factor; 
            }
            success_counter = AUTOTUNE_SUCCESS_COUNT;
        }
    }
    if (success_counter == AUTOTUNE_SUCCESS_COUNT) {
        next_freq = 0.0f;  //initializes next test that uses dwell test
        sweep_complete = false;
        reset_sweep_variables();
    }
}

// updating_max_gains: use dwells at increasing frequency to determine gain at which instability will occur.  This uses the frequency
// response of motor class input to rate response to determine the max allowable gain for rate P gain.  A phase of 161 deg is used to
// determine analytically the max gain to 6db gain margin for a unity feedback controller. Since acceleration can be more noisy, the
// response of the motor class input to rate response to determine the max allowable gain for rate D gain.  A phase of 251 deg is used
// to determine analytically the max gain to 6db gain margin for a unity feedback controller.
void AC_AutoTune_Heli::updating_max_gains(sweep_info &test_data, float &next_freq, max_gain_data &max_gain_p, max_gain_data &max_gain_d, float &tune_p, float &tune_d)
{
    float test_freq_incr = 0.5f * M_2PI;
    next_freq = test_data.freq;

    sweep_info data_at_phase;
    float sugg_freq;
    if (!found_max_p) {
        if (freq_search_for_phase(test_data, 161.0f, test_freq_incr, data_at_phase, sugg_freq)) {
            max_gain_p.freq = data_at_phase.freq;
            max_gain_p.gain = data_at_phase.gain;
            max_gain_p.phase = data_at_phase.phase;
            max_gain_p.max_allowed = powf(10.0f,-1 * (log10f(max_gain_p.gain) * 20.0f + 2.42) / 20.0f);
            // limit max gain allowed to be no more than 2x the max p gain limit to keep initial gains bounded
            max_gain_p.max_allowed = constrain_float(max_gain_p.max_allowed, 0.0f, 2.0f * AUTOTUNE_RP_MAX);
            found_max_p = true;
            if (!is_zero(sweep_mtr.ph270.freq)) {
                next_freq = sweep_mtr.ph270.freq;
            } else {
                next_freq = data_at_phase.freq;
            }
        } else {
            next_freq = sugg_freq;
        }
    } else if (!found_max_d) {
        if (freq_search_for_phase(test_data, 251.0f, test_freq_incr, data_at_phase, sugg_freq)) {
            max_gain_d.freq = data_at_phase.freq;
            max_gain_d.gain = data_at_phase.gain;
            max_gain_d.phase = data_at_phase.phase;
            max_gain_d.max_allowed = powf(10.0f,-1 * (log10f(max_gain_d.freq * max_gain_d.gain) * 20.0f + 2.42) / 20.0f);
            // limit max gain allowed to be no more than 2x the max d gain limit to keep initial gains bounded
            max_gain_d.max_allowed = constrain_float(max_gain_d.max_allowed, 0.0f, 2.0f * AUTOTUNE_RD_MAX);
            found_max_d = true;
        } else {
            next_freq = sugg_freq;
        }
    }

    if (found_max_p && found_max_d) {
        success_counter = AUTOTUNE_SUCCESS_COUNT;
        // reset variables for next test
        next_freq = 0.0f;  //initializes next test that uses dwell test
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Max rate P freq=%f gain=%f", (double)(max_rate_p.freq), (double)(max_rate_p.gain));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: ph=%f rate_p=%f", (double)(max_rate_p.phase), (double)(max_rate_p.max_allowed));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: Max Rate D freq=%f gain=%f", (double)(max_rate_d.freq), (double)(max_rate_d.gain));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoTune: ph=%f rate_d=%f", (double)(max_rate_d.phase), (double)(max_rate_d.max_allowed));
    }

}

float AC_AutoTune_Heli::target_angle_max_rp_cd() const
{
    return AUTOTUNE_ANGLE_TARGET_MAX_RP_CD;
}

float AC_AutoTune_Heli::target_angle_max_y_cd() const
{
    return AUTOTUNE_ANGLE_TARGET_MAX_Y_CD;
}

float AC_AutoTune_Heli::target_angle_min_rp_cd() const
{
    return AUTOTUNE_ANGLE_TARGET_MIN_RP_CD;
}

float AC_AutoTune_Heli::target_angle_min_y_cd() const
{
    return AUTOTUNE_ANGLE_TARGET_MIN_Y_CD;
}

float AC_AutoTune_Heli::angle_lim_max_rp_cd() const
{
    return AUTOTUNE_ANGLE_MAX_RP_CD;
}

float AC_AutoTune_Heli::angle_lim_neg_rpy_cd() const
{
    return AUTOTUNE_ANGLE_NEG_RPY_CD;
}

// freq_search_for_phase: general search strategy for specified phase.  interpolation done once specified phase has been bounded.
bool AC_AutoTune_Heli::freq_search_for_phase(sweep_info test, float desired_phase, float freq_incr, sweep_info &est_data, float &new_freq)
{
    new_freq = test.freq;
    float phase_delta = 20.0f; // delta from desired phase below and above which full steps are taken
    if (is_zero(test.phase)) {
        // bad test point. increase slightly in hope of getting better result
        new_freq += 0.1f * freq_incr;
        return false;
    }

    // test to see if desired phase is bounded with a 0.5 freq_incr delta in freq
    float freq_delta = fabsf(prev_test.freq - test.freq);
    if (test.phase > desired_phase && prev_test.phase < desired_phase && freq_delta < 0.75f * freq_incr && is_positive(prev_test.freq)) {
        est_data.freq = linear_interpolate(prev_test.freq,test.freq,desired_phase,prev_test.phase,test.phase);
        est_data.gain = linear_interpolate(prev_test.gain,test.gain,desired_phase,prev_test.phase,test.phase);
        est_data.phase = desired_phase;
        prev_test = {};
        return true;
    } else if (test.phase < desired_phase && prev_test.phase > desired_phase && freq_delta < 0.75f * freq_incr && is_positive(prev_test.freq)) {
        est_data.freq = linear_interpolate(test.freq,prev_test.freq,desired_phase,test.phase,prev_test.phase);
        est_data.gain = linear_interpolate(test.gain,prev_test.gain,desired_phase,test.phase,prev_test.phase);
        est_data.phase = desired_phase;
        prev_test = {};
        return true;
    }

    if (test.phase < desired_phase - phase_delta) {
        new_freq += freq_incr;
    } else if (test.phase > desired_phase + phase_delta) {
        new_freq -= freq_incr;
    } else if (test.phase >= desired_phase - phase_delta && test.phase < desired_phase) {
        new_freq += 0.5f * freq_incr;
    } else if (test.phase <= desired_phase + phase_delta && test.phase >= desired_phase) {
        new_freq -= 0.5f * freq_incr;
    }
    prev_test = test;
    return false;
}

#if HAL_LOGGING_ENABLED
// log autotune summary data
void AC_AutoTune_Heli::Log_AutoTune()
{

    switch (axis) {
    case AxisType::ROLL:
        Log_Write_AutoTune(axis, tune_type, curr_data.freq, curr_data.gain, curr_data.phase, tune_roll_rff,  tune_roll_rp, tune_roll_rd, tune_roll_sp, test_accel_max_cdss);
        break;
    case AxisType::PITCH:
        Log_Write_AutoTune(axis, tune_type, curr_data.freq, curr_data.gain, curr_data.phase, tune_pitch_rff, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, test_accel_max_cdss);
        break;
    case AxisType::YAW:
    case AxisType::YAW_D:
        Log_Write_AutoTune(axis, tune_type, curr_data.freq, curr_data.gain, curr_data.phase, tune_yaw_rff, tune_yaw_rp, tune_yaw_rd, tune_yaw_sp, test_accel_max_cdss);
        break;
    }
}

// log autotune time history results for command, angular rate, and attitude
void AC_AutoTune_Heli::Log_AutoTuneDetails()
{
    if (tune_type == TuneType::ANGLE_P_UP || tune_type == TuneType::TUNE_CHECK) {
        Log_Write_AutoTuneDetails(command_out, 0.0f, 0.0f, filt_target_rate, rotation_rate);
    } else {
        Log_Write_AutoTuneDetails(command_out, filt_target_rate, rotation_rate, 0.0f, 0.0f);
    }
}

// log autotune frequency response data
void AC_AutoTune_Heli::Log_AutoTuneSweep()
{
    Log_Write_AutoTuneSweep(curr_test_mtr.freq, curr_test_mtr.gain, curr_test_mtr.phase,curr_test_tgt.freq, curr_test_tgt.gain, curr_test_tgt.phase);
}

// @LoggerMessage: ATNH
// @Description: Heli AutoTune
// @Vehicles: Copter
// @Field: TimeUS: Time since system startup
// @Field: Axis: which axis is currently being tuned
// @Field: TuneStep: step in autotune process
// @Field: Freq: target dwell frequency
// @Field: Gain: measured gain of dwell
// @Field: Phase: measured phase of dwell
// @Field: RFF: new rate gain FF term
// @Field: RP: new rate gain P term
// @Field: RD: new rate gain D term
// @Field: SP: new angle P term
// @Field: ACC: new max accel term

// Write an Autotune summary data packet
void AC_AutoTune_Heli::Log_Write_AutoTune(AxisType _axis, TuneType tune_step, float dwell_freq, float meas_gain, float meas_phase, float new_gain_rff, float new_gain_rp, float new_gain_rd, float new_gain_sp, float max_accel_radss)
{
    AP::logger().Write(
        "ATNH",
        "TimeUS,Axis,TuneStep,Freq,Gain,Phase,RFF,RP,RD,SP,ACC",
        "s--E-d----e",
        "F--000----0",
        "QBBffffffff",
        AP_HAL::micros64(),
        (uint8_t)axis,
        tune_step,
        dwell_freq,
        meas_gain,
        meas_phase,
        new_gain_rff,
        new_gain_rp,
        new_gain_rd,
        new_gain_sp,
        degrees(max_accel_radss));
}

// Write an Autotune detailed data packet
void AC_AutoTune_Heli::Log_Write_AutoTuneDetails(float motor_cmd, float tgt_rate_rads, float rate_rads, float tgt_ang_rad, float ang_rad)
{
    // @LoggerMessage: ATDH
    // @Description: Heli AutoTune data packet
    // @Vehicles: Copter
    // @Field: TimeUS: Time since system startup
    // @Field: Cmd: current motor command
    // @Field: TRate: current target angular rate
    // @Field: Rate: current angular rate
    // @Field: TAng: current target angle
    // @Field: Ang: current angle
    AP::logger().WriteStreaming(
        "ATDH",
        "TimeUS,Cmd,TRate,Rate,TAng,Ang",
        "s-kkdd",
        "F00000",
        "Qfffff",
        AP_HAL::micros64(),
        motor_cmd,
        tgt_rate_rads*57.3,
        rate_rads*57.3f,
        tgt_ang_rad*57.3,
        ang_rad*57.3f);
}

// Write an Autotune frequency response data packet
void AC_AutoTune_Heli::Log_Write_AutoTuneSweep(float freq_mtr, float gain_mtr, float phase_mtr, float freq_tgt, float gain_tgt, float phase_tgt)
{
    // @LoggerMessage: ATSH
    // @Description: Heli AutoTune Sweep packet
    // @Vehicles: Copter
    // @Field: TimeUS: Time since system startup
    // @Field: freq_m: current frequency for motor input to rate
    // @Field: gain_m: current response gain for motor input to rate
    // @Field: phase_m: current response phase for motor input to rate
    // @Field: freq_t: current frequency for target rate to rate
    // @Field: gain_t: current response gain for target rate to rate
    // @Field: phase_t: current response phase for target rate to rate
    AP::logger().WriteStreaming(
        "ATSH",
        "TimeUS,freq_m,gain_m,phase_m,freq_t,gain_t,phase_t",
        "sE-dE-d",
        "F000000",
        "Qffffff",
        AP_HAL::micros64(),
        freq_mtr,
        gain_mtr,
        phase_mtr,
        freq_tgt,
        gain_tgt,
        phase_tgt);
}
#endif  // HAL_LOGGING_ENABLED

// reset the test variables for each vehicle
void AC_AutoTune_Heli::reset_vehicle_test_variables()
{
    // reset dwell test variables if sweep was interrupted in order to restart sweep
    if (!is_equal(start_freq, stop_freq)) {
        start_freq = 0.0f;
        stop_freq = 0.0f;
        next_test_freq = 0.0f;
        sweep_complete = false;
    }
}

// reset the update gain variables for heli
void AC_AutoTune_Heli::reset_update_gain_variables()
{
   // reset max gain variables
    reset_maxgains_update_gain_variables();

    // reset rd_up variables
    rd_prev_gain = 0.0f;

    // reset sp_up variables
    phase_max = 0.0f;
    freq_max = 0.0f;
    sp_prev_gain = 0.0f;
    found_max_gain_freq = false;
    found_peak = false;

}

// reset the max_gains update gain variables
void AC_AutoTune_Heli::reset_maxgains_update_gain_variables()
{
    max_rate_p = {};
    max_rate_d = {};

    found_max_p = false;
    found_max_d = false;

}

// reset the max_gains update gain variables
void AC_AutoTune_Heli::reset_sweep_variables()
{
    sweep_tgt.ph180 = {};
    sweep_tgt.ph270 = {};
    sweep_tgt.maxgain = {};
    sweep_tgt.progress = 0;

    sweep_mtr.ph180 = {};
    sweep_mtr.ph270 = {};
    sweep_mtr.maxgain = {};

    sweep_mtr.progress = 0;

}

// set the tuning test sequence
void AC_AutoTune_Heli::set_tune_sequence()
{
    uint8_t seq_cnt = 0;

    if (seq_bitmask & AUTOTUNE_SEQ_BITMASK_VFF) {
        tune_seq[seq_cnt] = TuneType::RATE_FF_UP;
        seq_cnt++;
    }
    if (seq_bitmask & AUTOTUNE_SEQ_BITMASK_RATE_D) {
        tune_seq[seq_cnt] = TuneType::MAX_GAINS;
        seq_cnt++;
        tune_seq[seq_cnt] = TuneType::RATE_D_UP;
        seq_cnt++;
        tune_seq[seq_cnt] = TuneType::RATE_P_UP;
        seq_cnt++;
    }
    if (seq_bitmask & AUTOTUNE_SEQ_BITMASK_ANGLE_P) {
        tune_seq[seq_cnt] = TuneType::ANGLE_P_UP;
        seq_cnt++;
    }
    if (seq_bitmask & AUTOTUNE_SEQ_BITMASK_MAX_GAIN && !(seq_bitmask & AUTOTUNE_SEQ_BITMASK_RATE_D)) {
        tune_seq[seq_cnt] = TuneType::MAX_GAINS;
        seq_cnt++;
    }
    if (seq_bitmask & AUTOTUNE_SEQ_BITMASK_TUNE_CHECK) {
        tune_seq[seq_cnt] = TuneType::TUNE_CHECK;
        seq_cnt++;
    }
    tune_seq[seq_cnt] = TuneType::TUNE_COMPLETE;

}

// get_testing_step_timeout_ms accessor
uint32_t AC_AutoTune_Heli::get_testing_step_timeout_ms() const
{
    return AUTOTUNE_TESTING_STEP_TIMEOUT_MS;
}

    // exceeded_freq_range - ensures tuning remains inside frequency range
bool AC_AutoTune_Heli::exceeded_freq_range(float frequency)
{
    bool ret = false;
    if (frequency < min_sweep_freq || frequency > max_sweep_freq) {
        ret = true;
    }
    return ret;
}

#endif  // AC_AUTOTUNE_ENABLED
