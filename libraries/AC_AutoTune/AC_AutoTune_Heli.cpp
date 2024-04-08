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
    AP_GROUPINFO("ACC_MAX", 7, AC_AutoTune_Heli, accel_max,  5000.0f),

    // @Param: RAT_MAX
    // @DisplayName: Autotune maximum allowable angular rate
    // @Description: maximum angular rate in deg/s allowed during autotune maneuvers
    // @Range: 0 500
    // @User: Standard
    AP_GROUPINFO("RAT_MAX", 8, AC_AutoTune_Heli, rate_max,  50.0f),

    AP_GROUPEND
};

// constructor
AC_AutoTune_Heli::AC_AutoTune_Heli()
{
    tune_seq[0] = TUNE_COMPLETE;
    AP_Param::setup_object_defaults(this, var_info);
}

// initialize tests for each tune type
void AC_AutoTune_Heli::test_init()
{

    AC_AutoTune_FreqResp::InputType input_type = AC_AutoTune_FreqResp::InputType::DWELL;
    AC_AutoTune_FreqResp::ResponseType resp_type = AC_AutoTune_FreqResp::ResponseType::RATE;
    uint8_t num_dwell_cycles = 6;
    DwellType dwell_test_type = RATE;
    switch (tune_type) {
    case RFF_UP:
        freq_cnt = 12;
        test_freq[freq_cnt] = 0.25f * 3.14159f * 2.0f;
        curr_test.freq = test_freq[freq_cnt];
        start_freq = curr_test.freq;
        stop_freq = curr_test.freq;

        attitude_control->bf_feedforward(false);
        attitude_control->use_sqrt_controller(false);

        // variables needed to initialize frequency response object and dwell test method
        resp_type = AC_AutoTune_FreqResp::ResponseType::RATE;
        pre_calc_cycles = 1.0f;
        num_dwell_cycles = 3;
        dwell_test_type = RATE;

        break;
    case MAX_GAINS:
        if (is_zero(start_freq)) {
            if (!is_zero(sweep_mtr.ph180.freq)) {
                freq_cnt = 12;
                test_freq[freq_cnt] = sweep_mtr.ph180.freq - 0.25f * 3.14159f * 2.0f;
                curr_test.freq = test_freq[freq_cnt];
                start_freq = curr_test.freq;
                stop_freq = curr_test.freq;
                if (tune_type == MAX_GAINS) {
                    reset_maxgains_update_gain_variables();
                }
            } else {
                start_freq = min_sweep_freq;
                stop_freq = max_sweep_freq;
            }
        }
        attitude_control->bf_feedforward(false);
        attitude_control->use_sqrt_controller(false);

        // variables needed to initialize frequency response object and dwell test method
        resp_type = AC_AutoTune_FreqResp::ResponseType::RATE;
        dwell_test_type = RATE;
        pre_calc_cycles = 6.25f;
        num_dwell_cycles = 6;

        break;
    case RP_UP:
    case RD_UP:
        // initialize start frequency
        if (is_zero(start_freq)) {
            // continue using frequency where testing left off or RD_UP completed
            if (test_phase[12] > 0.0f && test_phase[12] < 180.0f) {
                freq_cnt = 12;
            // start with freq found for sweep where phase was 180 deg
            } else if (!is_zero(sweep_tgt.ph180.freq)) {
                freq_cnt = 12;
                test_freq[freq_cnt] = sweep_tgt.ph180.freq - 0.25f * 3.14159f * 2.0f;
            // otherwise start at min freq to step up in dwell frequency until phase > 160 deg
            } else {
                freq_cnt = 0;
                test_freq[freq_cnt] = min_sweep_freq;
            }
            curr_test.freq = test_freq[freq_cnt];
            start_freq = curr_test.freq;
            stop_freq = curr_test.freq;
        }
        attitude_control->bf_feedforward(false);
        attitude_control->use_sqrt_controller(false);

        // variables needed to initialize frequency response object and dwell test method
        resp_type = AC_AutoTune_FreqResp::ResponseType::RATE;
        dwell_test_type = RATE;
        pre_calc_cycles = 6.25f;
        num_dwell_cycles = 6;

        break;
    case SP_UP:
        // initialize start frequency
        if (is_zero(start_freq)) {
            if (!is_zero(sweep_tgt.maxgain.freq)) {
                freq_cnt = 12;
                test_freq[freq_cnt] = sweep_tgt.maxgain.freq - 0.25f * 3.14159f * 2.0f;
                curr_test.freq = test_freq[freq_cnt];
                start_freq = curr_test.freq;
                stop_freq = curr_test.freq;
                test_accel_max = 0.0f;
            } else {
                start_freq = min_sweep_freq;
                stop_freq = max_sweep_freq;
            }
        }
        attitude_control->bf_feedforward(false);
        attitude_control->use_sqrt_controller(false);

        // variables needed to initialize frequency response object and dwell test method
        resp_type = AC_AutoTune_FreqResp::ResponseType::ANGLE;
        dwell_test_type = DRB;
        pre_calc_cycles = 6.25f;
        num_dwell_cycles = 6;

        break;
    case TUNE_CHECK:
        // initialize start frequency
        if (is_zero(start_freq)) {
            start_freq = min_sweep_freq;
            stop_freq = max_sweep_freq;
        }
        // variables needed to initialize frequency response object and dwell test method
        resp_type = AC_AutoTune_FreqResp::ResponseType::ANGLE;
        dwell_test_type = ANGLE;

        break;
    default:
        break;
    }

    // initialize frequency response object
    if (!is_equal(start_freq,stop_freq)) {
        input_type = AC_AutoTune_FreqResp::InputType::SWEEP;
        step_time_limit_ms = sweep_time_ms + 500;
    } else {
        input_type = AC_AutoTune_FreqResp::InputType::DWELL;
        freqresp_tgt.set_dwell_cycles(num_dwell_cycles);
        freqresp_mtr.set_dwell_cycles(num_dwell_cycles);
        if (!is_zero(start_freq)) {
            // time limit set by adding the pre calc cycles with the dwell cycles.  500 ms added to account for settling with buffer.
            step_time_limit_ms = (uint32_t) (500 + ((float) freqresp_tgt.get_dwell_cycles() + pre_calc_cycles + 2.0f) * 1000.0f * M_2PI / start_freq);
        }
    }
    freqresp_tgt.init(input_type, resp_type);
    freqresp_mtr.init(input_type, resp_type);

    // initialize dwell test method
    dwell_test_init(start_freq, stop_freq, start_freq, dwell_test_type);

    start_angles = Vector3f(roll_cd, pitch_cd, desired_yaw_cd);  // heli specific
}

// run tests for each tune type
void AC_AutoTune_Heli::test_run(AxisType test_axis, const float dir_sign)
{
    // if tune complete or beyond frequency range or no max allowed gains then exit testing
    if (tune_type == TUNE_COMPLETE ||
       ((tune_type == RP_UP || tune_type == RD_UP) && (max_rate_p.max_allowed <= 0.0f || max_rate_d.max_allowed <= 0.0f)) ||
       ((tune_type == MAX_GAINS || tune_type == RP_UP || tune_type == RD_UP || tune_type == SP_UP) && exceeded_freq_range(start_freq))){

        load_gains(GAIN_ORIGINAL);

        attitude_control->use_sqrt_controller(true);

        get_poshold_attitude(roll_cd, pitch_cd, desired_yaw_cd);

        // hold level attitude
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_cd, pitch_cd, desired_yaw_cd, true);

        if ((tune_type == RP_UP || tune_type == RD_UP) && (max_rate_p.max_allowed <= 0.0f || max_rate_d.max_allowed <= 0.0f)) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Max Gain Determination Failed");
            mode = FAILED;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_FAILED);
            update_gcs(AUTOTUNE_MESSAGE_FAILED);
        } else if ((tune_type == MAX_GAINS || tune_type == RP_UP || tune_type == RD_UP || tune_type == SP_UP) && exceeded_freq_range(start_freq)){
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Exceeded frequency range");
            mode = FAILED;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_FAILED);
            update_gcs(AUTOTUNE_MESSAGE_FAILED);
        } else if (tune_type == TUNE_COMPLETE) {
            counter = AUTOTUNE_SUCCESS_COUNT;
            step = UPDATE_GAINS;
        }
        return;
    }

    switch (tune_type) {
    case RFF_UP:
        dwell_test_run(1, start_freq, stop_freq, test_gain[freq_cnt], test_phase[freq_cnt], RATE);
        break;
    case RP_UP:
    case RD_UP:
        dwell_test_run(1, start_freq, stop_freq, test_gain[freq_cnt], test_phase[freq_cnt], RATE);
        break;
    case MAX_GAINS:
        dwell_test_run(0, start_freq, stop_freq, test_gain[freq_cnt], test_phase[freq_cnt], RATE);
        break;
    case SP_UP:
        dwell_test_run(1, start_freq, stop_freq, test_gain[freq_cnt], test_phase[freq_cnt], DRB);
        break;
    case TUNE_CHECK:
        dwell_test_run(1, start_freq, stop_freq, test_gain[freq_cnt], test_phase[freq_cnt], ANGLE);
        break;
    default:
        step = UPDATE_GAINS;
        break;
    }
}

// heli specific gcs announcements
void AC_AutoTune_Heli::do_gcs_announcements()
{
    const uint32_t now = AP_HAL::millis();
    if (now - announce_time < AUTOTUNE_ANNOUNCE_INTERVAL_MS) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: %s %s", axis_string(), type_string());
    send_step_string();
    switch (tune_type) {
    case RFF_UP:
    case RD_UP:
    case RP_UP:
    case MAX_GAINS:
    case SP_UP:
    case TUNE_CHECK:
        if (is_equal(start_freq,stop_freq)) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Dwell");
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Sweep");
            if (settle_time == 0) {
                gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f phase=%f", (double)(curr_test.freq), (double)(curr_test.gain), (double)(curr_test.phase));
            }
        }
        break;
    default:
        break;
    }

    announce_time = now;
}

// send post test updates to user
void AC_AutoTune_Heli::do_post_test_gcs_announcements() {
    float tune_rp = 0.0f;
    float tune_rd = 0.0f;
    float tune_rff = 0.0f;
    float tune_sp = 0.0f;
    float tune_accel = 0.0f;

    switch (axis) {
    case ROLL:
        tune_rp = tune_roll_rp;
        tune_rd = tune_roll_rd;
        tune_rff = tune_roll_rff;
        tune_sp = tune_roll_sp;
        tune_accel = tune_roll_accel;
        break;
    case PITCH:
        tune_rp = tune_pitch_rp;
        tune_rd = tune_pitch_rd;
        tune_rff = tune_pitch_rff;
        tune_sp = tune_pitch_sp;
        tune_accel = tune_pitch_accel;
        break;
    case YAW:
    case YAW_D:
        tune_rp = tune_yaw_rp;
        tune_rd = tune_yaw_rd;
        tune_rff = tune_yaw_rff;
        tune_sp = tune_yaw_sp;
        tune_accel = tune_yaw_accel;
        break;
    }

    if (step == UPDATE_GAINS) {
        switch (tune_type) {
        case RFF_UP:
        case RP_UP:
        case RD_UP:
        case SP_UP:
        case MAX_GAINS:
            if (is_equal(start_freq,stop_freq)) {
                // announce results of dwell
                gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f", (double)(test_freq[freq_cnt]), (double)(test_gain[freq_cnt]));
                gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: ph=%f", (double)(test_phase[freq_cnt]));
               if (tune_type == RP_UP) {
                   gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: rate_p=%f", (double)(tune_rp));
               } else if (tune_type == RD_UP) {
                   gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: rate_d=%f", (double)(tune_rd));
               } else if (tune_type == RFF_UP) {
                   gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: rate_ff=%f", (double)(tune_rff));
               } else if (tune_type == SP_UP) {
                   gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: angle_p=%f tune_accel=%f max_accel=%f", (double)(tune_sp), (double)(tune_accel), (double)(test_accel_max));
               }
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: max_freq=%f max_gain=%f", (double)(sweep_tgt.maxgain.freq), (double)(sweep_tgt.maxgain.gain));
                gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: ph180_freq=%f ph180_gain=%f", (double)(sweep_tgt.ph180.freq), (double)(sweep_tgt.ph180.gain));
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
    freq_cnt = 0;
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

    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_INITIALISED);
}

// load_orig_gains - set gains to their original values
//  called by stop and failed functions
void AC_AutoTune_Heli::load_orig_gains()
{
    attitude_control->bf_feedforward(orig_bf_feedforward);
    if (roll_enabled()) {
        load_gain_set(ROLL, orig_roll_rp, orig_roll_ri, orig_roll_rd, orig_roll_rff, orig_roll_sp, orig_roll_accel, orig_roll_fltt, 0.0f, orig_roll_smax);
    }
    if (pitch_enabled()) {
        load_gain_set(PITCH, orig_pitch_rp, orig_pitch_ri, orig_pitch_rd, orig_pitch_rff, orig_pitch_sp, orig_pitch_accel, orig_pitch_fltt, 0.0f, orig_pitch_smax);
    }
    if (yaw_enabled()) {
        load_gain_set(YAW, orig_yaw_rp, orig_yaw_ri, orig_yaw_rd, orig_yaw_rff, orig_yaw_sp, orig_yaw_accel, orig_yaw_fltt, orig_yaw_rLPF, orig_yaw_smax);
    }
}

// load_tuned_gains - load tuned gains
void AC_AutoTune_Heli::load_tuned_gains()
{
    if (!attitude_control->get_bf_feedforward()) {
        attitude_control->bf_feedforward(true);
        attitude_control->set_accel_roll_max_cdss(0.0f);
        attitude_control->set_accel_pitch_max_cdss(0.0f);
    }
    if (roll_enabled()) {
        load_gain_set(ROLL, tune_roll_rp, tune_roll_rff*AUTOTUNE_FFI_RATIO_FINAL, tune_roll_rd, tune_roll_rff, tune_roll_sp, tune_roll_accel, orig_roll_fltt, 0.0f, orig_roll_smax);
    }
    if (pitch_enabled()) {
        load_gain_set(PITCH, tune_pitch_rp, tune_pitch_rff*AUTOTUNE_FFI_RATIO_FINAL, tune_pitch_rd, tune_pitch_rff, tune_pitch_sp, tune_pitch_accel, orig_pitch_fltt, 0.0f, orig_pitch_smax);
    }
    if (yaw_enabled()) {
        if (!is_zero(tune_yaw_rp)) {
            load_gain_set(YAW, tune_yaw_rp, tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL, tune_yaw_rd, tune_yaw_rff, tune_yaw_sp, tune_yaw_accel, orig_yaw_fltt, tune_yaw_rLPF, orig_yaw_smax);
        }
    }
}

// load_intra_test_gains - gains used between tests
//  called during testing mode's update-gains step to set gains ahead of return-to-level step
void AC_AutoTune_Heli::load_intra_test_gains()
{
    // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
    // sanity check the gains
    attitude_control->bf_feedforward(true);
    if (roll_enabled()) {
        load_gain_set(ROLL, orig_roll_rp, orig_roll_rff * AUTOTUNE_FFI_RATIO_FOR_TESTING, orig_roll_rd, orig_roll_rff, orig_roll_sp, orig_roll_accel, orig_roll_fltt, 0.0f, orig_roll_smax);
    }
    if (pitch_enabled()) {
        load_gain_set(PITCH, orig_pitch_rp, orig_pitch_rff * AUTOTUNE_FFI_RATIO_FOR_TESTING, orig_pitch_rd, orig_pitch_rff, orig_pitch_sp, orig_pitch_accel, orig_pitch_fltt, 0.0f, orig_pitch_smax);
    }
    if (yaw_enabled()) {
        load_gain_set(YAW, orig_yaw_rp, orig_yaw_rp*AUTOTUNE_PI_RATIO_FOR_TESTING, orig_yaw_rd, orig_yaw_rff, orig_yaw_sp, orig_yaw_accel, orig_yaw_fltt, orig_yaw_rLPF, orig_yaw_smax);
    }
}

// load_test_gains - load the to-be-tested gains for a single axis
// called by control_attitude() just before it beings testing a gain (i.e. just before it twitches)
void AC_AutoTune_Heli::load_test_gains()
{
    float rate_p, rate_i, rate_d;
    switch (axis) {
    case ROLL:
        if (tune_type == SP_UP || tune_type == TUNE_CHECK) {
            rate_i = tune_roll_rff*AUTOTUNE_FFI_RATIO_FINAL;
        } else {
            // freeze integrator to hold trim by making i term small during rate controller tuning
            rate_i = 0.01f * orig_roll_ri;
        }
        if (tune_type == MAX_GAINS && !is_zero(tune_roll_rff)) {
            rate_p = 0.0f;
            rate_d = 0.0f;
        } else {
            rate_p = tune_roll_rp;
            rate_d = tune_roll_rd;
        }
        load_gain_set(ROLL, rate_p, rate_i, rate_d, tune_roll_rff, tune_roll_sp, tune_roll_accel, orig_roll_fltt, 0.0f, 0.0f);
        break;
    case PITCH:
        if (tune_type == SP_UP || tune_type == TUNE_CHECK) {
            rate_i = tune_pitch_rff*AUTOTUNE_FFI_RATIO_FINAL;
        } else {
            // freeze integrator to hold trim by making i term small during rate controller tuning
            rate_i = 0.01f * orig_pitch_ri;
        }
        if (tune_type == MAX_GAINS && !is_zero(tune_pitch_rff)) {
            rate_p = 0.0f;
            rate_d = 0.0f;
        } else {
            rate_p = tune_pitch_rp;
            rate_d = tune_pitch_rd;
        }
        load_gain_set(PITCH, rate_p, rate_i, rate_d, tune_pitch_rff, tune_pitch_sp, tune_pitch_accel, orig_pitch_fltt, 0.0f, 0.0f);
        break;
    case YAW:
    case YAW_D:
        if (tune_type == SP_UP || tune_type == TUNE_CHECK) {
            rate_i = tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL;
        } else {
            // freeze integrator to hold trim by making i term small during rate controller tuning
            rate_i = 0.01f * orig_yaw_ri;
        }
        load_gain_set(YAW, tune_yaw_rp, rate_i, tune_yaw_rd, tune_yaw_rff, tune_yaw_sp, tune_yaw_accel, orig_yaw_fltt, tune_yaw_rLPF, 0.0f);
        break;
    }
}

// load gains
void AC_AutoTune_Heli::load_gain_set(AxisType s_axis, float rate_p, float rate_i, float rate_d, float rate_ff, float angle_p, float max_accel, float rate_fltt, float rate_flte, float smax)
{
    switch (s_axis) {
    case ROLL:
        attitude_control->get_rate_roll_pid().kP(rate_p);
        attitude_control->get_rate_roll_pid().kI(rate_i);
        attitude_control->get_rate_roll_pid().kD(rate_d);
        attitude_control->get_rate_roll_pid().ff(rate_ff);
        attitude_control->get_rate_roll_pid().filt_T_hz(rate_fltt);
        attitude_control->get_rate_roll_pid().slew_limit(smax);
        attitude_control->get_angle_roll_p().kP(angle_p);
        attitude_control->set_accel_roll_max_cdss(max_accel);
        break;
    case PITCH:
        attitude_control->get_rate_pitch_pid().kP(rate_p);
        attitude_control->get_rate_pitch_pid().kI(rate_i);
        attitude_control->get_rate_pitch_pid().kD(rate_d);
        attitude_control->get_rate_pitch_pid().ff(rate_ff);
        attitude_control->get_rate_pitch_pid().filt_T_hz(rate_fltt);
        attitude_control->get_rate_pitch_pid().slew_limit(smax);
        attitude_control->get_angle_pitch_p().kP(angle_p);
        attitude_control->set_accel_pitch_max_cdss(max_accel);
        break;
    case YAW:
    case YAW_D:
        attitude_control->get_rate_yaw_pid().kP(rate_p);
        attitude_control->get_rate_yaw_pid().kI(rate_i);
        attitude_control->get_rate_yaw_pid().kD(rate_d);
        attitude_control->get_rate_yaw_pid().ff(rate_ff);
        attitude_control->get_rate_yaw_pid().filt_T_hz(rate_fltt);
        attitude_control->get_rate_yaw_pid().slew_limit(smax);
        attitude_control->get_rate_yaw_pid().filt_E_hz(rate_flte);
        attitude_control->get_angle_yaw_p().kP(angle_p);
        attitude_control->set_accel_yaw_max_cdss(max_accel);
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
        attitude_control->save_accel_roll_max_cdss(0.0f);
        attitude_control->save_accel_pitch_max_cdss(0.0f);
    }

    // sanity check the rate P values
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL) && roll_enabled()) {
        load_gain_set(ROLL, tune_roll_rp, tune_roll_rff*AUTOTUNE_FFI_RATIO_FINAL, tune_roll_rd, tune_roll_rff, tune_roll_sp, tune_roll_accel, orig_roll_fltt, 0.0f, orig_roll_smax);
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
        orig_roll_accel = attitude_control->get_accel_roll_max_cdss();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) && pitch_enabled()) {
        load_gain_set(PITCH, tune_pitch_rp, tune_pitch_rff*AUTOTUNE_FFI_RATIO_FINAL, tune_pitch_rd, tune_pitch_rff, tune_pitch_sp, tune_pitch_accel, orig_pitch_fltt, 0.0f, orig_pitch_smax);
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
        orig_pitch_accel = attitude_control->get_accel_pitch_max_cdss();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_YAW) && yaw_enabled() && !is_zero(tune_yaw_rp)) {
        load_gain_set(YAW, tune_yaw_rp, tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL, tune_yaw_rd, tune_yaw_rff, tune_yaw_sp, tune_yaw_accel, orig_yaw_fltt, orig_yaw_rLPF, orig_yaw_smax);
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
        orig_yaw_accel = attitude_control->get_accel_yaw_max_cdss();
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
        case ROLL:
            report_axis_gains("Roll", tune_roll_rp, tune_roll_rff*AUTOTUNE_FFI_RATIO_FINAL, tune_roll_rd, tune_roll_rff, tune_roll_sp, tune_roll_accel);
            break;
        case PITCH:
            report_axis_gains("Pitch", tune_pitch_rp, tune_pitch_rff*AUTOTUNE_FFI_RATIO_FINAL, tune_pitch_rd, tune_pitch_rff, tune_pitch_sp, tune_pitch_accel);
            break;
        case YAW:
        case YAW_D:
            report_axis_gains("Yaw", tune_yaw_rp, tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL, tune_yaw_rd, tune_yaw_rff, tune_yaw_sp, tune_yaw_accel);
            break;
    }
}

// report gain formatting helper
void AC_AutoTune_Heli::report_axis_gains(const char* axis_string, float rate_P, float rate_I, float rate_D, float rate_ff, float angle_P, float max_accel) const
{
    gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: %s complete", axis_string);
    gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: %s Rate: P:%0.4f, I:%0.4f, D:%0.5f, FF:%0.4f",axis_string,rate_P,rate_I,rate_D,rate_ff);
    gcs().send_text(MAV_SEVERITY_NOTICE,"AutoTune: %s Angle P:%0.2f, Max Accel:%0.0f",axis_string,angle_P,max_accel);
}

void AC_AutoTune_Heli::dwell_test_init(float start_frq, float stop_frq, float filt_freq, DwellType dwell_type)
{
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
    filt_command_reading.set_cutoff_frequency(0.2f * start_frq);
    filt_gyro_reading.set_cutoff_frequency(0.2f * start_frq);
    filt_tgt_rate_reading.set_cutoff_frequency(0.2f * start_frq);
    filt_att_fdbk_from_velxy_cd.set_cutoff_frequency(0.2f * start_frq);

    if (!is_equal(start_frq, stop_frq)) {
        reset_sweep_variables();
        curr_test.gain = 0.0f;
        curr_test.phase = 0.0f;
        chirp_input.init(0.001f * sweep_time_ms, start_frq / M_2PI, stop_frq / M_2PI, 0.0f, 0.0001f * sweep_time_ms, 0.0f);
    } else {
        chirp_input.init(0.001f * step_time_limit_ms, start_frq / M_2PI, stop_frq / M_2PI, 0.0f, 0.0001f * step_time_limit_ms, 0.0f);
    }

    cycle_complete_tgt = false;
    cycle_complete_mtr = false;


}

void AC_AutoTune_Heli::dwell_test_run(uint8_t freq_resp_input, float start_frq, float stop_frq, float &dwell_gain, float &dwell_phase, DwellType dwell_type)
{
    float gyro_reading = 0.0f;
    float command_reading = 0.0f;
    float tgt_rate_reading = 0.0f;
    float tgt_attitude;
    const uint32_t now = AP_HAL::millis();
    float target_angle_cd = 0.0f;
    float dwell_freq = start_frq;

    float cycle_time_ms = 0;
    if (!is_zero(dwell_freq)) {
        cycle_time_ms = 1000.0f * M_2PI / dwell_freq;
    }

    //Determine target attitude magnitude limiting acceleration and rate
    tgt_attitude = 5.0f * 0.01745f;

    // body frame calculation of velocity
    Vector3f velocity_ned, velocity_bf;
    if (ahrs_view->get_velocity_NED(velocity_ned)) {
        velocity_bf.x = velocity_ned.x * ahrs_view->cos_yaw() + velocity_ned.y * ahrs_view->sin_yaw();
        velocity_bf.y = -velocity_ned.x * ahrs_view->sin_yaw() + velocity_ned.y * ahrs_view->cos_yaw();
    }

    if (settle_time == 0) {
        target_angle_cd = -chirp_input.update((now - dwell_start_time_ms) * 0.001, tgt_attitude * 5730.0f);
        dwell_freq = chirp_input.get_frequency_rads();
        const Vector2f att_fdbk {
            -5730.0f * vel_hold_gain * velocity_bf.y,
            5730.0f * vel_hold_gain * velocity_bf.x
        };
        filt_att_fdbk_from_velxy_cd.apply(att_fdbk, AP::scheduler().get_loop_period_s());
    } else {
        target_angle_cd = 0.0f;
        trim_yaw_tgt_reading = (float)attitude_control->get_att_target_euler_cd().z;
        trim_yaw_heading_reading = (float)ahrs_view->yaw_sensor;
        dwell_start_time_ms = now;
        filt_att_fdbk_from_velxy_cd.reset(Vector2f(0.0f,0.0f));
        settle_time--;
    }

    const Vector2f trim_angle_cd {
        constrain_float(filt_att_fdbk_from_velxy_cd.get().x, -2000.0f, 2000.0f),
        constrain_float(filt_att_fdbk_from_velxy_cd.get().y, -2000.0f, 2000.0f)
    };

    switch (axis) {
    case ROLL:
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_angle_cd + trim_angle_cd.x, trim_angle_cd.y, 0.0f);
        command_reading = motors->get_roll();
        if (dwell_type == DRB) {
            tgt_rate_reading = (target_angle_cd) / 5730.0f;
            gyro_reading = ((float)ahrs_view->roll_sensor + trim_angle_cd.x - target_angle_cd) / 5730.0f;
        } else if (dwell_type == RATE) {
            tgt_rate_reading = attitude_control->rate_bf_targets().x;
            gyro_reading = ahrs_view->get_gyro().x;
        } else {
            tgt_rate_reading = ((float)attitude_control->get_att_target_euler_cd().x) / 5730.0f;
            gyro_reading = ((float)ahrs_view->roll_sensor) / 5730.0f;
        }
        break;
    case PITCH:
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(trim_angle_cd.x, target_angle_cd + trim_angle_cd.y, 0.0f);
        command_reading = motors->get_pitch();
        if (dwell_type == DRB) {
            tgt_rate_reading = (target_angle_cd) / 5730.0f;
            gyro_reading = ((float)ahrs_view->pitch_sensor + trim_angle_cd.y - target_angle_cd) / 5730.0f;
        } else if (dwell_type == RATE) {
            tgt_rate_reading = attitude_control->rate_bf_targets().y;
            gyro_reading = ahrs_view->get_gyro().y;
        } else {
            tgt_rate_reading = ((float)attitude_control->get_att_target_euler_cd().y) / 5730.0f;
            gyro_reading = ((float)ahrs_view->pitch_sensor) / 5730.0f;
        }
        break;
    case YAW:
    case YAW_D:
        attitude_control->input_euler_angle_roll_pitch_yaw(trim_angle_cd.x, trim_angle_cd.y, wrap_180_cd(trim_yaw_tgt_reading + target_angle_cd), false);
        command_reading = motors->get_yaw();
        if (dwell_type == DRB) {
            tgt_rate_reading = (target_angle_cd) / 5730.0f;
            gyro_reading = (wrap_180_cd((float)ahrs_view->yaw_sensor - trim_yaw_heading_reading - target_angle_cd)) / 5730.0f;
        } else if (dwell_type == RATE) {
            tgt_rate_reading = attitude_control->rate_bf_targets().z;
            gyro_reading = ahrs_view->get_gyro().z;
        } else {
            tgt_rate_reading = (wrap_180_cd((float)attitude_control->get_att_target_euler_cd().z - trim_yaw_tgt_reading)) / 5730.0f;
            gyro_reading = (wrap_180_cd((float)ahrs_view->yaw_sensor - trim_yaw_heading_reading)) / 5730.0f;
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
    if ((float)(now - dwell_start_time_ms) > pre_calc_cycles * cycle_time_ms || (!is_equal(start_frq,stop_frq) && settle_time == 0)) {
        freqresp_mtr.update(command_out,command_out,rotation_rate, dwell_freq);
        freqresp_tgt.update(command_out,filt_target_rate,rotation_rate, dwell_freq);

        if (freqresp_mtr.is_cycle_complete()) {
            if (!is_equal(start_frq,stop_frq)) {
                curr_test_mtr.freq = freqresp_mtr.get_freq();
                curr_test_mtr.gain = freqresp_mtr.get_gain();
                curr_test_mtr.phase = freqresp_mtr.get_phase();
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
            if (!is_equal(start_frq,stop_frq)) {
                curr_test_tgt.freq = freqresp_tgt.get_freq();
                curr_test_tgt.gain = freqresp_tgt.get_gain();
                curr_test_tgt.phase = freqresp_tgt.get_phase();
                if (dwell_type == DRB) {test_accel_max = freqresp_tgt.get_accel_max();}
                // reset cycle_complete to allow indication of next cycle
                freqresp_tgt.reset_cycle_complete();
#if HAL_LOGGING_ENABLED
                // log sweep data
                Log_AutoTuneSweep();
#endif
            } else {
                dwell_gain_tgt = freqresp_tgt.get_gain();
                dwell_phase_tgt = freqresp_tgt.get_phase();
                if (dwell_type == DRB) {test_accel_max = freqresp_tgt.get_accel_max();}
                cycle_complete_tgt = true;
            }
        }

        if (freq_resp_input == 1) {
            if (!is_equal(start_frq,stop_frq)) {
                curr_test = curr_test_tgt;
            } else {
                dwell_gain = dwell_gain_tgt;
                dwell_phase = dwell_phase_tgt;
            }
        } else {
            if (!is_equal(start_frq,stop_frq)) {
                curr_test = curr_test_mtr;
            } else {
                dwell_gain = dwell_gain_mtr;
                dwell_phase = dwell_phase_mtr;
            }
        }
    }

    // set sweep data if a frequency sweep is being conducted
    if (!is_equal(start_frq,stop_frq) && (float)(now - dwell_start_time_ms) > 2.5f * cycle_time_ms) {
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

        if (now - step_start_time_ms >= sweep_time_ms + 200) {
            // we have passed the maximum stop time
            step = UPDATE_GAINS;
        }
    } else {
        if (now - step_start_time_ms >= step_time_limit_ms || (freqresp_tgt.is_cycle_complete() && freqresp_mtr.is_cycle_complete())) {
            if (now - step_start_time_ms >= step_time_limit_ms) {
                    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Step time limit exceeded");
            }
            cycle_complete_tgt = false;
            cycle_complete_tgt = false;
            // we have passed the maximum stop time
            step = UPDATE_GAINS;
        }
    }
}

// update gains for the rate p up tune type
void AC_AutoTune_Heli::updating_rate_p_up_all(AxisType test_axis)
{
    switch (test_axis) {
    case ROLL:
        updating_rate_p_up(tune_roll_rp, test_freq, test_gain, test_phase, freq_cnt, max_rate_p);
        break;
    case PITCH:
        updating_rate_p_up(tune_pitch_rp, test_freq, test_gain, test_phase, freq_cnt, max_rate_p);
        break;
    case YAW:
    case YAW_D:
        updating_rate_p_up(tune_yaw_rp, test_freq, test_gain, test_phase, freq_cnt, max_rate_p);
        break;
    }
}

// update gains for the rate d up tune type
void AC_AutoTune_Heli::updating_rate_d_up_all(AxisType test_axis)
{
    switch (test_axis) {
    case ROLL:
        updating_rate_d_up(tune_roll_rd, test_freq, test_gain, test_phase, freq_cnt, max_rate_d);
        break;
    case PITCH:
        updating_rate_d_up(tune_pitch_rd, test_freq, test_gain, test_phase, freq_cnt, max_rate_d);
        break;
    case YAW:
    case YAW_D:
        updating_rate_d_up(tune_yaw_rd, test_freq, test_gain, test_phase, freq_cnt, max_rate_d);
        break;
    }
}

// update gains for the rate ff up tune type
void AC_AutoTune_Heli::updating_rate_ff_up_all(AxisType test_axis)
{
    switch (test_axis) {
    case ROLL:
        updating_rate_ff_up(tune_roll_rff, test_freq, test_gain, test_phase, freq_cnt);
        break;
    case PITCH:
        updating_rate_ff_up(tune_pitch_rff, test_freq, test_gain, test_phase, freq_cnt);
        break;
    case YAW:
    case YAW_D:
        updating_rate_ff_up(tune_yaw_rff, test_freq, test_gain, test_phase, freq_cnt);
        // TODO make FF updating routine determine when to set rff gain to zero based on A/C response
        if (tune_yaw_rff <= AUTOTUNE_RFF_MIN && counter == AUTOTUNE_SUCCESS_COUNT) {
            tune_yaw_rff = 0.0f;
        }
        break;
    }
}

// update gains for the angle p up tune type
void AC_AutoTune_Heli::updating_angle_p_up_all(AxisType test_axis)
{
    attitude_control->bf_feedforward(orig_bf_feedforward);

    switch (test_axis) {
    case ROLL:
        updating_angle_p_up(tune_roll_sp, test_freq, test_gain, test_phase, freq_cnt);
        break;
    case PITCH:
        updating_angle_p_up(tune_pitch_sp, test_freq, test_gain, test_phase, freq_cnt);
        break;
    case YAW:
    case YAW_D:
        updating_angle_p_up(tune_yaw_sp, test_freq, test_gain, test_phase, freq_cnt);
        break;
    }
}

// update gains for the max gain tune type
void AC_AutoTune_Heli::updating_max_gains_all(AxisType test_axis)
{
    switch (test_axis) {
    case ROLL:
        updating_max_gains(&test_freq[0], &test_gain[0], &test_phase[0], freq_cnt, max_rate_p, max_rate_d, tune_roll_rp, tune_roll_rd);
        break;
    case PITCH:
        updating_max_gains(&test_freq[0], &test_gain[0], &test_phase[0], freq_cnt, max_rate_p, max_rate_d, tune_pitch_rp, tune_pitch_rd);
        break;
    case YAW:
    case YAW_D:
        updating_max_gains(&test_freq[0], &test_gain[0], &test_phase[0], freq_cnt, max_rate_p, max_rate_d, tune_yaw_rp, tune_yaw_rd);
        // rate P and rate D can be non zero for yaw and need to be included in the max allowed gain
        if (!is_zero(max_rate_p.max_allowed) && counter == AUTOTUNE_SUCCESS_COUNT) {
            max_rate_p.max_allowed += tune_yaw_rp;
        }
        if (!is_zero(max_rate_d.max_allowed) && counter == AUTOTUNE_SUCCESS_COUNT) {
            max_rate_d.max_allowed += tune_yaw_rd;
        }
        break;
    }
}

// set gains post tune for the tune type
void AC_AutoTune_Heli::set_gains_post_tune(AxisType test_axis)
{
    switch (tune_type) {
    case RD_UP:
        switch (test_axis) {
        case ROLL:
            tune_roll_rd = MAX(0.0f, tune_roll_rd * AUTOTUNE_RD_BACKOFF);
            break;
        case PITCH:
            tune_pitch_rd = MAX(0.0f, tune_pitch_rd * AUTOTUNE_RD_BACKOFF);
            break;
        case YAW:
        case YAW_D:
            tune_yaw_rd = MAX(0.0f, tune_yaw_rd * AUTOTUNE_RD_BACKOFF);
            break;
        }
        break;
    case RP_UP:
        switch (test_axis) {
        case ROLL:
            tune_roll_rp = MAX(0.0f, tune_roll_rp * AUTOTUNE_RP_BACKOFF);
            break;
        case PITCH:
            tune_pitch_rp = MAX(0.0f, tune_pitch_rp * AUTOTUNE_RP_BACKOFF);
            break;
        case YAW:
        case YAW_D:
            tune_yaw_rp = MAX(AUTOTUNE_RP_MIN, tune_yaw_rp * AUTOTUNE_RP_BACKOFF);
            break;
        }
        break;
    case SP_UP:
        switch (test_axis) {
        case ROLL:
            tune_roll_sp = MAX(AUTOTUNE_SP_MIN, tune_roll_sp * AUTOTUNE_SP_BACKOFF);
            break;
        case PITCH:
            tune_pitch_sp = MAX(AUTOTUNE_SP_MIN, tune_pitch_sp * AUTOTUNE_SP_BACKOFF);
            break;
        case YAW:
        case YAW_D:
            tune_yaw_sp = MAX(AUTOTUNE_SP_MIN, tune_yaw_sp * AUTOTUNE_SP_BACKOFF);
            break;
        }
        break;
    case RFF_UP:
        break;
    default:
        break;
    }
}

// updating_rate_ff_up - adjust FF to ensure the target is reached
// FF is adjusted until rate requested is achieved
void AC_AutoTune_Heli::updating_rate_ff_up(float &tune_ff, float *freq, float *gain, float *phase, uint8_t &frq_cnt)
{
    float test_freq_incr = 0.05f * 3.14159f * 2.0f;

    if (phase[frq_cnt] > 15.0f) {
        curr_test.freq = curr_test.freq - test_freq_incr;
        freq[frq_cnt] = curr_test.freq;
    } else if (phase[frq_cnt] < 0.0f) {
        curr_test.freq = curr_test.freq + test_freq_incr;
        freq[frq_cnt] = curr_test.freq;
    } else {
        if ((gain[frq_cnt] > 0.1 && gain[frq_cnt] < 0.93) || gain[frq_cnt] > 0.98) {
            if (tune_ff > 0.0f) {
                tune_ff =  tune_ff / gain[frq_cnt];    
            } else {
                tune_ff = 0.03f;
            }
        } else if (gain[frq_cnt] >= 0.93 && gain[frq_cnt] <= 0.98) {
            counter = AUTOTUNE_SUCCESS_COUNT;
            // reset curr_test.freq and frq_cnt for next test
            curr_test.freq = freq[0];
            frq_cnt = 0;
            tune_ff = constrain_float(tune_ff,0.0f,1.0f);
        }
    }

    if (counter == AUTOTUNE_SUCCESS_COUNT) {
        start_freq = 0.0f;  //initializes next test that uses dwell test
    } else {
        start_freq = curr_test.freq;
        stop_freq = curr_test.freq;
    }
}

// updating_rate_p_up - uses maximum allowable gain determined from max_gain test to determine rate p gain that does not exceed exceed max response gain
void AC_AutoTune_Heli::updating_rate_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_p)
{
    float test_freq_incr = 0.25f * 3.14159f * 2.0f;

    if (frq_cnt < 12 && is_equal(start_freq,stop_freq)) {
        if (phase[frq_cnt] <= 180.0f && !is_zero(phase[frq_cnt])) {
            rp_prev_good_frq_cnt = frq_cnt;
        } else if (frq_cnt > 1 && phase[frq_cnt] > phase[frq_cnt-1] + 360.0f && !is_zero(phase[frq_cnt])) {
            if (phase[frq_cnt] - 360.0f < 180.0f) {
                rp_prev_good_frq_cnt = frq_cnt;
            }
        } else if (frq_cnt > 1 && phase[frq_cnt] > 200.0f && !is_zero(phase[frq_cnt])) {
            frq_cnt = 11;
        }
        frq_cnt++;
        if (frq_cnt == 12) {
            freq[frq_cnt] = freq[rp_prev_good_frq_cnt];
            curr_test.freq = freq[frq_cnt];
        } else {
            freq[frq_cnt] = freq[frq_cnt-1] + test_freq_incr;
            curr_test.freq = freq[frq_cnt];
        }
    } else if (is_equal(start_freq,stop_freq)) {
        if (phase[frq_cnt] > 180.0f) {
            curr_test.freq = curr_test.freq - 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test.freq;
        } else if (phase[frq_cnt] < 160.0f) {
            curr_test.freq = curr_test.freq + 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test.freq;
        } else if (phase[frq_cnt] <= 180.0f && phase[frq_cnt] >= 160.0f) {
            if (gain[frq_cnt] < max_resp_gain && tune_p < 0.6f * max_gain_p.max_allowed) {
                tune_p += 0.05f * max_gain_p.max_allowed;
            } else {
                counter = AUTOTUNE_SUCCESS_COUNT;
                // reset curr_test.freq and frq_cnt for next test
                curr_test.freq = freq[0];
                frq_cnt = 0;
                tune_p -= 0.05f * max_gain_p.max_allowed;
                tune_p = constrain_float(tune_p,0.0f,0.6f * max_gain_p.max_allowed);
            }
        }
    }

    if (counter == AUTOTUNE_SUCCESS_COUNT) {
        start_freq = 0.0f;  //initializes next test that uses dwell test
    } else {
        start_freq = curr_test.freq;
        stop_freq = curr_test.freq;
    }
}

// updating_rate_d_up - uses maximum allowable gain determined from max_gain test to determine rate d gain where the response gain is at a minimum
void AC_AutoTune_Heli::updating_rate_d_up(float &tune_d, float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_d)
{
    float test_freq_incr = 0.25f * 3.14159f * 2.0f;  // set for 1/4 hz increments

    // if sweep failed to find frequency for 180 phase then use dwell to find frequency
    if (frq_cnt < 12 && is_equal(start_freq,stop_freq)) {
        if (phase[frq_cnt] <= 180.0f && !is_zero(phase[frq_cnt])) {
            rd_prev_good_frq_cnt = frq_cnt;
        } else if (frq_cnt > 1 && phase[frq_cnt] > phase[frq_cnt-1] + 360.0f && !is_zero(phase[frq_cnt])) {
            if (phase[frq_cnt] - 360.0f < 180.0f) {
                rd_prev_good_frq_cnt = frq_cnt;
            }
        } else if (frq_cnt > 1 && phase[frq_cnt] > 200.0f && !is_zero(phase[frq_cnt])) {
            frq_cnt = 11;
        }
        frq_cnt++;
        if (frq_cnt == 12) {
            freq[frq_cnt] = freq[rd_prev_good_frq_cnt];
            curr_test.freq = freq[frq_cnt];
        } else {
            freq[frq_cnt] = freq[frq_cnt-1] + test_freq_incr;
            curr_test.freq = freq[frq_cnt];
        }
    } else if (is_equal(start_freq,stop_freq)) {
        if (phase[frq_cnt] > 180.0f) {
            curr_test.freq = curr_test.freq - 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test.freq;
        } else if (phase[frq_cnt] < 160.0f) {
            curr_test.freq = curr_test.freq + 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test.freq;
        } else if (phase[frq_cnt] <= 180.0f && phase[frq_cnt] >= 160.0f) {
            if ((gain[frq_cnt] < rd_prev_gain || is_zero(rd_prev_gain)) && tune_d < 0.6f * max_gain_d.max_allowed) {
                tune_d += 0.05f * max_gain_d.max_allowed;
                rd_prev_gain = gain[frq_cnt];
            } else {
                counter = AUTOTUNE_SUCCESS_COUNT;
                // reset curr_test.freq and frq_cnt for next test
                curr_test.freq = freq[0];
                frq_cnt = 0;
                rd_prev_gain = 0.0f;
                tune_d -= 0.05f * max_gain_d.max_allowed;
                tune_d = constrain_float(tune_d,0.0f,0.6f * max_gain_d.max_allowed);
            }
        }
    }
    if (counter == AUTOTUNE_SUCCESS_COUNT) {
        start_freq = 0.0f;  //initializes next test that uses dwell test
    } else {
        start_freq = curr_test.freq;
        stop_freq = curr_test.freq;
    }
}

// updating_angle_p_up - determines maximum angle p gain for pitch and roll
void AC_AutoTune_Heli::updating_angle_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt)
{
    float test_freq_incr = 0.5f * 3.14159f * 2.0f;
    float gain_incr = 0.5f;

    if (!is_equal(start_freq,stop_freq)) {
        if (!is_zero(sweep_tgt.maxgain.freq)) {
            frq_cnt = 12;
            freq[frq_cnt] = sweep_tgt.maxgain.freq - 0.5f * test_freq_incr;
            freq_cnt_max = frq_cnt;
        } else {
            frq_cnt = 1;
            freq[frq_cnt] = min_sweep_freq;
            freq_cnt_max = 0;
        }
        curr_test.freq = freq[frq_cnt];
    }
    if (frq_cnt < 12 && is_equal(start_freq,stop_freq)) {
        if (gain[frq_cnt] > max_resp_gain && tune_p > AUTOTUNE_SP_MIN) {
            // exceeded max response gain already, reduce tuning gain to remain under max response gain
            tune_p -= gain_incr;
            // force counter to stay on frequency
            frq_cnt -= 1;
        } else if (gain[frq_cnt] > max_resp_gain && tune_p <= AUTOTUNE_SP_MIN) {
            // exceeded max response gain at the minimum allowable tuning gain. terminate testing.
            tune_p = AUTOTUNE_SP_MIN;
            counter = AUTOTUNE_SUCCESS_COUNT;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
        } else if (gain[frq_cnt] > gain[freq_cnt_max]) {
            freq_cnt_max = frq_cnt;
            phase_max = phase[frq_cnt];
            sp_prev_gain = gain[frq_cnt];
        } else if (freq[frq_cnt] > max_sweep_freq || (gain[frq_cnt] > 0.0f && gain[frq_cnt] < 0.5f)) {
            // must be past peak, continue on to determine angle p
            frq_cnt = 11;
        }
        frq_cnt++;
        if (frq_cnt == 12) {
            freq[frq_cnt] = freq[freq_cnt_max];
            curr_test.freq = freq[frq_cnt];
        } else {
            freq[frq_cnt] = freq[frq_cnt-1] + test_freq_incr;
            curr_test.freq = freq[frq_cnt];
        }
    }

    // once finished with sweep of frequencies, cnt = 12 is used to then tune for max response gain
    if (frq_cnt >= 12 && is_equal(start_freq,stop_freq)) {
        if (gain[frq_cnt] < max_resp_gain && tune_p < AUTOTUNE_SP_MAX && !find_peak) {
            // keep increasing tuning gain unless phase changes or max response gain is achieved
            if (phase[frq_cnt]-phase_max > 20.0f && phase[frq_cnt] < 210.0f) {
                freq[frq_cnt] += 0.5 * test_freq_incr;
                find_peak = true;
            } else {
                tune_p += gain_incr;
                freq[frq_cnt] = freq[freq_cnt_max];
                if (tune_p >= AUTOTUNE_SP_MAX) {
                    tune_p = AUTOTUNE_SP_MAX;
                    counter = AUTOTUNE_SUCCESS_COUNT;
                    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
                }
            }
            curr_test.freq = freq[frq_cnt];
            sp_prev_gain = gain[frq_cnt];
        } else if (gain[frq_cnt] > 1.1f * max_resp_gain && tune_p > AUTOTUNE_SP_MIN && !find_peak) {
            tune_p -= gain_incr;
        } else if (find_peak) {
            // find the frequency where the response gain is maximum
            if (gain[frq_cnt] > sp_prev_gain) {
                freq[frq_cnt] += 0.5 * test_freq_incr;
            } else {
                find_peak = false;
                phase_max = phase[frq_cnt];
            }
            curr_test.freq = freq[frq_cnt];
            sp_prev_gain = gain[frq_cnt];
        } else {
            // adjust tuning gain so max response gain is not exceeded
            if (sp_prev_gain < max_resp_gain && gain[frq_cnt] > max_resp_gain) {
                float adj_factor = (max_resp_gain - gain[frq_cnt]) / (gain[frq_cnt] - sp_prev_gain);
                tune_p = tune_p + gain_incr * adj_factor; 
            }
            counter = AUTOTUNE_SUCCESS_COUNT;
        }
    }
    if (counter == AUTOTUNE_SUCCESS_COUNT) {
        start_freq = 0.0f;  //initializes next test that uses dwell test
        reset_sweep_variables();
        curr_test.freq = freq[0];
        frq_cnt = 0;
    } else {
        start_freq = curr_test.freq;
        stop_freq = curr_test.freq;
    }
}

// updating_max_gains: use dwells at increasing frequency to determine gain at which instability will occur
void AC_AutoTune_Heli::updating_max_gains(float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_p, max_gain_data &max_gain_d, float &tune_p, float &tune_d)
{
    float test_freq_incr = 1.0f * M_PI * 2.0f;

    if (!is_equal(start_freq,stop_freq)) {
        frq_cnt = 2;
        if (!is_zero(sweep_mtr.ph180.freq)) {
            freq[frq_cnt] = sweep_mtr.ph180.freq - 0.5f * test_freq_incr;
        } else {
            freq[frq_cnt] = min_sweep_freq;
        }
        curr_test.freq = freq[frq_cnt];
        start_freq = curr_test.freq;
        stop_freq = curr_test.freq;

    } else if (frq_cnt < 12 && is_equal(start_freq,stop_freq)) {
        if (frq_cnt > 2 && phase[frq_cnt] > 161.0f && phase[frq_cnt] < 270.0f &&
            !find_middle && !found_max_p) {
            find_middle = true;
        } else if (find_middle && !found_max_p) {
            if (phase[frq_cnt] > 161.0f) {
                // interpolate between frq_cnt-2 and frq_cnt
                max_gain_p.freq = linear_interpolate(freq[frq_cnt-2],freq[frq_cnt],161.0f,phase[frq_cnt-2],phase[frq_cnt]);
                max_gain_p.gain = linear_interpolate(gain[frq_cnt-2],gain[frq_cnt],161.0f,phase[frq_cnt-2],phase[frq_cnt]);
            } else {
                // interpolate between frq_cnt-1 and frq_cnt
                max_gain_p.freq = linear_interpolate(freq[frq_cnt],freq[frq_cnt-1],161.0f,phase[frq_cnt],phase[frq_cnt-1]);
                max_gain_p.gain = linear_interpolate(gain[frq_cnt],gain[frq_cnt-1],161.0f,phase[frq_cnt],phase[frq_cnt-1]);
            }
            max_gain_p.phase = 161.0f;
            max_gain_p.max_allowed = powf(10.0f,-1 * (log10f(max_gain_p.gain) * 20.0f + 2.42) / 20.0f);
            // limit max gain allowed to be no more than 2x the max p gain limit to keep initial gains bounded
            max_gain_p.max_allowed = constrain_float(max_gain_p.max_allowed, 0.0f, 2.0f * AUTOTUNE_RP_MAX);
            found_max_p = true;
            find_middle = false;

            if (!is_zero(sweep_mtr.ph270.freq)) {
                // set freq cnt back to reinitialize process
                frq_cnt = 1;  // set to 1 because it will be incremented
                // set frequency back at least one test_freq_incr as it will be added
                freq[1] = sweep_mtr.ph270.freq - 1.5f * test_freq_incr;
            }
        }
        if (frq_cnt > 2 && phase[frq_cnt] > 251.0f && phase[frq_cnt] < 360.0f &&
            !find_middle && !found_max_d && found_max_p) {
            find_middle = true;
        } else if (find_middle && found_max_p && !found_max_d) {
            if (phase[frq_cnt] > 251.0f) {
                // interpolate between frq_cnt-2 and frq_cnt
                max_gain_d.freq = linear_interpolate(freq[frq_cnt-2],freq[frq_cnt],251.0f,phase[frq_cnt-2],phase[frq_cnt]);
                max_gain_d.gain = linear_interpolate(gain[frq_cnt-2],gain[frq_cnt],251.0f,phase[frq_cnt-2],phase[frq_cnt]);
            } else {
                // interpolate between frq_cnt-1 and frq_cnt
                max_gain_d.freq = linear_interpolate(freq[frq_cnt],freq[frq_cnt-1],251.0f,phase[frq_cnt],phase[frq_cnt-1]);
                max_gain_d.gain = linear_interpolate(gain[frq_cnt],gain[frq_cnt-1],251.0f,phase[frq_cnt],phase[frq_cnt-1]);
            }
            max_gain_d.phase = 251.0f;
            max_gain_d.max_allowed = powf(10.0f,-1 * (log10f(max_gain_d.freq * max_gain_d.gain) * 20.0f + 2.42) / 20.0f);
            // limit max gain allowed to be no more than 2x the max d gain limit to keep initial gains bounded
            max_gain_d.max_allowed = constrain_float(max_gain_d.max_allowed, 0.0f, 2.0f * AUTOTUNE_RD_MAX);
            found_max_d = true;
            find_middle = false;
        }
        // stop progression in frequency.
        if ((frq_cnt > 1 && phase[frq_cnt] > 330.0f && !is_zero(phase[frq_cnt])) || found_max_d) {
            frq_cnt = 11;
        }
        frq_cnt++;
        if (frq_cnt == 12) {
            counter = AUTOTUNE_SUCCESS_COUNT;
            // reset variables for next test
            curr_test.freq = freq[0];
            frq_cnt = 0;
            start_freq = 0.0f;  //initializes next test that uses dwell test
        } else {
            if (frq_cnt == 3 && phase[2] >= 161.0f && !found_max_p) {
                // phase greater than 161 deg won't allow max p to be found
                // reset freq cnt to 2 and lower dwell freq to push phase below 161 deg
                frq_cnt = 2;
                freq[frq_cnt] = freq[frq_cnt] - 0.5f * test_freq_incr;
            } else if (frq_cnt == 3 && phase[2] >= 251.0f && !found_max_d) {
                // phase greater than 161 deg won't allow max p to be found
                // reset freq cnt to 2 and lower dwell freq to push phase below 161 deg
                frq_cnt = 2;
                freq[frq_cnt] = freq[frq_cnt] - 0.5f * test_freq_incr;
            } else if (find_middle) {
                freq[frq_cnt] = freq[frq_cnt-1] - 0.5f * test_freq_incr;
            } else {
                freq[frq_cnt] = freq[frq_cnt-1] + test_freq_incr;
            }
            curr_test.freq = freq[frq_cnt];
            start_freq = curr_test.freq;
            stop_freq = curr_test.freq;
        }
    }
    if (found_max_p && found_max_d) {
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Max rate P freq=%f gain=%f", (double)(max_rate_p.freq), (double)(max_rate_p.gain));
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: ph=%f rate_p=%f", (double)(max_rate_p.phase), (double)(max_rate_p.max_allowed));
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Max Rate D freq=%f gain=%f", (double)(max_rate_d.freq), (double)(max_rate_d.gain));
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: ph=%f rate_d=%f", (double)(max_rate_d.phase), (double)(max_rate_d.max_allowed));
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

#if HAL_LOGGING_ENABLED
// log autotune summary data
void AC_AutoTune_Heli::Log_AutoTune()
{

    switch (axis) {
    case ROLL:
        Log_Write_AutoTune(axis, tune_type, test_freq[freq_cnt], test_gain[freq_cnt], test_phase[freq_cnt], tune_roll_rff,  tune_roll_rp, tune_roll_rd, tune_roll_sp, test_accel_max);
        break;
    case PITCH:
        Log_Write_AutoTune(axis, tune_type, test_freq[freq_cnt], test_gain[freq_cnt], test_phase[freq_cnt], tune_pitch_rff, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, test_accel_max);
        break;
    case YAW:
    case YAW_D:
        Log_Write_AutoTune(axis, tune_type, test_freq[freq_cnt], test_gain[freq_cnt], test_phase[freq_cnt], tune_yaw_rff, tune_yaw_rp, tune_yaw_rd, tune_yaw_sp, test_accel_max);
        break;
    }
}

// log autotune time history results for command, angular rate, and attitude
void AC_AutoTune_Heli::Log_AutoTuneDetails()
{
    if (tune_type == SP_UP) {
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
void AC_AutoTune_Heli::Log_Write_AutoTune(uint8_t _axis, uint8_t tune_step, float dwell_freq, float meas_gain, float meas_phase, float new_gain_rff, float new_gain_rp, float new_gain_rd, float new_gain_sp, float max_accel)
{
    AP::logger().Write(
        "ATNH",
        "TimeUS,Axis,TuneStep,Freq,Gain,Phase,RFF,RP,RD,SP,ACC",
        "s--E-d-----",
        "F--000-----",
        "QBBffffffff",
        AP_HAL::micros64(),
        axis,
        tune_step,
        dwell_freq,
        meas_gain,
        meas_phase,
        new_gain_rff,
        new_gain_rp,
        new_gain_rd,
        new_gain_sp,
        max_accel);
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
        freq_cnt = 0;
        start_freq = 0.0f;
        stop_freq = 0.0f;
    }
}

// reset the update gain variables for heli
void AC_AutoTune_Heli::reset_update_gain_variables()
{
    // reset feedforward update gain variables
    ff_up_first_iter = true;
    first_dir_complete = false;

    // reset max gain variables
    reset_maxgains_update_gain_variables();

    // reset rd_up variables
    rd_prev_good_frq_cnt = 0;
    rd_prev_gain = 0.0f;

    // reset rp_up variables
    rp_prev_good_frq_cnt = 0;

    // reset sp_up variables
    phase_max = 0.0f;
    sp_prev_gain = 0.0f;
    find_peak = false;

}

// reset the max_gains update gain variables
void AC_AutoTune_Heli::reset_maxgains_update_gain_variables()
{
    max_rate_p = {};
    max_rate_d = {};
    found_max_p = false;
    found_max_d = false;
    find_middle = false;

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
        tune_seq[seq_cnt] = RFF_UP;
        seq_cnt++;
    }
    if (seq_bitmask & AUTOTUNE_SEQ_BITMASK_RATE_D) {
        tune_seq[seq_cnt] = MAX_GAINS;
        seq_cnt++;
        tune_seq[seq_cnt] = RD_UP;
        seq_cnt++;
        tune_seq[seq_cnt] = RP_UP;
        seq_cnt++;
    }
    if (seq_bitmask & AUTOTUNE_SEQ_BITMASK_ANGLE_P) {
        tune_seq[seq_cnt] = SP_UP;
        seq_cnt++;
    }
    if (seq_bitmask & AUTOTUNE_SEQ_BITMASK_MAX_GAIN && !(seq_bitmask & AUTOTUNE_SEQ_BITMASK_RATE_D)) {
        tune_seq[seq_cnt] = MAX_GAINS;
        seq_cnt++;
    }
    if (seq_bitmask & AUTOTUNE_SEQ_BITMASK_TUNE_CHECK) {
        tune_seq[seq_cnt] = TUNE_CHECK;
        seq_cnt++;
    }
    tune_seq[seq_cnt] = TUNE_COMPLETE;

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
