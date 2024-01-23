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
    switch (tune_type) {
    case RFF_UP:
        rate_ff_test_init();
        step_time_limit_ms = 10000;
        break;
    case MAX_GAINS:
    case RP_UP:
    case RD_UP:
        // initialize start frequency
        if (is_zero(start_freq)) {
            if (tune_type == RP_UP) {
                // continue using frequency where testing left off or RD_UP completed
                if (test_phase[12] > 0.0f && test_phase[12] < 180.0f) {
                    freq_cnt = 12;
                // start with freq found for sweep where phase was 180 deg
                } else if (!is_zero(sweep.ph180.freq)) {
                    freq_cnt = 12;
                    test_freq[freq_cnt] = sweep.ph180.freq - 0.25f * 3.14159f * 2.0f;
                // otherwise start at min freq to step up in dwell frequency until phase > 160 deg
                } else {
                    freq_cnt = 0;
                    test_freq[freq_cnt] = min_sweep_freq;
                }
                curr_test.freq = test_freq[freq_cnt];
                start_freq = curr_test.freq;
                stop_freq = curr_test.freq;

            // MAX_GAINS and RD_UP both start with a sweep initially but if it has been completed then start dwells at the freq for 180 deg phase
            } else {
                if (!is_zero(sweep.ph180.freq)) {
                    freq_cnt = 12;
                    test_freq[freq_cnt] = sweep.ph180.freq - 0.25f * 3.14159f * 2.0f;
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
        }
        if (!is_equal(start_freq,stop_freq)) {
            // initialize determine_gain function whenever test is initialized
            freqresp.init(AC_AutoTune_FreqResp::InputType::SWEEP, AC_AutoTune_FreqResp::ResponseType::RATE);
            dwell_test_init(start_freq, stop_freq, stop_freq, RATE);
        } else {
            // initialize determine_gain function whenever test is initialized
            freqresp.init(AC_AutoTune_FreqResp::InputType::DWELL, AC_AutoTune_FreqResp::ResponseType::RATE);
            dwell_test_init(start_freq, stop_freq, start_freq, RATE);
        }
        if (!is_zero(start_freq)) {
            // 4 seconds is added to allow aircraft to achieve start attitude.  Then the time to conduct the dwells is added to it.
            step_time_limit_ms = (uint32_t)(4000 + (float)(AUTOTUNE_DWELL_CYCLES + 2) * 1000.0f * M_2PI / start_freq);
        }
        break;
    case SP_UP:
        // initialize start frequency
        if (is_zero(start_freq)) {
            if (!is_zero(sweep.maxgain.freq)) {
                freq_cnt = 12;
                test_freq[freq_cnt] = sweep.maxgain.freq - 0.25f * 3.14159f * 2.0f;
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

        if (!is_equal(start_freq,stop_freq)) {
            // initialize determine gain function
            freqresp.init(AC_AutoTune_FreqResp::InputType::SWEEP, AC_AutoTune_FreqResp::ResponseType::ANGLE);
            dwell_test_init(start_freq, stop_freq, stop_freq, DRB);
        } else {
            // initialize determine gain function
            freqresp.init(AC_AutoTune_FreqResp::InputType::DWELL, AC_AutoTune_FreqResp::ResponseType::ANGLE);
            dwell_test_init(start_freq, stop_freq, start_freq, DRB);
        }

        // TODO add time limit for sweep test
        if (!is_zero(start_freq)) {
            // 1 seconds is added for a little buffer.  Then the time to conduct the dwells is added to it.
            step_time_limit_ms = (uint32_t)(2000 + (float)(AUTOTUNE_DWELL_CYCLES + 7) * 1000.0f * M_2PI / start_freq);
        }
        break;
    case TUNE_CHECK:
        // initialize start frequency
        if (is_zero(start_freq)) {
            start_freq = min_sweep_freq;
            stop_freq = max_sweep_freq;
        }
        // initialize determine gain function
        freqresp.init(AC_AutoTune_FreqResp::InputType::SWEEP, AC_AutoTune_FreqResp::ResponseType::ANGLE);
        dwell_test_init(start_freq, stop_freq, stop_freq, ANGLE);
        // TODO add time limit for sweep test
        if (!is_zero(start_freq)) {
            // 1 seconds is added for a little buffer.  Then the time to conduct the dwells is added to it.
            step_time_limit_ms = (uint32_t)(2000 + (float)(AUTOTUNE_DWELL_CYCLES + 7) * 1000.0f * M_2PI / start_freq);
        }
        break;
    default:
        break;
    }

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
        rate_ff_test_run(AUTOTUNE_HELI_TARGET_ANGLE_RLLPIT_CD, AUTOTUNE_HELI_TARGET_RATE_RLLPIT_CDS, dir_sign);
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
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: target=%f rotation=%f command=%f", (double)(test_tgt_rate_filt*57.3f), (double)(test_rate_filt*57.3f), (double)(test_command_filt));
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: ff=%f", (double)tune_rff);
            break;
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
               } else if (tune_type == SP_UP) {
                   gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: angle_p=%f tune_accel=%f max_accel=%f", (double)(tune_sp), (double)(tune_accel), (double)(test_accel_max));
               }
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: max_freq=%f max_gain=%f", (double)(sweep.maxgain.freq), (double)(sweep.maxgain.gain));
                gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: ph180_freq=%f ph180_gain=%f", (double)(sweep.ph180.freq), (double)(sweep.ph180.gain));
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


void AC_AutoTune_Heli::rate_ff_test_init()
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
    rate_request_cds.reset(0);
    rate_request_cds.set_cutoff_frequency(1.0f);
    angle_request_cd.reset(0);
    angle_request_cd.set_cutoff_frequency(1.0f);
}

void AC_AutoTune_Heli::rate_ff_test_run(float max_angle_cd, float target_rate_cds, float dir_sign)
{
    float gyro_reading = 0.0f;
    float command_reading =0.0f;
    float tgt_rate_reading = 0.0f;
    const uint32_t now = AP_HAL::millis();

    target_rate_cds = dir_sign * target_rate_cds;

    switch (axis) {
    case ROLL:
        gyro_reading = ahrs_view->get_gyro().x;
        command_reading = motors->get_roll();
        tgt_rate_reading = attitude_control->rate_bf_targets().x;
        if (settle_time > 0) {
            settle_time--;
            trim_command_reading = motors->get_roll();
            rate_request_cds.reset(gyro_reading);
        } else if (((ahrs_view->roll_sensor <= max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->roll_sensor >= -max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 0) {
            rate_request_cds.apply(target_rate_cds, AP::scheduler().get_loop_period_s());
            attitude_control->input_rate_bf_roll_pitch_yaw(rate_request_cds.get(), 0.0f, 0.0f);
        } else if (((ahrs_view->roll_sensor > max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->roll_sensor < -max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 0) {
            ff_test_phase = 1;
            rate_request_cds.apply(-target_rate_cds, AP::scheduler().get_loop_period_s());
            attitude_control->input_rate_bf_roll_pitch_yaw(rate_request_cds.get(), 0.0f, 0.0f);
            attitude_control->rate_bf_roll_target(rate_request_cds.get());
        } else if (((ahrs_view->roll_sensor >= -max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->roll_sensor <= max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 1 ) {
            rate_request_cds.apply(-target_rate_cds, AP::scheduler().get_loop_period_s());
            attitude_control->input_rate_bf_roll_pitch_yaw(rate_request_cds.get(), 0.0f, 0.0f);
            attitude_control->rate_bf_roll_target(rate_request_cds.get());
        } else if (((ahrs_view->roll_sensor < -max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->roll_sensor > max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 1 ) {
            ff_test_phase = 2;
            attitude_control->reset_target_and_rate(false);
            angle_request_cd.reset(ahrs_view->roll_sensor);
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(angle_request_cd.get(), start_angles.y, 0.0f);
        } else if (ff_test_phase == 2 ) {
            angle_request_cd.apply(start_angles.x, AP::scheduler().get_loop_period_s());
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(angle_request_cd.get(), start_angles.y, 0.0f);
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
            rate_request_cds.reset(gyro_reading);
        } else if (((ahrs_view->pitch_sensor <= max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->pitch_sensor >= -max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 0) {
            rate_request_cds.apply(target_rate_cds, AP::scheduler().get_loop_period_s());
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, rate_request_cds.get(), 0.0f);
        } else if (((ahrs_view->pitch_sensor > max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->pitch_sensor < -max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 0) {
            ff_test_phase = 1;
            rate_request_cds.apply(-target_rate_cds, AP::scheduler().get_loop_period_s());
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, rate_request_cds.get(), 0.0f);
            attitude_control->rate_bf_pitch_target(rate_request_cds.get());
        } else if (((ahrs_view->pitch_sensor >= -max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->pitch_sensor <= max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 1 ) {
            rate_request_cds.apply(-target_rate_cds, AP::scheduler().get_loop_period_s());
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, rate_request_cds.get(), 0.0f);
            attitude_control->rate_bf_pitch_target(rate_request_cds.get());
        } else if (((ahrs_view->pitch_sensor < -max_angle_cd + start_angle && is_positive(dir_sign))
                   || (ahrs_view->pitch_sensor > max_angle_cd + start_angle && !is_positive(dir_sign)))
                   && ff_test_phase == 1 ) {
            ff_test_phase = 2;
            attitude_control->reset_target_and_rate(false);
            angle_request_cd.reset(ahrs_view->pitch_sensor);
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, angle_request_cd.get(), 0.0f);
        } else if (ff_test_phase == 2 ) {
            angle_request_cd.apply(start_angles.y, AP::scheduler().get_loop_period_s());
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(start_angles.x, angle_request_cd.get(), 0.0f);
            phase_out_time--;
        }
        break;
    case YAW:
    case YAW_D:
        gyro_reading = ahrs_view->get_gyro().z;
        command_reading = motors->get_yaw();
        tgt_rate_reading = attitude_control->rate_bf_targets().z;
        if (settle_time > 0) {
            settle_time--;
            trim_command_reading = motors->get_yaw();
            trim_heading = ahrs_view->yaw_sensor;
            rate_request_cds.reset(gyro_reading);
        } else if (((wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) <= 2.0f * max_angle_cd && is_positive(dir_sign))
                   || (wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) >= -2.0f * max_angle_cd && !is_positive(dir_sign)))
                   && ff_test_phase == 0) {
            rate_request_cds.apply(target_rate_cds, AP::scheduler().get_loop_period_s());
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, rate_request_cds.get());
        } else if (((wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) > 2.0f * max_angle_cd && is_positive(dir_sign))
                   || (wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) < -2.0f * max_angle_cd && !is_positive(dir_sign)))
                   && ff_test_phase == 0) {
            ff_test_phase = 1;
            rate_request_cds.apply(-target_rate_cds, AP::scheduler().get_loop_period_s());
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, rate_request_cds.get());
            attitude_control->rate_bf_yaw_target(rate_request_cds.get());
        } else if (((wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) >= -2.0f * max_angle_cd && is_positive(dir_sign))
                   || (wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) <= 2.0f * max_angle_cd && !is_positive(dir_sign)))
                   && ff_test_phase == 1 ) {
            rate_request_cds.apply(-target_rate_cds, AP::scheduler().get_loop_period_s());
            attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, rate_request_cds.get());
            attitude_control->rate_bf_yaw_target(rate_request_cds.get());
        } else if (((wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) < -2.0f * max_angle_cd && is_positive(dir_sign))
                   || (wrap_180_cd(ahrs_view->yaw_sensor - trim_heading) > 2.0f * max_angle_cd && !is_positive(dir_sign)))
                   && ff_test_phase == 1 ) {
            ff_test_phase = 2;
            attitude_control->reset_yaw_target_and_rate(false);
            angle_request_cd.reset(wrap_180_cd(ahrs_view->yaw_sensor - trim_heading));
            attitude_control->input_euler_angle_roll_pitch_yaw(start_angles.x, start_angles.y, angle_request_cd.get(), false);
        } else if (ff_test_phase == 2 ) {
            angle_request_cd.apply(0.0f, AP::scheduler().get_loop_period_s());
            attitude_control->input_euler_angle_roll_pitch_yaw(start_angles.x, start_angles.y, wrap_360_cd(trim_heading + angle_request_cd.get()), false);
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
    case YAW_D:
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
    }

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

    if (dwell_type == RATE) {
        filt_pit_roll_cd.set_cutoff_frequency(0.2f * start_frq);
        filt_heading_error_cd.set_cutoff_frequency(0.2f * start_frq);

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
        case YAW_D:
            trim_meas_rate = ahrs_view->get_gyro().z;
            ff_term = attitude_control->get_rate_yaw_pid().get_ff();
            p_term = attitude_control->get_rate_yaw_pid().get_p();
            break;
        }
        trim_pff_out = ff_term + p_term;
    }

    if (!is_equal(start_frq, stop_frq)) {
        reset_sweep_variables();
        curr_test.gain = 0.0f;
        curr_test.phase = 0.0f;
    }

    chirp_input.init(sweep_time_ms * 0.001f, start_frq / M_2PI, stop_frq / M_2PI, 0.0f, 0.0001f * sweep_time_ms, 0.0f);

}

void AC_AutoTune_Heli::dwell_test_run(uint8_t freq_resp_input, float start_frq, float stop_frq, float &dwell_gain, float &dwell_phase, DwellType dwell_type)
{
    float gyro_reading = 0.0f;
    float command_reading = 0.0f;
    float tgt_rate_reading = 0.0f;
    float tgt_attitude;
    const uint32_t now = AP_HAL::millis();
    float target_angle_cd = 0.0f;
    float target_rate_cds = 0.0f;
    float dwell_freq = start_frq;
    float target_rate_mag_cds;
    const float att_hold_gain = 4.5f;

    float cycle_time_ms = 0;
    if (!is_zero(dwell_freq)) {
        cycle_time_ms = 1000.0f * M_2PI / dwell_freq;
    }

    if (dwell_type == RATE) {
        // keep controller from requesting too high of a rate
        tgt_attitude = 2.5f * 0.01745f;
        target_rate_mag_cds = dwell_freq * tgt_attitude * 5730.0f;
        if (target_rate_mag_cds > 5000.0f) {
            target_rate_mag_cds = 5000.0f;
        }
    } else {
        tgt_attitude = 5.0f * 0.01745f;
        // adjust target attitude based on input_tc so amplitude decrease with increased frequency is minimized
        const float freq_co = 1.0f / attitude_control->get_input_tc();
        const float added_ampl = (safe_sqrt(powf(dwell_freq,2.0) + powf(freq_co,2.0)) / freq_co) - 1.0f;
        tgt_attitude = constrain_float(0.08725f * (1.0f + 0.2f * added_ampl), 0.08725f, 0.5235f);
    }

    // body frame calculation of velocity
    Vector3f velocity_ned, velocity_bf;
    if (ahrs_view->get_velocity_NED(velocity_ned)) {
        velocity_bf.x = velocity_ned.x * ahrs_view->cos_yaw() + velocity_ned.y * ahrs_view->sin_yaw();
        velocity_bf.y = -velocity_ned.x * ahrs_view->sin_yaw() + velocity_ned.y * ahrs_view->cos_yaw();
    }

    Vector3f attitude_cd = Vector3f((float)ahrs_view->roll_sensor, (float)ahrs_view->pitch_sensor, (float)ahrs_view->yaw_sensor);
    if (settle_time == 0) {
        if (dwell_type == RATE) {
            target_rate_cds = -chirp_input.update((now - dwell_start_time_ms) * 0.001, target_rate_mag_cds);
            filt_pit_roll_cd.apply(Vector2f(attitude_cd.x,attitude_cd.y), AP::scheduler().get_loop_period_s());
            filt_heading_error_cd.apply(wrap_180_cd(trim_attitude_cd.z - attitude_cd.z), AP::scheduler().get_loop_period_s());
        } else {
            target_angle_cd = -chirp_input.update((now - dwell_start_time_ms) * 0.001, tgt_attitude * 5730.0f);
        }
        const Vector2f att_fdbk {
            -5730.0f * vel_hold_gain * velocity_bf.y,
            5730.0f * vel_hold_gain * velocity_bf.x
        };
        filt_att_fdbk_from_velxy_cd.apply(att_fdbk, AP::scheduler().get_loop_period_s());
        dwell_freq = chirp_input.get_frequency_rads();
    } else {
        if (dwell_type == RATE) {
            target_rate_cds = 0.0f;
            trim_command = command_out;
            trim_attitude_cd = attitude_cd;
            filt_pit_roll_cd.reset(Vector2f(attitude_cd.x,attitude_cd.y));
            filt_heading_error_cd.reset(0.0f);
        } else {
            target_angle_cd = 0.0f;
            trim_yaw_tgt_reading = (float)attitude_control->get_att_target_euler_cd().z;
            trim_yaw_heading_reading = (float)ahrs_view->yaw_sensor;
        }
        dwell_start_time_ms = now;
        filt_att_fdbk_from_velxy_cd.reset(Vector2f(0.0f,0.0f));
        settle_time--;
    }

    if (dwell_type == RATE) {
        // limit rate correction for position hold
        Vector3f trim_rate_cds {
            constrain_float(att_hold_gain * ((trim_attitude_cd.x + filt_att_fdbk_from_velxy_cd.get().x) - filt_pit_roll_cd.get().x), -15000.0f, 15000.0f),
            constrain_float(att_hold_gain * ((trim_attitude_cd.y + filt_att_fdbk_from_velxy_cd.get().y) - filt_pit_roll_cd.get().y), -15000.0f, 15000.0f),
            constrain_float(att_hold_gain * filt_heading_error_cd.get(), -15000.0f, 15000.0f)
        };
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
                trim_rate_cds.x += ff_rate_contr;
                attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, trim_rate_cds.y, 0.0f);
                attitude_control->rate_bf_roll_target(target_rate_cds + trim_rate_cds.x);
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
                trim_rate_cds.y += ff_rate_contr;
                attitude_control->input_rate_bf_roll_pitch_yaw(trim_rate_cds.x, 0.0f, 0.0f);
                attitude_control->rate_bf_pitch_target(target_rate_cds + trim_rate_cds.y);
            } else {
                attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
                if (!is_zero(attitude_control->get_rate_pitch_pid().ff() + attitude_control->get_rate_pitch_pid().kP())) {
                    float trim_tgt_rate_cds = 5730.0f * (trim_pff_out + trim_meas_rate * attitude_control->get_rate_pitch_pid().kP()) / (attitude_control->get_rate_pitch_pid().ff() + attitude_control->get_rate_pitch_pid().kP());
                    attitude_control->rate_bf_pitch_target(trim_tgt_rate_cds);
                }
            }
            break;
        case YAW:
        case YAW_D:
            gyro_reading = ahrs_view->get_gyro().z;
            command_reading = motors->get_yaw();
            tgt_rate_reading = attitude_control->rate_bf_targets().z;
            if (settle_time == 0) {
                float rp_rate_contr = 0.0f;
                if (tune_yaw_rp > 0.0f) {
                    rp_rate_contr = 5730.0f * trim_command / tune_yaw_rp;
                }
                trim_rate_cds.z += rp_rate_contr;
                attitude_control->input_rate_bf_roll_pitch_yaw(trim_rate_cds.x, trim_rate_cds.y, 0.0f);
                attitude_control->rate_bf_yaw_target(target_rate_cds + trim_rate_cds.z);
            } else {
                attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
                if (!is_zero(attitude_control->get_rate_yaw_pid().ff() + attitude_control->get_rate_yaw_pid().kP())) {
                    float trim_tgt_rate_cds = 5730.0f * (trim_pff_out + trim_meas_rate * attitude_control->get_rate_yaw_pid().kP()) / (attitude_control->get_rate_yaw_pid().ff() + attitude_control->get_rate_yaw_pid().kP());
                    attitude_control->rate_bf_yaw_target(trim_tgt_rate_cds);
                }
            }
            break;
        }
    } else {
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
            } else {
                tgt_rate_reading = ((float)attitude_control->get_att_target_euler_cd().y) / 5730.0f;
                gyro_reading = ((float)ahrs_view->pitch_sensor) / 5730.0f;
            }
            break;
        case YAW:
        case YAW_D:
            command_reading = motors->get_yaw();
            if (dwell_type == DRB) {
                tgt_rate_reading = (target_angle_cd) / 5730.0f;
                gyro_reading = (wrap_180_cd((float)ahrs_view->yaw_sensor - trim_yaw_heading_reading - target_angle_cd)) / 5730.0f;
            } else {
                tgt_rate_reading = (wrap_180_cd((float)attitude_control->get_att_target_euler_cd().z - trim_yaw_tgt_reading)) / 5730.0f;
                gyro_reading = (wrap_180_cd((float)ahrs_view->yaw_sensor - trim_yaw_heading_reading)) / 5730.0f;
            }
            attitude_control->input_euler_angle_roll_pitch_yaw(trim_angle_cd.x, trim_angle_cd.y, wrap_180_cd(trim_yaw_tgt_reading + target_angle_cd), false);
            break;
        }
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

    // wait for dwell to start before determining gain and phase
    if ((float)(now - dwell_start_time_ms) > 6.25f * cycle_time_ms || (!is_equal(start_frq,stop_frq) && settle_time == 0)) {
        if (freq_resp_input == 1) {
            freqresp.update(command_out,filt_target_rate,rotation_rate, dwell_freq);
        } else {
            freqresp.update(command_out,command_out,rotation_rate, dwell_freq);
        }

        if (freqresp.is_cycle_complete()) {
            if (!is_equal(start_frq,stop_frq)) {
                curr_test.freq = freqresp.get_freq();
                curr_test.gain = freqresp.get_gain();
                curr_test.phase = freqresp.get_phase();
                if (dwell_type == DRB) {test_accel_max = freqresp.get_accel_max();}
                // reset cycle_complete to allow indication of next cycle
                freqresp.reset_cycle_complete();
#if HAL_LOGGING_ENABLED
                // log sweep data
                Log_AutoTuneSweep();
#endif
            } else {
                dwell_gain = freqresp.get_gain();
                dwell_phase = freqresp.get_phase();
                if (dwell_type == DRB) {test_accel_max = freqresp.get_accel_max();}
            }
        }
    }

    // set sweep data if a frequency sweep is being conducted
    if (!is_equal(start_frq,stop_frq) && (float)(now - dwell_start_time_ms) > 2.5f * cycle_time_ms) {
        // track sweep phase to prevent capturing 180 deg and 270 deg data after phase has wrapped.
        if (curr_test.phase > 180.0f && sweep.progress == 0) {
            sweep.progress = 1;
        } else if (curr_test.phase > 270.0f && sweep.progress == 1) {
            sweep.progress = 2;
        }
        if (curr_test.phase <= 160.0f && curr_test.phase >= 150.0f && sweep.progress == 0) {
            sweep.ph180 = curr_test;
        }
        if (curr_test.phase <= 250.0f && curr_test.phase >= 240.0f && sweep.progress == 1) {
            sweep.ph270 = curr_test;
        }
        if (curr_test.gain > sweep.maxgain.gain) {
            sweep.maxgain = curr_test;
        }
        if (now - step_start_time_ms >= sweep_time_ms + 200) {
            // we have passed the maximum stop time
            step = UPDATE_GAINS;
        }
    } else {
        if (now - step_start_time_ms >= step_time_limit_ms || freqresp.is_cycle_complete()) {
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
        updating_rate_ff_up(tune_roll_rff, test_tgt_rate_filt*5730.0f, test_rate_filt*5730.0f, test_command_filt);
        break;
    case PITCH:
        updating_rate_ff_up(tune_pitch_rff, test_tgt_rate_filt*5730.0f, test_rate_filt*5730.0f, test_command_filt);
        break;
    case YAW:
    case YAW_D:
        updating_rate_ff_up(tune_yaw_rff, test_tgt_rate_filt*5730.0f, test_rate_filt*5730.0f, test_command_filt);
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
void AC_AutoTune_Heli::updating_rate_ff_up(float &tune_ff, float rate_target, float meas_rate, float meas_command)
{

    if (ff_up_first_iter) {
        if (!is_zero(meas_rate)) {
            tune_ff = 5730.0f * meas_command / meas_rate;
        }
        tune_ff = constrain_float(tune_ff, AUTOTUNE_RFF_MIN, AUTOTUNE_RFF_MAX);
        ff_up_first_iter = false;
    } else if (is_positive(rate_target * meas_rate) && fabsf(meas_rate) < 1.05f * fabsf(rate_target) &&
               fabsf(meas_rate) > 0.95f * fabsf(rate_target)) {
        if (!first_dir_complete) {
            first_dir_rff = tune_ff;
            first_dir_complete = true;
            positive_direction = !positive_direction;
        } else {
            counter = AUTOTUNE_SUCCESS_COUNT;
            tune_ff = 0.95f * 0.5 * (tune_ff + first_dir_rff);
            tune_ff = constrain_float(tune_ff, AUTOTUNE_RFF_MIN, AUTOTUNE_RFF_MAX);
        }
    } else if (is_positive(rate_target * meas_rate) && fabsf(meas_rate) > 1.05f * fabsf(rate_target)) {
        tune_ff = 0.98f * tune_ff;
        if (tune_ff <= AUTOTUNE_RFF_MIN) {
            tune_ff = AUTOTUNE_RFF_MIN;
            counter = AUTOTUNE_SUCCESS_COUNT;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
        }
    } else if (is_positive(rate_target * meas_rate) && fabsf(meas_rate) < 0.95f * fabsf(rate_target)) {
        tune_ff = 1.02f * tune_ff;
        if (tune_ff >= AUTOTUNE_RFF_MAX) {
            tune_ff = AUTOTUNE_RFF_MAX;
            counter = AUTOTUNE_SUCCESS_COUNT;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
        }
    } else {
        if (!is_zero(meas_rate)) {
            tune_ff = 5730.0f * meas_command / meas_rate;
        }
        tune_ff = constrain_float(tune_ff, AUTOTUNE_RFF_MIN, AUTOTUNE_RFF_MAX);
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

    // frequency sweep was conducted.  check to see if freq for 180 deg phase was determined and start there if it was
    if (!is_equal(start_freq,stop_freq)) {
        if (!is_zero(sweep.ph180.freq)) {
            freq[frq_cnt] = sweep.ph180.freq - 0.5f * test_freq_incr;
            frq_cnt = 12;
            freq_cnt_max = frq_cnt;
        } else {
            frq_cnt = 1;
            freq[frq_cnt] = min_sweep_freq;
        }
        curr_test.freq = freq[frq_cnt];
    }
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
        reset_sweep_variables();
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
        if (!is_zero(sweep.maxgain.freq)) {
            frq_cnt = 12;
            freq[frq_cnt] = sweep.maxgain.freq - 0.5f * test_freq_incr;
            freq_cnt_max = frq_cnt;
        } else {
            frq_cnt = 1;
            freq[frq_cnt] = min_sweep_freq;
            freq_cnt_max = 0;
        }
        curr_test.freq = freq[frq_cnt];
    }
    if (freq_cnt < 12 && is_equal(start_freq,stop_freq)) {
        if (gain[freq_cnt] > max_resp_gain && tune_p > AUTOTUNE_SP_MIN) {
            // exceeded max response gain already, reduce tuning gain to remain under max response gain
            tune_p -= gain_incr;
            // force counter to stay on frequency
            freq_cnt -= 1;
        } else if (gain[freq_cnt] > max_resp_gain && tune_p <= AUTOTUNE_SP_MIN) {
            // exceeded max response gain at the minimum allowable tuning gain. terminate testing.
            tune_p = AUTOTUNE_SP_MIN;
            counter = AUTOTUNE_SUCCESS_COUNT;
            LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
        } else if (gain[freq_cnt] > gain[freq_cnt_max]) {
            freq_cnt_max = freq_cnt;
            phase_max = phase[freq_cnt];
            sp_prev_gain = gain[freq_cnt];
        } else if (freq[freq_cnt] > max_sweep_freq || (gain[freq_cnt] > 0.0f && gain[freq_cnt] < 0.5f)) {
            // must be past peak, continue on to determine angle p
            freq_cnt = 11;
        }
        freq_cnt++;
        if (freq_cnt == 12) {
            freq[freq_cnt] = freq[freq_cnt_max];
            curr_test.freq = freq[freq_cnt];
        } else {
            freq[freq_cnt] = freq[freq_cnt-1] + test_freq_incr;
            curr_test.freq = freq[freq_cnt];
        }
    }

    // once finished with sweep of frequencies, cnt = 12 is used to then tune for max response gain
    if (freq_cnt >= 12 && is_equal(start_freq,stop_freq)) {
        if (gain[freq_cnt] < max_resp_gain && tune_p < AUTOTUNE_SP_MAX && !find_peak) {
            // keep increasing tuning gain unless phase changes or max response gain is achieved
            if (phase[freq_cnt]-phase_max > 20.0f && phase[freq_cnt] < 210.0f) {
                freq[freq_cnt] += 0.5 * test_freq_incr;
                find_peak = true;
            } else {
                tune_p += gain_incr;
                freq[freq_cnt] = freq[freq_cnt_max];
                if (tune_p >= AUTOTUNE_SP_MAX) {
                    tune_p = AUTOTUNE_SP_MAX;
                    counter = AUTOTUNE_SUCCESS_COUNT;
                    LOGGER_WRITE_EVENT(LogEvent::AUTOTUNE_REACHED_LIMIT);
                }
            }
            curr_test.freq = freq[freq_cnt];
            sp_prev_gain = gain[freq_cnt];
        } else if (gain[freq_cnt] > 1.1f * max_resp_gain && tune_p > AUTOTUNE_SP_MIN && !find_peak) {
            tune_p -= gain_incr;
        } else if (find_peak) {
            // find the frequency where the response gain is maximum
            if (gain[freq_cnt] > sp_prev_gain) {
                freq[freq_cnt] += 0.5 * test_freq_incr;
            } else {
                find_peak = false;
                phase_max = phase[freq_cnt];
            }
            curr_test.freq = freq[freq_cnt];
            sp_prev_gain = gain[freq_cnt];
        } else {
            // adjust tuning gain so max response gain is not exceeded
            if (sp_prev_gain < max_resp_gain && gain[freq_cnt] > max_resp_gain) {
                float adj_factor = (max_resp_gain - gain[freq_cnt]) / (gain[freq_cnt] - sp_prev_gain);
                tune_p = tune_p + gain_incr * adj_factor; 
            }
            counter = AUTOTUNE_SUCCESS_COUNT;
        }
    }
    if (counter == AUTOTUNE_SUCCESS_COUNT) {
        start_freq = 0.0f;  //initializes next test that uses dwell test
        reset_sweep_variables();
        curr_test.freq = freq[0];
        freq_cnt = 0;
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
        if (!is_zero(sweep.ph180.freq)) {
            freq[frq_cnt] = sweep.ph180.freq - 0.5f * test_freq_incr;
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

            if (!is_zero(sweep.ph270.freq)) {
                // set freq cnt back to reinitialize process
                frq_cnt = 1;  // set to 1 because it will be incremented
                // set frequency back at least one test_freq_incr as it will be added
                freq[1] = sweep.ph270.freq - 1.5f * test_freq_incr;
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
            reset_sweep_variables();
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
    Log_Write_AutoTuneSweep(curr_test.freq, curr_test.gain, curr_test.phase);
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
void AC_AutoTune_Heli::Log_Write_AutoTuneSweep(float freq, float gain, float phase)
{
    // @LoggerMessage: ATSH
    // @Description: Heli AutoTune Sweep packet
    // @Vehicles: Copter
    // @Field: TimeUS: Time since system startup
    // @Field: freq: current frequency
    // @Field: gain: current response gain
    // @Field: phase: current response phase
    AP::logger().WriteStreaming(
        "ATSH",
        "TimeUS,freq,gain,phase",
        "sE-d",
        "F000",
        "Qfff",
        AP_HAL::micros64(),
        freq,
        gain,
        phase);
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
    sweep.ph180 = {};
    sweep.ph270 = {};
    sweep.maxgain = {};

    sweep.progress = 0;
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
