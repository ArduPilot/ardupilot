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

#include "AC_AutoTune_Heli.h"

#define AUTOTUNE_HELI_TARGET_ANGLE_RLLPIT_CD     2000   // target roll/pitch angle during AUTOTUNE FeedForward rate test
#define AUTOTUNE_HELI_TARGET_RATE_RLLPIT_CDS     5000   // target roll/pitch rate during AUTOTUNE FeedForward rate test
#define AUTOTUNE_FFI_RATIO_FOR_TESTING     0.5f     // I is set 2x smaller than VFF during testing
#define AUTOTUNE_FFI_RATIO_FINAL           0.5f     // I is set 0.5x VFF after testing
#define AUTOTUNE_PI_RATIO_FINAL            1.0f     // I is set 1x P after testing
#define AUTOTUNE_YAW_PI_RATIO_FINAL        0.1f     // I is set 1x P after testing
#define AUTOTUNE_RD_STEP                  0.0005f     // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                  0.005f     // minimum increment when increasing/decreasing Rate P term
#define AUTOTUNE_SP_STEP                  0.05f     // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1f     // I is set 10x smaller than P during testing
#define AUTOTUNE_RD_MAX                  0.020f     // maximum Rate D value
#define AUTOTUNE_RLPF_MIN                  1.0f     // minimum Rate Yaw filter value
#define AUTOTUNE_RLPF_MAX                  20.0f     // maximum Rate Yaw filter value
#define AUTOTUNE_RP_MIN                   0.001f     // minimum Rate P value
#define AUTOTUNE_RP_MAX                   0.3f     // maximum Rate P value
#define AUTOTUNE_SP_MAX                    10.0f     // maximum Stab P value
#define AUTOTUNE_SP_MIN                    3.0f     // maximum Stab P value
#define AUTOTUNE_RFF_MAX                   0.5f     // maximum Stab P value
#define AUTOTUNE_RFF_MIN                   0.025f    // maximum Stab P value
#define AUTOTUNE_D_UP_DOWN_MARGIN          0.2f     // The margin below the target that we tune D in

#define AUTOTUNE_SEQ_BITMASK_VFF             1
#define AUTOTUNE_SEQ_BITMASK_RATE_D          2
#define AUTOTUNE_SEQ_BITMASK_ANGLE_P         4
#define AUTOTUNE_SEQ_BITMASK_MAX_GAIN        8

const AP_Param::GroupInfo AC_AutoTune_Heli::var_info[] = {

    // @Param: AXES
    // @DisplayName: Autotune axis bitmask
    // @Description: 1-byte bitmap of axes to autotune
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw
    // @User: Standard
    AP_GROUPINFO("AXES", 1, AC_AutoTune_Heli, axis_bitmask,  7),  // AUTOTUNE_AXIS_BITMASK_DEFAULT

    // @Param: SEQ
    // @DisplayName: AutoTune Sequence Bitmask
    // @Description: 2-byte bitmask to select what tuning should be performed.  Max gain automatically performed if Rate D is selected. Values: 7:All,1:VFF Only,2:Rate D Only,4:Angle P Only,8:Max Gain Only,3:VFF and Rate D (incl max gain),5:VFF and Angle P,13:VFF max gain and angle P
    // @Bitmask: 0:VFF,1:Rate D,2:Angle P,3:Max Gain Only
    // @User: Standard
    AP_GROUPINFO("SEQ", 2, AC_AutoTune_Heli, seq_bitmask,  5),

    // @Param: MIN_FRQ
    // @DisplayName: AutoTune minimum sweep frequency
    // @Description: Defines the start frequency for sweeps and dwells
    // @Range: 10 30
    // @User: Standard
    AP_GROUPINFO("MIN_FRQ", 3, AC_AutoTune_Heli, min_sweep_freq,  10.0f),

    // @Param: MAX_FRQ
    // @DisplayName: AutoTune maximum sweep frequency
    // @Description: Defines the end frequency for sweeps and dwells
    // @Range: 50 120
    // @User: Standard
    AP_GROUPINFO("MAX_FRQ", 4, AC_AutoTune_Heli, max_sweep_freq,  70.0f),

    // @Param: MAX_GN
    // @DisplayName: AutoTune maximum response gain
    // @Description: Defines the response gain (output/input) to tune
    // @Range: 1 2.5
    // @User: Standard
    AP_GROUPINFO("MAX_GN", 5, AC_AutoTune_Heli, max_resp_gain,  1.4f),

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
    if ((tune_type == RFF_UP) || (tune_type == RFF_DOWN)) {
        rate_ff_test_init();
        step_time_limit_ms = 10000;
    } else if (tune_type == MAX_GAINS || tune_type == RP_UP || tune_type == RD_UP) {
        // initialize start frequency and determine gain function when dwell test is used
        if (is_zero(start_freq)) {
            if (test_phase[12] > 160.0f && test_phase[12] < 180.0f && tune_type == RP_UP) {
                freq_cnt = 12;
                curr_test_freq = test_freq[12];
                start_freq = curr_test_freq;
                stop_freq = curr_test_freq;
            } else if (method == 1 && tune_type == RP_UP) {
                freq_cnt = 12;
                test_freq[12] = sweep.maxgain_freq;
                curr_test_freq = test_freq[12];
                start_freq = curr_test_freq;
                stop_freq = curr_test_freq;
            } else if (!is_zero(max_rate_p.freq) && tune_type == RP_UP) {
                freq_cnt = 12;
                test_freq[12] = max_rate_p.freq;
                curr_test_freq = test_freq[12];
                start_freq = curr_test_freq;
                stop_freq = curr_test_freq;
            } else if (tune_type == MAX_GAINS || tune_type == RD_UP) {
                start_freq = min_sweep_freq;
                stop_freq = max_sweep_freq;
                method = 0; //reset the method for rate D and rate P tuning.
            } else {
                test_freq[0] = 2.0f * 3.14159f * 2.0f;
                curr_test_freq = test_freq[0];
                start_freq = curr_test_freq;
                stop_freq = curr_test_freq;
            }
        }
        if (!is_equal(start_freq,stop_freq)) {
            // initialize determine_gain function whenever test is initialized
            freqresp_rate.init(AC_AutoTune_FreqResp::InputType::SWEEP);
            dwell_test_init(stop_freq);
        } else {
            // initialize determine_gain function whenever test is initialized
            freqresp_rate.init(AC_AutoTune_FreqResp::InputType::DWELL);
            dwell_test_init(start_freq);
        }
        if (!is_zero(start_freq)) {
            // 4 seconds is added to allow aircraft to achieve start attitude.  Then the time to conduct the dwells is added to it.
            step_time_limit_ms = (uint32_t)(4000 + (float)(AUTOTUNE_DWELL_CYCLES + 2) * 1000.0f * M_2PI / start_freq);
        }
    } else if (tune_type == SP_UP) {
        // initialize start frequency when dwell test is used
        if (is_zero(start_freq)) {
            test_freq[0] = 1.5f * 3.14159f * 2.0f;
            curr_test_freq = test_freq[0];
            test_accel_max = 0.0f;
            start_freq = min_sweep_freq;
            stop_freq = max_sweep_freq;
        }

        if (!is_equal(start_freq,stop_freq)) {
            // initialize determine gain function
            freqresp_angle.init(AC_AutoTune_FreqResp::InputType::SWEEP);
            angle_dwell_test_init(stop_freq);
        } else {
            // initialize determine gain function
            freqresp_angle.init(AC_AutoTune_FreqResp::InputType::DWELL);
            angle_dwell_test_init(start_freq);
        }

        // TODO add time limit for sweep test
        if (!is_zero(start_freq)) {
            // 1 seconds is added for a little buffer.  Then the time to conduct the dwells is added to it.
            step_time_limit_ms = (uint32_t)(2000 + (float)(AUTOTUNE_DWELL_CYCLES + 7) * 1000.0f * M_2PI / start_freq);
        }
    } else {

    }

    start_angles = Vector3f(roll_cd, pitch_cd, desired_yaw_cd);  // heli specific
}

// run tests for each tune type
void AC_AutoTune_Heli::test_run(AxisType test_axis, const float dir_sign)
{
    
    if (tune_type == SP_UP) {
        angle_dwell_test_run(start_freq, stop_freq, test_gain[freq_cnt], test_phase[freq_cnt]);
    } else if ((tune_type == RFF_UP) || (tune_type == RFF_DOWN)) {
        rate_ff_test_run(AUTOTUNE_HELI_TARGET_ANGLE_RLLPIT_CD, AUTOTUNE_HELI_TARGET_RATE_RLLPIT_CDS, dir_sign);
    } else if (tune_type == RP_UP || tune_type == RD_UP) {
        dwell_test_run(1, start_freq, stop_freq, test_gain[freq_cnt], test_phase[freq_cnt]);
    } else if (tune_type == MAX_GAINS) {
        dwell_test_run(0, start_freq, stop_freq, test_gain[freq_cnt], test_phase[freq_cnt]);
    } else if (tune_type == TUNE_COMPLETE) {
        return;
    } else {
        step = UPDATE_GAINS;
    }
}

// heli specific gcs announcements
void AC_AutoTune_Heli::do_gcs_announcements()
{
    const uint32_t now = AP_HAL::millis();
    if (now - announce_time < AUTOTUNE_ANNOUNCE_INTERVAL_MS) {
        return;
    }
    float tune_rp = 0.0f;
    float tune_rd = 0.0f;
    float tune_rff = 0.0f;
    float tune_sp = 0.0f;
    float tune_accel = 0.0f;
    char axis_char = '?';
    switch (axis) {
    case ROLL:
        tune_rp = tune_roll_rp;
        tune_rd = tune_roll_rd;
        tune_rff = tune_roll_rff;
        tune_sp = tune_roll_sp;
        tune_accel = tune_roll_accel;
        axis_char = 'R';
        break;
    case PITCH:
        tune_rp = tune_pitch_rp;
        tune_rd = tune_pitch_rd;
        tune_rff = tune_pitch_rff;
        tune_sp = tune_pitch_sp;
        tune_accel = tune_pitch_accel;
        axis_char = 'P';
        break;
    case YAW:
        tune_rp = tune_yaw_rp;
        tune_rd = tune_yaw_rd;
        tune_rff = tune_yaw_rff;
        tune_sp = tune_yaw_sp;
        tune_accel = tune_yaw_accel;
        axis_char = 'Y';
        break;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: (%c) %s", axis_char, type_string());
    send_step_string();
    switch (tune_type) {
    case RD_UP:
        //        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f ph=%f d=%f", (double)(test_freq[freq_cnt]), (double)(test_gain[freq_cnt]), (double)(test_phase[freq_cnt]), (double)tune_rd);
        break;
    case RD_DOWN:
    case RP_DOWN:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: p=%f d=%f", (double)tune_rp, (double)tune_rd);
        break;
    case RP_UP:
        //        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f p=%f", (double)(test_freq[freq_cnt]), (double)(test_gain[freq_cnt]), (double)tune_rp);
        break;
    case RFF_UP:
        if (!is_zero(test_rate_filt)) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: target=%f rotation=%f command=%f", (double)(test_tgt_rate_filt*57.3f), (double)(test_rate_filt*57.3f), (double)(test_command_filt));
        }
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: ff=%f", (double)tune_rff);
        break;
    case RFF_DOWN:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: ff=%f", (double)tune_rff);
        break;
    case SP_DOWN:
    case SP_UP:
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: p=%f accel=%f", (double)tune_sp, (double)tune_accel);
        break;
    case MAX_GAINS:
    case TUNE_COMPLETE:
        break;
    }
    //    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: success %u/%u", counter, AUTOTUNE_SUCCESS_COUNT);

    announce_time = now;
}

// load_test_gains - load the to-be-tested gains for a single axis
// called by control_attitude() just before it beings testing a gain (i.e. just before it twitches)
void AC_AutoTune_Heli::load_test_gains()
{
    switch (axis) {
    case ROLL:
        if (tune_type == SP_UP) {
            attitude_control->get_rate_roll_pid().kI(orig_roll_ri);
        } else {
            // freeze integrator to hold trim by making i term small during rate controller tuning
            attitude_control->get_rate_roll_pid().kI(0.01f * orig_roll_ri);
        }
        if (tune_type == MAX_GAINS && !is_zero(tune_roll_rff)) {
            attitude_control->get_rate_roll_pid().kP(0.0f);
            attitude_control->get_rate_roll_pid().kD(0.0f);
        } else {
            attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
            attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
        }
        attitude_control->get_rate_roll_pid().ff(tune_roll_rff);
        attitude_control->get_rate_roll_pid().filt_T_hz(orig_roll_fltt);
        attitude_control->get_rate_roll_pid().slew_limit(0.0f);
        attitude_control->get_angle_roll_p().kP(tune_roll_sp);
        break;
    case PITCH:
        if (tune_type == SP_UP) {
            attitude_control->get_rate_pitch_pid().kI(orig_pitch_ri);
        } else {
            // freeze integrator to hold trim by making i term small during rate controller tuning
            attitude_control->get_rate_pitch_pid().kI(0.01f * orig_pitch_ri);
        }
        if (tune_type == MAX_GAINS && !is_zero(tune_pitch_rff)) {
            attitude_control->get_rate_pitch_pid().kP(0.0f);
            attitude_control->get_rate_pitch_pid().kD(0.0f);
        } else {
            attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
            attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
        }
        attitude_control->get_rate_pitch_pid().ff(tune_pitch_rff);
        attitude_control->get_rate_pitch_pid().filt_T_hz(orig_pitch_fltt);
        attitude_control->get_rate_pitch_pid().slew_limit(0.0f);
        attitude_control->get_angle_pitch_p().kP(tune_pitch_sp);
        break;
    case YAW:
        // freeze integrator to hold trim by making i term small during rate controller tuning
        attitude_control->get_rate_yaw_pid().kP(tune_yaw_rp);
        attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*0.01f);
        attitude_control->get_rate_yaw_pid().kD(tune_yaw_rd);
        attitude_control->get_rate_yaw_pid().ff(tune_yaw_rff);
        attitude_control->get_rate_yaw_pid().filt_E_hz(tune_yaw_rLPF);
        attitude_control->get_rate_yaw_pid().filt_T_hz(orig_yaw_fltt);
        attitude_control->get_rate_yaw_pid().slew_limit(0.0f);
        attitude_control->get_angle_yaw_p().kP(tune_yaw_sp);
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
        // rate roll gains
        attitude_control->get_rate_roll_pid().kP(tune_roll_rp);
        attitude_control->get_rate_roll_pid().kI(tune_roll_rff*AUTOTUNE_FFI_RATIO_FINAL);
        attitude_control->get_rate_roll_pid().kD(tune_roll_rd);
        attitude_control->get_rate_roll_pid().ff(tune_roll_rff);
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

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) && pitch_enabled()) {
        // rate pitch gains
        attitude_control->get_rate_pitch_pid().kP(tune_pitch_rp);
        attitude_control->get_rate_pitch_pid().kI(tune_pitch_rff*AUTOTUNE_FFI_RATIO_FINAL);
        attitude_control->get_rate_pitch_pid().kD(tune_pitch_rd);
        attitude_control->get_rate_pitch_pid().ff(tune_pitch_rff);
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
        attitude_control->get_rate_yaw_pid().kD(tune_yaw_rd);
        attitude_control->get_rate_yaw_pid().ff(tune_yaw_rff);
        attitude_control->get_rate_yaw_pid().filt_T_hz(orig_yaw_fltt);
        attitude_control->get_rate_yaw_pid().slew_limit(orig_yaw_smax);
        attitude_control->get_rate_yaw_pid().filt_E_hz(orig_yaw_rLPF);
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
}

void AC_AutoTune_Heli::rate_ff_test_run(float max_angle_cd, float target_rate_cds, float dir_sign)
{
    float gyro_reading = 0.0f;
    float command_reading =0.0f;
    float tgt_rate_reading = 0.0f;
    const uint32_t now = AP_HAL::millis();

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

void AC_AutoTune_Heli::dwell_test_init(float filt_freq)
{
    rotation_rate_filt.reset(0);
    rotation_rate_filt.set_cutoff_frequency(filt_freq);
    command_filt.reset(0);
    command_filt.set_cutoff_frequency(filt_freq);
    target_rate_filt.reset(0);
    target_rate_filt.set_cutoff_frequency(filt_freq);
    filt_target_rate = 0.0f;
    dwell_start_time_ms = 0.0f;
    settle_time = 200;
    if (!is_equal(start_freq, stop_freq)) {
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

void AC_AutoTune_Heli::dwell_test_run(uint8_t freq_resp_input, float start_frq, float stop_frq, float &dwell_gain, float &dwell_phase)
{
    float gyro_reading = 0.0f;
    float command_reading = 0.0f;
    float tgt_rate_reading = 0.0f;
    float tgt_attitude = 2.5f * 0.01745f;
    const uint32_t now = AP_HAL::millis();
    float target_rate_cds;
    float sweep_time_ms = 23000;
    const float att_hold_gain = 4.5f;
    Vector3f attitude_cd;

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
        velocity_bf.y = -velocity_ned.x * ahrs_view->sin_yaw() + velocity_ned.y * ahrs_view->cos_yaw();
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
        filt_att_fdbk_from_velxy_cd.x += alpha * (-5730.0f * vel_hold_gain * velocity_bf.y - filt_att_fdbk_from_velxy_cd.x);
        filt_att_fdbk_from_velxy_cd.y += alpha * (5730.0f * vel_hold_gain * velocity_bf.x - filt_att_fdbk_from_velxy_cd.y);
    } else {
        target_rate_cds = 0.0f;
        settle_time--;
        dwell_start_time_ms = now;
        trim_command = command_out;
        filt_attitude_cd = attitude_cd;
        trim_attitude_cd = attitude_cd;
        filt_att_fdbk_from_velxy_cd = Vector2f(0.0f,0.0f);
    }

    // limit rate correction for position hold
    Vector3f trim_rate_cds;
    trim_rate_cds.x = att_hold_gain * ((trim_attitude_cd.x + filt_att_fdbk_from_velxy_cd.x) - filt_attitude_cd.x);
    trim_rate_cds.y = att_hold_gain * ((trim_attitude_cd.y + filt_att_fdbk_from_velxy_cd.y) - filt_attitude_cd.y);
    trim_rate_cds.z = att_hold_gain * wrap_180_cd(trim_attitude_cd.z - filt_attitude_cd.z);
    trim_rate_cds.x = constrain_float(trim_rate_cds.x, -15000.0f, 15000.0f);
    trim_rate_cds.y = constrain_float(trim_rate_cds.y, -15000.0f, 15000.0f);
    trim_rate_cds.z = constrain_float(trim_rate_cds.z, -15000.0f, 15000.0f);

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
            freqresp_rate.update_rate(filt_target_rate,rotation_rate, dwell_freq);
        } else {
            freqresp_rate.update_rate(command_out,rotation_rate, dwell_freq);
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

void AC_AutoTune_Heli::angle_dwell_test_init(float filt_freq)
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
    if (!is_equal(start_freq, stop_freq)) {
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

void AC_AutoTune_Heli::angle_dwell_test_run(float start_frq, float stop_frq, float &dwell_gain, float &dwell_phase)
{
    float gyro_reading = 0.0f;
    float command_reading = 0.0f;
    float tgt_rate_reading = 0.0f;
    float tgt_attitude = 5.0f * 0.01745f;
    const uint32_t now = AP_HAL::millis();
    float target_angle_cd;
    float sweep_time_ms = 23000;
    float dwell_freq = start_frq;

    const float alpha = calc_lowpass_alpha_dt(0.0025f, 0.2f * start_frq);

    // adjust target attitude based on input_tc so amplitude decrease with increased frequency is minimized
    const float freq_co = 1.0f / attitude_control->get_input_tc();
    const float added_ampl = (safe_sqrt(powf(dwell_freq,2.0) + powf(freq_co,2.0)) / freq_co) - 1.0f;
    tgt_attitude = constrain_float(0.08725f * (1.0f + 0.2f * added_ampl), 0.08725f, 0.5235f);

    float cycle_time_ms = 0;
    if (!is_zero(dwell_freq)) {
        cycle_time_ms = 1000.0f * 6.28f / dwell_freq;
    }

    // body frame calculation of velocity
    Vector3f velocity_ned, velocity_bf;
    if (ahrs_view->get_velocity_NED(velocity_ned)) {
        velocity_bf.x = velocity_ned.x * ahrs_view->cos_yaw() + velocity_ned.y * ahrs_view->sin_yaw();
        velocity_bf.y = -velocity_ned.x * ahrs_view->sin_yaw() + velocity_ned.y * ahrs_view->cos_yaw();
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
        filt_att_fdbk_from_velxy_cd.x += alpha * (-5730.0f * vel_hold_gain * velocity_bf.y - filt_att_fdbk_from_velxy_cd.x);
        filt_att_fdbk_from_velxy_cd.y += alpha * (5730.0f * vel_hold_gain * velocity_bf.x - filt_att_fdbk_from_velxy_cd.y);
        filt_att_fdbk_from_velxy_cd.x = constrain_float(filt_att_fdbk_from_velxy_cd.x, -2000.0f, 2000.0f);
        filt_att_fdbk_from_velxy_cd.y = constrain_float(filt_att_fdbk_from_velxy_cd.y, -2000.0f, 2000.0f);
    } else {
        target_angle_cd = 0.0f;
        trim_yaw_tgt_reading = (float)attitude_control->get_att_target_euler_cd().z;
        trim_yaw_heading_reading = (float)ahrs_view->yaw_sensor;
        settle_time--;
        dwell_start_time_ms = now;
        filt_att_fdbk_from_velxy_cd = Vector2f(0.0f,0.0f);
    }

    switch (axis) {
    case ROLL:
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_angle_cd + filt_att_fdbk_from_velxy_cd.x, filt_att_fdbk_from_velxy_cd.y, 0.0f);
        command_reading = motors->get_roll();
        tgt_rate_reading = ((float)attitude_control->get_att_target_euler_cd().x) / 5730.0f;
        gyro_reading = ((float)ahrs_view->roll_sensor) / 5730.0f;
        break;
    case PITCH:
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(filt_att_fdbk_from_velxy_cd.x, target_angle_cd + filt_att_fdbk_from_velxy_cd.y, 0.0f);
        command_reading = motors->get_pitch();
        tgt_rate_reading = ((float)attitude_control->get_att_target_euler_cd().y) / 5730.0f;
        gyro_reading = ((float)ahrs_view->pitch_sensor) / 5730.0f;
        break;
    case YAW:
        command_reading = motors->get_yaw();
        tgt_rate_reading = (wrap_180_cd((float)attitude_control->get_att_target_euler_cd().z - trim_yaw_tgt_reading)) / 5730.0f;
        gyro_reading = (wrap_180_cd((float)ahrs_view->yaw_sensor - trim_yaw_heading_reading)) / 5730.0f;
        attitude_control->input_euler_angle_roll_pitch_yaw(filt_att_fdbk_from_velxy_cd.x, filt_att_fdbk_from_velxy_cd.y, wrap_180_cd(trim_yaw_tgt_reading + target_angle_cd), false);
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
        freqresp_angle.update_angle(command_out, filt_target_rate, rotation_rate, dwell_freq);
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
                test_accel_max = freqresp_angle.get_accel_max();
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
float AC_AutoTune_Heli::waveform(float time, float time_record, float waveform_magnitude, float wMin, float wMax)
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

// update gains for the rate p up tune type
void AC_AutoTune_Heli::updating_rate_p_up_all(AxisType test_axis)
{
    float p_gain = 0.0f;

    switch (test_axis) {
    case ROLL:
        p_gain = tune_roll_rp;
        break;
    case PITCH:
        p_gain = tune_pitch_rp;
        break;
    case YAW:
        p_gain = tune_yaw_rp;
        break;
    }
    // announce results of dwell and update
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f ph=%f rate_p=%f", (double)(test_freq[freq_cnt]), (double)(test_gain[freq_cnt]), (double)(test_phase[freq_cnt]), (double)(p_gain));

    switch (test_axis) {
    case ROLL:
        updating_rate_p_up(tune_roll_rp, test_freq, test_gain, test_phase, freq_cnt, max_rate_p);
        break;
    case PITCH:
        updating_rate_p_up(tune_pitch_rp, test_freq, test_gain, test_phase, freq_cnt, max_rate_p);
        break;
    case YAW:
        updating_rate_p_up(tune_yaw_rp, test_freq, test_gain, test_phase, freq_cnt, max_rate_p);
        break;
    }
}

// update gains for the rate d up tune type
void AC_AutoTune_Heli::updating_rate_d_up_all(AxisType test_axis)
{
    float d_gain = 0.0f;

    switch (test_axis) {
    case ROLL:
        d_gain = tune_roll_rd;
        break;
    case PITCH:
        d_gain = tune_pitch_rd;
        break;
    case YAW:
        d_gain = tune_yaw_rd;
        break;
    }
    // announce results of dwell and update
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f ph=%f rate_d=%f", (double)(test_freq[freq_cnt]), (double)(test_gain[freq_cnt]), (double)(test_phase[freq_cnt]), (double)(d_gain));

    switch (test_axis) {
    case ROLL:
        updating_rate_d_up(tune_roll_rd, test_freq, test_gain, test_phase, freq_cnt, max_rate_d);
        break;
    case PITCH:
        updating_rate_d_up(tune_pitch_rd, test_freq, test_gain, test_phase, freq_cnt, max_rate_d);
        break;
    case YAW:
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
    // announce results of dwell and update
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f", (double)(test_freq[freq_cnt]), (double)(test_gain[freq_cnt]));
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: phase=%f accel=%f", (double)(test_phase[freq_cnt]), (double)(test_accel_max));

    switch (test_axis) {
    case ROLL:
        updating_angle_p_up(tune_roll_sp, test_freq, test_gain, test_phase, freq_cnt);
        break;
    case PITCH:
        updating_angle_p_up(tune_pitch_sp, test_freq, test_gain, test_phase, freq_cnt);
        break;
    case YAW:
        updating_angle_p_up(tune_yaw_sp, test_freq, test_gain, test_phase, freq_cnt);
        break;
    }
}

// update gains for the max gain tune type
void AC_AutoTune_Heli::updating_max_gains_all(AxisType test_axis)
{

    // announce results of dwell and update
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f ph=%f", (double)(test_freq[freq_cnt]), (double)(test_gain[freq_cnt]), (double)(test_phase[freq_cnt]));

    switch (test_axis) {
    case ROLL:
        updating_max_gains(&test_freq[0], &test_gain[0], &test_phase[0], freq_cnt, max_rate_p, max_rate_d, tune_roll_rp, tune_roll_rd);
        break;
    case PITCH:
        updating_max_gains(&test_freq[0], &test_gain[0], &test_phase[0], freq_cnt, max_rate_p, max_rate_d, tune_pitch_rp, tune_pitch_rd);
        break;
    case YAW:
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

// updating_rate_ff_up - adjust FF to ensure the target is reached
// FF is adjusted until rate requested is acheived
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
            ff_up_first_iter = true;
            first_dir_complete = false;
        }
    } else if (is_positive(rate_target * meas_rate) && fabsf(meas_rate) > 1.05f * fabsf(rate_target)) {
        tune_ff = 0.98f * tune_ff;
        if (tune_ff <= AUTOTUNE_RFF_MIN) {
            tune_ff = AUTOTUNE_RFF_MIN;
            counter = AUTOTUNE_SUCCESS_COUNT;
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            ff_up_first_iter = true;
            first_dir_complete = false;
        }
    } else if (is_positive(rate_target * meas_rate) && fabsf(meas_rate) < 0.95f * fabsf(rate_target)) {
        tune_ff = 1.02f * tune_ff;
        if (tune_ff >= AUTOTUNE_RFF_MAX) {
            tune_ff = AUTOTUNE_RFF_MAX;
            counter = AUTOTUNE_SUCCESS_COUNT;
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            ff_up_first_iter = true;
            first_dir_complete = false;
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
        if (frq_cnt == 0) {
            tune_p = max_gain_p.max_allowed * 0.10f;
            freq_cnt_max = 0;
        } else if (phase[frq_cnt] <= 180.0f && !is_zero(phase[frq_cnt])) {
            rp_prev_good_frq_cnt = frq_cnt;
        } else if (frq_cnt > 1 && phase[frq_cnt] > phase[frq_cnt-1] + 360.0f && !is_zero(phase[frq_cnt])) {
            if (phase[frq_cnt] - 360.0f < 180.0f) {
                rp_prev_good_frq_cnt = frq_cnt;
            }
        } else if (frq_cnt > 1 && phase[frq_cnt] > 300.0f && !is_zero(phase[frq_cnt])) {
            frq_cnt = 11;
        }
        frq_cnt++;
        if (frq_cnt == 12) {
            freq[frq_cnt] = freq[rp_prev_good_frq_cnt];
            curr_test_freq = freq[frq_cnt];
        } else {
            freq[frq_cnt] = freq[frq_cnt-1] + test_freq_incr;
            curr_test_freq = freq[frq_cnt];
        }
    } else if (is_equal(start_freq,stop_freq) && method == 2) {
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: cnt=%f freq=%f gain=%f phase=%f", (double)(frq_cnt), (double)(curr_test_freq),  (double)(gain[frq_cnt]),  (double)(phase[frq_cnt]));
        if (is_zero(tune_p)) {
            tune_p = 0.05f * max_gain_p.max_allowed;
        } else if (phase[frq_cnt] > 180.0f) {
            curr_test_freq = curr_test_freq - 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test_freq;
        } else if (phase[frq_cnt] < 160.0f) {
            curr_test_freq = curr_test_freq + 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test_freq;
        } else if (phase[frq_cnt] <= 180.0f && phase[frq_cnt] >= 160.0f) {
            if (gain[frq_cnt] < max_resp_gain && tune_p < 0.6f * max_gain_p.max_allowed) {
                tune_p += 0.05f * max_gain_p.max_allowed;
            } else {
                counter = AUTOTUNE_SUCCESS_COUNT;
                // reset curr_test_freq and frq_cnt for next test
                curr_test_freq = freq[0];
                frq_cnt = 0;
                tune_p -= 0.05f * max_gain_p.max_allowed;
                tune_p = constrain_float(tune_p,0.0f,0.6f * max_gain_p.max_allowed);
//                rp_prev_gain = 0.0f;
            }
        }
//        rp_prev_gain = gain[frq_cnt];
    } else if (is_equal(start_freq,stop_freq) && method == 1) {
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: cnt=%f freq=%f gain=%f phase=%f", (double)(frq_cnt), (double)(curr_test_freq),  (double)(gain[frq_cnt]),  (double)(phase[frq_cnt]));

        if (is_zero(tune_p)) {
            tune_p = 0.05f * max_gain_p.max_allowed;
            rp_prev_gain = gain[frq_cnt];
        } else if ((gain[frq_cnt] < rp_prev_gain || is_zero(rp_prev_gain)) && tune_p < 0.6f * max_gain_p.max_allowed) {
            tune_p += 0.05f * max_gain_p.max_allowed;
            rp_prev_gain = gain[frq_cnt];
        } else {
            counter = AUTOTUNE_SUCCESS_COUNT;
            // reset curr_test_freq and frq_cnt for next test
            curr_test_freq = freq[0];
            frq_cnt = 0;
            rp_prev_gain = 0.0f;
            tune_p -= 0.05f * max_gain_p.max_allowed;
            tune_p = constrain_float(tune_p,0.0f,0.6f * max_gain_p.max_allowed);
        }

    }

    if (counter == AUTOTUNE_SUCCESS_COUNT) {
        start_freq = 0.0f;  //initializes next test that uses dwell test
    } else {
        start_freq = curr_test_freq;
        stop_freq = curr_test_freq;
    }
}

// updating_rate_d_up - uses maximum allowable gain determined from max_gain test to determine rate d gain where the response gain is at a minimum
void AC_AutoTune_Heli::updating_rate_d_up(float &tune_d, float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_d)
{
    float test_freq_incr = 0.25f * 3.14159f * 2.0f;

    if (!is_equal(start_freq,stop_freq)) {
        frq_cnt = 12;
        if (sweep.maxgain_freq > sweep.ph180_freq) {
//            freq[frq_cnt] = sweep.maxgain_freq - 0.5f * test_freq_incr;
            freq[frq_cnt] = sweep.ph180_freq - 0.5f * test_freq_incr;
            freq_cnt_max = frq_cnt;
//            method = 1;
            method = 2;
        } else if (!is_zero(sweep.ph180_freq)) {
            freq[frq_cnt] = sweep.ph180_freq - 0.5f * test_freq_incr;
            // using 180 phase as max gain to start
            freq_cnt_max = frq_cnt;
            method = 2;
        } else {
            freq[frq_cnt] = 4.0f * M_PI;
            method = 0;
        }
        curr_test_freq = freq[frq_cnt];
    }
    if (frq_cnt < 12 && is_equal(start_freq,stop_freq)) {
        if (frq_cnt == 0) {
            tune_d = max_gain_d.max_allowed * 0.25f;
            freq_cnt_max = 0;
        } else if (phase[frq_cnt] <= 180.0f && !is_zero(phase[frq_cnt])) {
            rd_prev_good_frq_cnt = frq_cnt;
        } else if (frq_cnt > 1 && phase[frq_cnt] > phase[frq_cnt-1] + 360.0f && !is_zero(phase[frq_cnt])) {
            if (phase[frq_cnt] - 360.0f < 180.0f) {
                rd_prev_good_frq_cnt = frq_cnt;
            }
        } else if (frq_cnt > 1 && phase[frq_cnt] > 300.0f && !is_zero(phase[frq_cnt])) {
            frq_cnt = 11;
        }
        frq_cnt++;
        if (frq_cnt == 12) {
            freq[frq_cnt] = freq[rd_prev_good_frq_cnt];
            curr_test_freq = freq[frq_cnt];
            method = 2;
        } else {
            freq[frq_cnt] = freq[frq_cnt-1] + test_freq_incr;
            curr_test_freq = freq[frq_cnt];
        }
    } else if (is_equal(start_freq,stop_freq) && method == 2) {
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: cnt=%f freq=%f gain=%f phase=%f", (double)(frq_cnt), (double)(curr_test_freq),  (double)(gain[frq_cnt]),  (double)(phase[frq_cnt]));
        if (is_zero(tune_d)) {
            tune_d = 0.05f * max_gain_d.max_allowed;
            rd_prev_gain = gain[frq_cnt];
        } else if (phase[frq_cnt] > 180.0f) {
            curr_test_freq = curr_test_freq - 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test_freq;
        } else if (phase[frq_cnt] < 160.0f) {
            curr_test_freq = curr_test_freq + 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test_freq;
        } else if (phase[frq_cnt] <= 180.0f && phase[frq_cnt] >= 160.0f) {
            if ((gain[frq_cnt] < rd_prev_gain || is_zero(rd_prev_gain)) && tune_d < 0.6f * max_gain_d.max_allowed) {
                tune_d += 0.05f * max_gain_d.max_allowed;
                rd_prev_gain = gain[frq_cnt];
            } else {
                counter = AUTOTUNE_SUCCESS_COUNT;
                // reset curr_test_freq and frq_cnt for next test
                curr_test_freq = freq[0];
                frq_cnt = 0;
                rd_prev_gain = 0.0f;
                tune_d -= 0.05f * max_gain_d.max_allowed;
                tune_d = constrain_float(tune_d,0.0f,0.6f * max_gain_d.max_allowed);
            }
        }
    } else if (is_equal(start_freq,stop_freq) && method == 1) {
        gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: cnt=%f freq=%f gain=%f phase=%f", (double)(frq_cnt), (double)(curr_test_freq),  (double)(gain[frq_cnt]),  (double)(phase[frq_cnt]));
        if (is_zero(tune_d)) {
            tune_d = 0.05f * max_gain_d.max_allowed;
            rd_prev_gain = gain[frq_cnt];
        } else if ((gain[frq_cnt] < rd_prev_gain || is_zero(rd_prev_gain)) && tune_d < 0.6f * max_gain_d.max_allowed) {
            tune_d += 0.05f * max_gain_d.max_allowed;
            rd_prev_gain = gain[frq_cnt];
        } else {
            counter = AUTOTUNE_SUCCESS_COUNT;
            // reset curr_test_freq and frq_cnt for next test
            curr_test_freq = freq[0];
            frq_cnt = 0;
            rd_prev_gain = 0.0f;
            tune_d -= 0.05f * max_gain_d.max_allowed;
            tune_d = constrain_float(tune_d,0.0f,0.6f * max_gain_d.max_allowed);
        }
    }
    if (counter == AUTOTUNE_SUCCESS_COUNT) {
        start_freq = 0.0f;  //initializes next test that uses dwell test
    } else {
        start_freq = curr_test_freq;
        stop_freq = curr_test_freq;
    }
}

// updating_angle_p_up - determines maximum angle p gain for pitch and roll
void AC_AutoTune_Heli::updating_angle_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt)
{
    float test_freq_incr = 0.5f * 3.14159f * 2.0f;
    float gain_incr = 0.5f;

    if (!is_equal(start_freq,stop_freq)) {
        frq_cnt = 12;
        if (!is_zero(sweep.ph180_freq)) {
//            freq[frq_cnt] = sweep.ph180_freq - 0.5f * test_freq_incr;
            freq[frq_cnt] = sweep.maxgain_freq - 0.5f * test_freq_incr;
            // using 180 phase as max gain to start
            freq_cnt_max = frq_cnt;
        } else {
            freq[frq_cnt] = 4.0f * M_PI;
        }
        curr_test_freq = freq[frq_cnt];
    }
    if (freq_cnt < 12 && is_equal(start_freq,stop_freq)) {
        if (freq_cnt == 0) {
            freq_cnt_max = 0;
        } else if (gain[freq_cnt] > max_resp_gain && tune_p > AUTOTUNE_SP_MIN) {
            // exceeded max response gain already, reduce tuning gain to remain under max response gain
            tune_p -= gain_incr;
            // force counter to stay on frequency
            freq_cnt -= 1;
        } else if (gain[freq_cnt] > max_resp_gain && tune_p <= AUTOTUNE_SP_MIN) {
            // exceeded max response gain at the minimum allowable tuning gain. terminate testing.
            tune_p = AUTOTUNE_SP_MIN;
            counter = AUTOTUNE_SUCCESS_COUNT;
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            curr_test_freq = freq[0];
            freq_cnt = 0;
        } else if (gain[freq_cnt] > gain[freq_cnt_max]) {
            freq_cnt_max = freq_cnt;
            phase_max = phase[freq_cnt];
            sp_prev_gain = gain[freq_cnt];
        } else if (gain[freq_cnt] > 0.0f && gain[freq_cnt] < 0.5f) {
            // must be past peak, continue on to determine angle p
            freq_cnt = 11;
        }
        freq_cnt++;
        if (freq_cnt == 12) {
            freq[freq_cnt] = freq[freq_cnt_max];
            curr_test_freq = freq[freq_cnt];
        } else {
            freq[freq_cnt] = freq[freq_cnt-1] + test_freq_incr;
            curr_test_freq = freq[freq_cnt];
        }
    }

    // once finished with sweep of frequencies, cnt = 12 is used to then tune for max response gain
    if (freq_cnt >= 12 && is_equal(start_freq,stop_freq)) {
        if (gain[freq_cnt] < max_resp_gain && tune_p < AUTOTUNE_SP_MAX && !find_peak) {
            // keep increasing tuning gain unless phase changes or max response gain is acheived
            if (phase[freq_cnt]-phase_max > 20.0f && phase[freq_cnt] < 210.0f) {
                freq[freq_cnt] += 0.5 * test_freq_incr;
                find_peak = true;
            } else {
                tune_p += gain_incr;
                freq[freq_cnt] = freq[freq_cnt_max];
                if (tune_p >= AUTOTUNE_SP_MAX) {
                    tune_p = AUTOTUNE_SP_MAX;
                    counter = AUTOTUNE_SUCCESS_COUNT;
                    AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
                    curr_test_freq = freq[0];
                    freq_cnt = 0;
                }
            }
            curr_test_freq = freq[freq_cnt];
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
            curr_test_freq = freq[freq_cnt];
            sp_prev_gain = gain[freq_cnt];
        } else {
            // adjust tuning gain so max response gain is not exceeded
            if (sp_prev_gain < max_resp_gain && gain[freq_cnt] > max_resp_gain) {
                float adj_factor = (max_resp_gain - gain[freq_cnt]) / (gain[freq_cnt] - sp_prev_gain);
                tune_p = tune_p + gain_incr * adj_factor; 
            }
            counter = AUTOTUNE_SUCCESS_COUNT;
            // reset curr_test_freq and freq_cnt for next test
            curr_test_freq = freq[0];
            freq_cnt = 0;
        }
    }
    if (counter == AUTOTUNE_SUCCESS_COUNT) {
        start_freq = 0.0f;  //initializes next test that uses dwell test
    } else {
        start_freq = curr_test_freq;
        stop_freq = curr_test_freq;
    }
}

// updating_angle_p_up_yaw - determines maximum angle p gain for yaw
void AC_AutoTune_Heli::updating_angle_p_up_yaw(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt)
{
    float test_freq_incr = 0.5f * 3.14159f * 2.0f;

    if (frq_cnt < 12) {
        if (frq_cnt == 0) {
            freq_cnt_max = 0;
        } else if (phase[frq_cnt] <= 180.0f && !is_zero(phase[frq_cnt])) {
            sp_prev_good_frq_cnt = frq_cnt;
        } else if (frq_cnt > 1 && phase[frq_cnt] > phase[frq_cnt-1] + 360.0f && !is_zero(phase[frq_cnt])) {
            if (phase[frq_cnt] - 360.0f < 180.0f) {
                sp_prev_good_frq_cnt = frq_cnt;
            }
        } else if (frq_cnt > 1 && phase[frq_cnt] > 300.0f && !is_zero(phase[frq_cnt])) {
            frq_cnt = 11;
        }
        frq_cnt++;
        if (frq_cnt == 12) {
            freq[frq_cnt] = freq[sp_prev_good_frq_cnt];
            curr_test_freq = freq[frq_cnt];
        } else {
            freq[frq_cnt] = freq[frq_cnt-1] + test_freq_incr;
            curr_test_freq = freq[frq_cnt];
        }
    }

    // once finished with sweep of frequencies, cnt = 12 is used to then tune for max response gain
    if (freq_cnt >= 12) {
        if (gain[frq_cnt] < max_resp_gain && phase[frq_cnt] <= 180.0f && phase[frq_cnt] >= 160.0f && tune_p < AUTOTUNE_SP_MAX) {
            tune_p += 0.5f;
            if (tune_p >= AUTOTUNE_SP_MAX) {
                tune_p = AUTOTUNE_SP_MAX;
                counter = AUTOTUNE_SUCCESS_COUNT;
                AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
                curr_test_freq = freq[0];
                freq_cnt = 0;
            }
        } else if (gain[frq_cnt] < max_resp_gain && phase[frq_cnt] > 180.0f) {
            curr_test_freq = curr_test_freq - 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test_freq;
        } else if (gain[frq_cnt] < max_resp_gain && phase[frq_cnt] < 160.0f) {
            curr_test_freq = curr_test_freq + 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test_freq;
        } else {
            counter = AUTOTUNE_SUCCESS_COUNT;
            // reset curr_test_freq and frq_cnt for next test
            curr_test_freq = freq[0];
            frq_cnt = 0;
        }

        // guard against frequency getting too high or too low
        if (curr_test_freq > 50.24f || curr_test_freq < 3.14f) {
            counter = AUTOTUNE_SUCCESS_COUNT;
            AP::logger().Write_Event(LogEvent::AUTOTUNE_REACHED_LIMIT);
            curr_test_freq = freq[0];
            freq_cnt = 0;
        }

    }
    if (counter == AUTOTUNE_SUCCESS_COUNT) {
        start_freq = 0.0f;  //initializes next test that uses dwell test
    } else {
        start_freq = curr_test_freq;
        stop_freq = curr_test_freq;
    }
}

// updating_max_gains: use dwells at increasing frequency to determine gain at which instability will occur
void AC_AutoTune_Heli::updating_max_gains(float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_p, max_gain_data &max_gain_d, float &tune_p, float &tune_d)
{
    float test_freq_incr = 1.0f * M_PI * 2.0f;

    if (!is_equal(start_freq,stop_freq)) {
        frq_cnt = 2;
        if (!is_zero(sweep.ph180_freq)) {
            freq[frq_cnt] = sweep.ph180_freq - 0.5f * test_freq_incr;
        } else {
            freq[frq_cnt] = 4.0f * M_PI;
        }
        curr_test_freq = freq[frq_cnt];
        start_freq = curr_test_freq;
        stop_freq = curr_test_freq;

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

            if (!is_zero(sweep.ph270_freq)) {
                // set freq cnt back to reinitialize process
                frq_cnt = 1;  // set to 1 because it will be incremented
                // set frequency back at least one test_freq_incr as it will be added
                freq[1] = sweep.ph270_freq - 1.5f * test_freq_incr;
            }

            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Max rate P freq=%f gain=%f ph=%f rate_p=%f", (double)(max_gain_p.freq), (double)(max_gain_p.gain), (double)(max_gain_p.phase), (double)(max_gain_p.max_allowed));
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
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Max Rate D freq=%f gain=%f ph=%f rate_d=%f", (double)(max_gain_d.freq), (double)(max_gain_d.gain), (double)(max_gain_d.phase), (double)(max_gain_d.max_allowed));
        }
        // stop progression in frequency.
        if ((frq_cnt > 1 && phase[frq_cnt] > 330.0f && !is_zero(phase[frq_cnt])) || found_max_d) {
            frq_cnt = 11;
        }
        frq_cnt++;
        if (frq_cnt == 12) {
            counter = AUTOTUNE_SUCCESS_COUNT;
            // reset variables for next test
            curr_test_freq = freq[0];
            frq_cnt = 0;
            found_max_p = false;
            found_max_d = false;
            find_middle = false;
//            tune_p = 0.35f * max_gain_p.max_allowed;
//            tune_d = 0.25f * max_gain_d.max_allowed;
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
            curr_test_freq = freq[frq_cnt];
            start_freq = curr_test_freq;
            stop_freq = curr_test_freq;
        }
    }
}

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
            Log_Write_AutoTune(axis, tune_type, test_freq[freq_cnt], test_gain[freq_cnt], test_phase[freq_cnt], tune_yaw_rff, tune_yaw_rp, tune_yaw_rd, tune_yaw_sp, test_accel_max);
            break;
        }
//    }
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
    Log_Write_AutoTuneSweep(curr_test_freq, curr_test_gain, curr_test_phase);
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

// get intra test rate I gain for the specified axis
float AC_AutoTune_Heli::get_intra_test_ri(AxisType test_axis)
{
    float ret = 0.0f;
    switch (test_axis) {
    case ROLL:
        ret = orig_roll_rff * AUTOTUNE_FFI_RATIO_FOR_TESTING;
        break;
    case PITCH:
        ret = orig_pitch_rff * AUTOTUNE_FFI_RATIO_FOR_TESTING;
        break;
    case YAW:
        ret = orig_yaw_rp*AUTOTUNE_PI_RATIO_FOR_TESTING;
        break;
    }
    return ret;
}

// get tuned rate I gain for the specified axis
float AC_AutoTune_Heli::get_tuned_ri(AxisType test_axis)
{
    float ret = 0.0f;
    switch (test_axis) {
    case ROLL:
        ret = tune_roll_rff*AUTOTUNE_FFI_RATIO_FINAL;
        break;
    case PITCH:
        ret = tune_pitch_rff*AUTOTUNE_FFI_RATIO_FINAL;
        break;
    case YAW:
        ret = tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL;
        break;
    }
    return ret;
}

// get minimum rate P (for any axis)
float AC_AutoTune_Heli::get_rp_min() const
{
    return AUTOTUNE_RP_MIN;
}

// get minimum angle P (for any axis)
float AC_AutoTune_Heli::get_sp_min() const
{
    return AUTOTUNE_SP_MIN;
}

// get minimum rate Yaw filter value
float AC_AutoTune_Heli::get_yaw_rate_filt_min() const
{
    return AUTOTUNE_RLPF_MIN;
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
    tune_seq[seq_cnt] = TUNE_COMPLETE;

}
