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
#define AUTOTUNE_RP_MIN                   0.01f     // minimum Rate P value
#define AUTOTUNE_RP_MAX                    2.0f     // maximum Rate P value
#define AUTOTUNE_SP_MAX                   10.0f     // maximum Stab P value
#define AUTOTUNE_SP_MIN                    0.5f     // maximum Stab P value
#define AUTOTUNE_D_UP_DOWN_MARGIN          0.2f     // The margin below the target that we tune D in

#include "AC_AutoTune_Heli.h"

void AC_AutoTune_Heli::test_init()
{

    if ((tune_type == RFF_UP) || (tune_type == RFF_DOWN)) {
        rate_ff_test_init();
        step_time_limit_ms = 10000;
    } else if (tune_type == MAX_GAINS || tune_type == RP_UP || tune_type == RD_UP) {

        // initialize start frequency and determine gain function when dwell test is used
        if (freq_cnt == 0) {
            test_freq[0] = 2.0f * 3.14159f * 2.0f;
            curr_test_freq = test_freq[0];
            // reset determine_gain function for first use in the event autotune is restarted
            determine_gain(0.0f, 0.0f, curr_test_freq, test_gain[freq_cnt], test_phase[freq_cnt], dwell_complete, true);
        }
        dwell_test_init(curr_test_freq);
        if (!is_zero(curr_test_freq)) {
            // 4 seconds is added to allow aircraft to achieve start attitude.  Then the time to conduct the dwells is added to it.
            step_time_limit_ms = (uint32_t)(4000 + (float)(AUTOTUNE_DWELL_CYCLES + 2) * 1000.0f * 6.28f / curr_test_freq);
        }
    } else if (tune_type == SP_UP) {
        // initialize start frequency and determine gain function when dwell test is used
        if (freq_cnt == 0) {
            test_freq[0] = 0.5f * 3.14159f * 2.0f;
            curr_test_freq = test_freq[0];
            // reset determine_gain function for first use in the event autotune is restarted
            determine_gain(0.0f, 0.0f, curr_test_freq, test_gain[freq_cnt], test_phase[freq_cnt], dwell_complete, true);
        }
        angle_dwell_test_init(curr_test_freq);
        if (!is_zero(curr_test_freq)) {
            // 1 seconds is added for a little buffer.  Then the time to conduct the dwells is added to it.
            step_time_limit_ms = (uint32_t)(2000 + (float)(AUTOTUNE_DWELL_CYCLES + 2) * 1000.0f * 6.28f / curr_test_freq);
        }
    } else {

    }
    start_angles = Vector3f(roll_cd, pitch_cd, desired_yaw_cd);  // heli specific
}

void AC_AutoTune_Heli::test_run(AxisType test_axis, const float dir_sign)
{

    if (tune_type == SP_UP) {
        angle_dwell_test_run(curr_test_freq, test_gain[freq_cnt], test_phase[freq_cnt]);
    } else if ((tune_type == RFF_UP) || (tune_type == RFF_DOWN)) {
        rate_ff_test_run(AUTOTUNE_HELI_TARGET_ANGLE_RLLPIT_CD, AUTOTUNE_HELI_TARGET_RATE_RLLPIT_CDS);
    } else if (tune_type == RP_UP || tune_type == RD_UP) {
        dwell_test_run(1, curr_test_freq, test_gain[freq_cnt], test_phase[freq_cnt]);
    } else if (tune_type == MAX_GAINS) {
        dwell_test_run(0, curr_test_freq, test_gain[freq_cnt], test_phase[freq_cnt]);
    } else {
        step = UPDATE_GAINS;
    }
}

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
    AC_AutoTune::load_test_gains();

    switch (axis) {
    case ROLL:
        if (tune_type == SP_UP) {
            attitude_control->get_rate_roll_pid().kI(orig_roll_ri);
        } else {
            attitude_control->get_rate_roll_pid().kI(0.0f);
        }
        attitude_control->get_rate_roll_pid().ff(tune_roll_rff);
        attitude_control->get_rate_roll_pid().filt_T_hz(orig_roll_fltt);
        attitude_control->get_rate_roll_pid().slew_limit(0.0f);
        break;
    case PITCH:
        if (tune_type == SP_UP) {
            attitude_control->get_rate_pitch_pid().kI(orig_pitch_ri);
        } else {
            attitude_control->get_rate_pitch_pid().kI(0.0f);
        }
        attitude_control->get_rate_pitch_pid().ff(tune_pitch_rff);
        attitude_control->get_rate_pitch_pid().filt_T_hz(orig_pitch_fltt);
        attitude_control->get_rate_pitch_pid().slew_limit(0.0f);
        break;
    case YAW:
        attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*0.01f);
        attitude_control->get_rate_yaw_pid().kD(tune_yaw_rd);
        attitude_control->get_rate_yaw_pid().ff(tune_yaw_rff);
        attitude_control->get_rate_yaw_pid().filt_T_hz(orig_yaw_fltt);
        attitude_control->get_rate_yaw_pid().slew_limit(0.0f);
        break;
    }
}

// save_tuning_gains - save the final tuned gains for each axis
// save discovered gains to eeprom if autotuner is enabled (i.e. switch is in the high position)
void AC_AutoTune_Heli::save_tuning_gains()
{

    AC_AutoTune::save_tuning_gains();

    // sanity check the rate P values
    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_ROLL) && roll_enabled() && !is_zero(tune_roll_rp)) {
        // rate roll gains
        attitude_control->get_rate_roll_pid().ff(tune_roll_rff);
        attitude_control->get_rate_roll_pid().filt_T_hz(orig_roll_fltt);
        attitude_control->get_rate_roll_pid().slew_limit(orig_roll_smax);
        attitude_control->get_rate_roll_pid().kI(tune_roll_rff*AUTOTUNE_FFI_RATIO_FINAL);
        attitude_control->get_rate_roll_pid().save_gains();

        // resave pids to originals in case the autotune is run again
        orig_roll_rff = attitude_control->get_rate_roll_pid().ff();
        orig_roll_ri = attitude_control->get_rate_roll_pid().kI();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_PITCH) && pitch_enabled() && !is_zero(tune_pitch_rp)) {
        // rate pitch gains
        attitude_control->get_rate_pitch_pid().ff(tune_pitch_rff);
        attitude_control->get_rate_pitch_pid().filt_T_hz(orig_pitch_fltt);
        attitude_control->get_rate_pitch_pid().slew_limit(orig_pitch_smax);
        attitude_control->get_rate_pitch_pid().kI(tune_pitch_rff*AUTOTUNE_FFI_RATIO_FINAL);
        attitude_control->get_rate_pitch_pid().save_gains();

        // resave pids to originals in case the autotune is run again
        orig_pitch_rff = attitude_control->get_rate_pitch_pid().ff();
        orig_pitch_ri = attitude_control->get_rate_pitch_pid().kI();
    }

    if ((axes_completed & AUTOTUNE_AXIS_BITMASK_YAW) && yaw_enabled() && !is_zero(tune_yaw_rp)) {
        // rate yaw gains
        attitude_control->get_rate_yaw_pid().kD(tune_yaw_rd);
        attitude_control->get_rate_yaw_pid().ff(tune_yaw_rff);
        attitude_control->get_rate_yaw_pid().filt_T_hz(orig_yaw_fltt);
        attitude_control->get_rate_yaw_pid().slew_limit(orig_yaw_smax);
        attitude_control->get_rate_yaw_pid().filt_E_hz(orig_yaw_rLPF);
        attitude_control->get_rate_yaw_pid().kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
        attitude_control->get_rate_yaw_pid().save_gains();

        // resave pids to originals in case the autotune is run again
        orig_yaw_rd = attitude_control->get_rate_yaw_pid().kD();
        orig_yaw_rLPF = attitude_control->get_rate_yaw_pid().filt_E_hz();
        orig_yaw_rff = attitude_control->get_rate_yaw_pid().ff();
        orig_yaw_ri = attitude_control->get_rate_yaw_pid().kI();
    }

    // update GCS and log save gains event
    update_gcs(AUTOTUNE_MESSAGE_SAVED_GAINS);
    AP::logger().Write_Event(LogEvent::AUTOTUNE_SAVEDGAINS);

    reset();
}

// generic method used to update gains for the rate p up tune type
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
        updating_rate_d_up(tune_roll_rp, test_freq, test_gain, test_phase, freq_cnt, max_rate_p);
        break;
    case PITCH:
        updating_rate_d_up(tune_pitch_rp, test_freq, test_gain, test_phase, freq_cnt, max_rate_p);
        break;
    case YAW:
        updating_rate_d_up(tune_yaw_rp, test_freq, test_gain, test_phase, freq_cnt, max_rate_p);
        break;
    }
}

// generic method used to update gains for the rate d up tune type
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

// generic method used to update gains for the rate ff up tune type
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
        break;
    }
}

// generic method used to update gains for the angle p up tune type
void AC_AutoTune_Heli::updating_angle_p_up_all(AxisType test_axis)
{
    // announce results of dwell and update
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f ph=%f", (double)(test_freq[freq_cnt]), (double)(test_gain[freq_cnt]), (double)(test_phase[freq_cnt]));

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

// generic method used to update gains for the max gain tune type
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
        tune_ff = constrain_float(tune_ff, 0.01, 1);
        ff_up_first_iter = false;
    } else if (is_positive(rate_target * meas_rate) && fabsf(meas_rate) < 1.05f * fabsf(rate_target) &&
               fabsf(meas_rate) > 0.95f * fabsf(rate_target)) {
        counter = AUTOTUNE_SUCCESS_COUNT;
        tune_ff = 0.75f * tune_ff;
    } else if (is_positive(rate_target * meas_rate) && fabsf(meas_rate) > 1.05f * fabsf(rate_target)) {
        tune_ff = 0.98f * tune_ff;
    } else if (is_positive(rate_target * meas_rate) && fabsf(meas_rate) < 0.95f * fabsf(rate_target)) {
        tune_ff = 1.02f * tune_ff;
    } else {
        if (!is_zero(meas_rate)) {
            tune_ff = 5730.0f * meas_command / meas_rate;
        }
        tune_ff = constrain_float(tune_ff, 0.01, 1);
    }
}

void AC_AutoTune_Heli::updating_rate_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt, float gain_incr, float max_gain)
{
    float test_freq_incr = 0.5f * 3.14159f * 2.0f;

    if (freq_cnt < 12) {
        if (freq_cnt == 0) {
            freq_cnt_max = 0;
        } else if (gain[freq_cnt] > gain[freq_cnt_max]) {
            freq_cnt_max = freq_cnt;
        }
        freq_cnt++;
        freq[freq_cnt] = freq[freq_cnt-1] + test_freq_incr;
        curr_test_freq = freq[freq_cnt];
    } else {
        if (gain[freq_cnt] < max_gain) {
            tune_p += gain_incr;
            curr_test_freq = freq[freq_cnt_max];
            freq[freq_cnt] = curr_test_freq;
        } else {
            counter = AUTOTUNE_SUCCESS_COUNT;
            // reset curr_test_freq and freq_cnt for next test
            curr_test_freq = freq[0];
            freq_cnt = 0;
        }
    }
    // reset determine_gain function
    determine_gain(0.0f, 0.0f, curr_test_freq, gain[freq_cnt], phase[freq_cnt], dwell_complete, true);

}

void AC_AutoTune_Heli::updating_rate_d_up(float &tune_d, float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_d)
{
    float test_freq_incr = 0.5f * 3.14159f * 2.0f;
    static uint8_t prev_good_frq_cnt;
    float max_gain = 1.2f;

    if (frq_cnt < 12) {
        if (frq_cnt == 0) {
            tune_d = max_gain_d.max_allowed * 0.5f;
            freq_cnt_max = 0;
        } else if (phase[frq_cnt] <= 180.0f && !is_zero(phase[frq_cnt])) {
            prev_good_frq_cnt = frq_cnt;
        } else if (frq_cnt > 1 && phase[frq_cnt] > phase[frq_cnt-1] + 360.0f && !is_zero(phase[frq_cnt])) {
            if (phase[frq_cnt] - 360.0f < 180.0f) {
                prev_good_frq_cnt = frq_cnt;
            }
        } else if (frq_cnt > 1 && phase[frq_cnt] > 300.0f && !is_zero(phase[frq_cnt])) {
            frq_cnt = 11;
        }
        frq_cnt++;
        if (frq_cnt == 12) {
            freq[frq_cnt] = freq[prev_good_frq_cnt];
            curr_test_freq = freq[frq_cnt];
        } else {
            freq[frq_cnt] = freq[frq_cnt-1] + test_freq_incr;
            curr_test_freq = freq[frq_cnt];
        }
    } else {
        /*        if (!is_zero(phase[prev_good_frq_cnt + 1])) {
                    freq_cnt_max = prev_good_frq_cnt + 2;
                } else {
                    freq_cnt_max = prev_good_frq_cnt + 1;
                }
                float phase_freq = (180.0f - phase[prev_good_frq_cnt]) / (phase[freq_cnt_max] - phase[prev_good_frq_cnt]);
                phase_freq = freq[prev_good_frq_cnt] + phase_freq * (freq[freq_cnt_max] - freq[prev_good_frq_cnt]); */
        if (gain[frq_cnt] < max_gain && phase[frq_cnt] <= 180.0f && phase[frq_cnt] >= 160.0f && tune_d < 0.8f * max_gain_d.max_allowed) {
            tune_d += 0.1f * max_gain_d.max_allowed;
        } else if (gain[frq_cnt] < max_gain && phase[frq_cnt] > 180.0f) {
            curr_test_freq = curr_test_freq - 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test_freq;
        } else if (gain[frq_cnt] < max_gain && phase[frq_cnt] < 160.0f) {
            curr_test_freq = curr_test_freq + 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test_freq;
        } else if (gain[frq_cnt] >= max_gain || tune_d > 0.8f * max_gain_d.max_allowed) {
            counter = AUTOTUNE_SUCCESS_COUNT;
            tune_d = 0.5f * tune_d;
            // reset curr_test_freq and frq_cnt for next test
            curr_test_freq = freq[0];
            frq_cnt = 0;
        }
    }
    // reset determine_gain function
    determine_gain(0.0f, 0.0f, curr_test_freq, gain[frq_cnt], phase[frq_cnt], dwell_complete, true);
}

void AC_AutoTune_Heli::updating_angle_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt)
{
    float test_freq_incr = 0.5f * 3.14159f * 2.0f;
    static uint8_t prev_good_frq_cnt;
    float max_gain = 1.2f;

    if (frq_cnt < 12) {
        if (frq_cnt == 0) {
            freq_cnt_max = 0;
        } else if (phase[frq_cnt] <= 180.0f && !is_zero(phase[frq_cnt])) {
            prev_good_frq_cnt = frq_cnt;
        } else if (frq_cnt > 1 && phase[frq_cnt] > phase[frq_cnt-1] + 360.0f && !is_zero(phase[frq_cnt])) {
            if (phase[frq_cnt] - 360.0f < 180.0f) {
                prev_good_frq_cnt = frq_cnt;
            }
            //        } else if (frq_cnt > 1 && phase[frq_cnt] > 300.0f && !is_zero(phase[frq_cnt])) {
            //            frq_cnt = 11;
        }
        frq_cnt++;
        if (frq_cnt == 12) {
            freq[frq_cnt] = freq[prev_good_frq_cnt];
            curr_test_freq = freq[frq_cnt];
        } else {
            freq[frq_cnt] = freq[frq_cnt-1] + test_freq_incr;
            curr_test_freq = freq[frq_cnt];
        }
    } else {
        if (gain[frq_cnt] < max_gain && phase[frq_cnt] <= 180.0f && phase[frq_cnt] >= 160.0f) {
            tune_p += 0.5f;
        } else if (gain[frq_cnt] < max_gain && phase[frq_cnt] > 180.0f) {
            curr_test_freq = curr_test_freq - 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test_freq;
        } else if (gain[frq_cnt] < max_gain && phase[frq_cnt] < 160.0f) {
            curr_test_freq = curr_test_freq + 0.5 * test_freq_incr;
            freq[frq_cnt] = curr_test_freq;
        } else if (gain[frq_cnt] >= max_gain || tune_p > 10.0f) {
            counter = AUTOTUNE_SUCCESS_COUNT;
            // reset curr_test_freq and frq_cnt for next test
            curr_test_freq = freq[0];
            frq_cnt = 0;
        }
    }
    // reset determine_gain function
    determine_gain(0.0f, 0.0f, curr_test_freq, gain[frq_cnt], phase[frq_cnt], dwell_complete, true);
}

// updating_max_gains: use dwells at increasing frequency to determine gain at which instability will occur
void AC_AutoTune_Heli::updating_max_gains(float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_p, max_gain_data &max_gain_d, float &tune_p, float &tune_d)
{
    float test_freq_incr = 0.5f * 3.14159f * 2.0f;
    static uint8_t find_max_p = 0;
    static uint8_t find_max_d = 0;
    if (frq_cnt < 12) {
        if (frq_cnt > 1 && phase[frq_cnt] > 161.0f && phase[frq_cnt] < 200.0f &&
            phase[frq_cnt-1] > 90.0f && phase[frq_cnt-1] < 161.0f &&
            !is_zero(phase[frq_cnt]) && find_max_p == 0) {
            max_gain_p.freq = linear_interpolate(freq[frq_cnt-1],freq[frq_cnt],161.0f,phase[frq_cnt-1],phase[frq_cnt]);
            max_gain_p.gain = linear_interpolate(gain[frq_cnt-1],gain[frq_cnt],161.0f,phase[frq_cnt-1],phase[frq_cnt]);
            max_gain_p.phase = 161.0f;
            max_gain_p.max_allowed = powf(10.0f,-1 * (log10f(max_gain_p.gain) * 20.0f + 2.42) / 20.0f);
            find_max_p = 1;
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Max rate P freq=%f gain=%f ph=%f rate_d=%f", (double)(max_gain_p.freq), (double)(max_gain_p.gain), (double)(max_gain_p.phase), (double)(max_gain_p.max_allowed));
        }
        if (frq_cnt > 1 && phase[frq_cnt] > 251.0f && phase[frq_cnt] < 300.0f &&
            phase[frq_cnt-1] > 180.0f && phase[frq_cnt-1] < 251.0f &&
            !is_zero(phase[frq_cnt]) && find_max_d == 0) {
            max_gain_d.freq = linear_interpolate(freq[frq_cnt-1],freq[frq_cnt],251.0f,phase[frq_cnt-1],phase[frq_cnt]);
            max_gain_d.gain = linear_interpolate(gain[frq_cnt-1],gain[frq_cnt],251.0f,phase[frq_cnt-1],phase[frq_cnt]);
            max_gain_d.phase = 251.0f;
            max_gain_d.max_allowed = powf(10.0f,-1 * (log10f(max_gain_d.freq * max_gain_d.gain) * 20.0f + 2.42) / 20.0f);
            find_max_d = 1;
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: Max Rate D freq=%f gain=%f ph=%f rate_d=%f", (double)(max_gain_d.freq), (double)(max_gain_d.gain), (double)(max_gain_d.phase), (double)(max_gain_d.max_allowed));
        }
        if (frq_cnt > 1 && phase[frq_cnt] > 300.0f && !is_zero(phase[frq_cnt])) {
            frq_cnt = 11;
        }
        frq_cnt++;
        if (frq_cnt == 12) {
            counter = AUTOTUNE_SUCCESS_COUNT;
            // reset curr_test_freq and frq_cnt for next test
            curr_test_freq = freq[0];
            frq_cnt = 0;
            tune_p = 0.35f * max_gain_p.max_allowed;
            tune_d = 0.25f * max_gain_d.max_allowed;
        } else {
            freq[frq_cnt] = freq[frq_cnt-1] + test_freq_incr;
            curr_test_freq = freq[frq_cnt];
        }
    }
    // reset determine_gain function
    determine_gain(0.0f, 0.0f, curr_test_freq, gain[frq_cnt], phase[frq_cnt], dwell_complete, true);
}

void AC_AutoTune_Heli::Log_AutoTune()
{
    if ((tune_type == SP_DOWN) || (tune_type == SP_UP)) {
        switch (axis) {
        case ROLL:
            Log_Write_AutoTune(axis, tune_type, test_freq[freq_cnt], test_gain[freq_cnt], test_phase[freq_cnt], tune_roll_rff,  tune_roll_rp, tune_roll_rd, tune_roll_sp);
            break;
        case PITCH:
            Log_Write_AutoTune(axis, tune_type, test_freq[freq_cnt], test_gain[freq_cnt], test_phase[freq_cnt], tune_pitch_rff, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp);
            break;
        case YAW:
            Log_Write_AutoTune(axis, tune_type, test_freq[freq_cnt], test_gain[freq_cnt], test_phase[freq_cnt], tune_yaw_rff, tune_yaw_rp, tune_yaw_rd, tune_yaw_sp);
            break;
        }
    } else {
        switch (axis) {
        case ROLL:
            Log_Write_AutoTune(axis, tune_type, test_freq[freq_cnt], test_gain[freq_cnt], test_phase[freq_cnt], tune_roll_rff,  tune_roll_rp, tune_roll_rd, tune_roll_sp);
            break;
        case PITCH:
            Log_Write_AutoTune(axis, tune_type, test_freq[freq_cnt], test_gain[freq_cnt], test_phase[freq_cnt], tune_pitch_rff, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp);
            break;
        case YAW:
            Log_Write_AutoTune(axis, tune_type, test_freq[freq_cnt], test_gain[freq_cnt], test_phase[freq_cnt], tune_yaw_rff, tune_yaw_rp, tune_yaw_rd, tune_yaw_sp);
            break;
        }
    }
}

void AC_AutoTune_Heli::Log_AutoTuneDetails()
{
    Log_Write_AutoTuneDetails(command_out, filt_target_rate, rotation_rate);
}

// @LoggerMessage: ATUN
// @Description: Copter/QuadPlane AutoTune
// @Vehicles: Copter, Plane
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

// Write an Autotune data packet
void AC_AutoTune_Heli::Log_Write_AutoTune(uint8_t _axis, uint8_t tune_step, float dwell_freq, float meas_gain, float meas_phase, float new_gain_rff, float new_gain_rp, float new_gain_rd, float new_gain_sp)
{
    AP::logger().Write(
        "ATUN",
        "TimeUS,Axis,TuneStep,Freq,Gain,Phase,RFF,RP,RD,SP",
        "s---------",
        "F--000----",
        "QBBfffffff",
        AP_HAL::micros64(),
        axis,
        tune_step,
        dwell_freq,
        meas_gain,
        meas_phase,
        new_gain_rff,
        new_gain_rp,
        new_gain_rd,
        new_gain_sp);
}

// Write an Autotune data packet
void AC_AutoTune_Heli::Log_Write_AutoTuneDetails(float motor_cmd, float tgt_rate_rads, float rate_rads)
{
    // @LoggerMessage: ATDE
    // @Description: AutoTune data packet
    // @Field: TimeUS: Time since system startup
    // @Field: Cmd: current motor command
    // @Field: TRate: current target angular rate
    // @Field: Rate: current angular rate
    AP::logger().WriteStreaming(
        "ATDE",
        "TimeUS,Cmd,TRate,Rate",
        "s-kk",
        "F000",
        "Qfff",
        AP_HAL::micros64(),
        motor_cmd,
        tgt_rate_rads*57.3f,
        rate_rads*57.3f);
}

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

float AC_AutoTune_Heli::get_load_tuned_ri(AxisType test_axis)
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

float AC_AutoTune_Heli::get_rp_min() const
{
    return AUTOTUNE_RP_MIN;
}
float AC_AutoTune_Heli::get_sp_min() const
{
    return AUTOTUNE_SP_MIN;
}

float AC_AutoTune_Heli::get_rlpf_min() const
{
    return AUTOTUNE_RLPF_MIN;
}
