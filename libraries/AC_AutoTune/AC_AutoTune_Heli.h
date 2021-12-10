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
  support for autotune of helicopters
 */

#pragma once

#include "AC_AutoTune.h"

class AC_AutoTune_Heli : public AC_AutoTune
{
public:
    // constructor
    AC_AutoTune_Heli();

    // save gained, called on disarm
    void save_tuning_gains() override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    void load_test_gains() override;

    // get intra test rate I gain for the specified axis
    float get_intra_test_ri(AxisType test_axis) override;

    // get tuned rate I gain for the specified axis
    float get_tuned_ri(AxisType test_axis) override;

    // get tuned yaw rate d gain
    float get_tuned_yaw_rd() override { return tune_yaw_rd; }

    // initializes test
    void test_init() override;

    // runs test
    void test_run(AxisType test_axis, const float dir_sign) override;

    // update gains for the rate p up tune type
    void updating_rate_p_up_all(AxisType test_axis) override;

    // update gains for the rate d up tune type
    void updating_rate_d_up_all(AxisType test_axis) override;

    // update gains for the rate d down tune type
    void updating_rate_d_down_all(AxisType test_axis) override {};

    // update gains for the rate ff up tune type
    void updating_rate_ff_up_all(AxisType test_axis) override;

    // update gains for the angle p up tune type
    void updating_angle_p_up_all(AxisType test_axis) override;

    // update gains for the angle p down tune type
    void updating_angle_p_down_all(AxisType test_axis) override {};

    // update gains for the max gain tune type
    void updating_max_gains_all(AxisType test_axis) override;

    // get minimum rate P (for any axis)
    float get_rp_min() const override;

    // get minimum angle P (for any axis)
    float get_sp_min() const override;

    // get minimum rate Yaw filter value
    float get_yaw_rate_filt_min() const override;

    // reverse direction for twitch test
    bool twitch_reverse_direction() override { return positive_direction; }

    // methods to log autotune summary data
    void Log_AutoTune() override;
    void Log_Write_AutoTune(uint8_t _axis, uint8_t tune_step, float dwell_freq, float meas_gain, float meas_phase, float new_gain_rff, float new_gain_rp, float new_gain_rd, float new_gain_sp, float max_accel);

    // methods to log autotune time history results for command, angular rate, and attitude.
    void Log_AutoTuneDetails() override;
    void Log_Write_AutoTuneDetails(float motor_cmd, float tgt_rate_rads, float rate_rads, float tgt_ang_rad, float ang_rad);

    // methods to log autotune frequency response results
    void Log_AutoTuneSweep() override;
    void Log_Write_AutoTuneSweep(float freq, float gain, float phase);

    // returns true if rate P gain of zero is acceptable for this vehicle
    bool allow_zero_rate_p() override { return true; }

    // returns true if max tested accel is used for parameter
    bool set_accel_to_max_test_value() override { return false; }

    // send intermittant updates to user on status of tune
    void do_gcs_announcements() override;

    // set the tuning test sequence
    void set_tune_sequence() override;

    // tuning sequence bitmask
    AP_Int8  seq_bitmask;

    // minimum sweep frequency
    AP_Float min_sweep_freq;

    // maximum sweep frequency
    AP_Float max_sweep_freq;

    // maximum response gain
    AP_Float max_resp_gain;

private:
    // updating_rate_ff_up - adjust FF to ensure the target is reached
    // FF is adjusted until rate requested is acheived
    void updating_rate_ff_up(float &tune_ff, float rate_target, float meas_rate, float meas_command);

    // updating_rate_p_up - uses maximum allowable gain determined from max_gain test to determine rate p gain that does not exceed exceed max response gain
    void updating_rate_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_p);

    // updating_rate_d_up - uses maximum allowable gain determined from max_gain test to determine rate d gain where the response gain is at a minimum
    void updating_rate_d_up(float &tune_d, float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_d);

    // updating_angle_p_up - determines maximum angle p gain for pitch and roll
    void updating_angle_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt);

    // updating_angle_p_up_yaw - determines maximum angle p gain for yaw
    void updating_angle_p_up_yaw(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt);

   // updating_max_gains: use dwells at increasing frequency to determine gain at which instability will occur
    void updating_max_gains(float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_p, max_gain_data &max_gain_d, float &tune_p, float &tune_d);

    uint8_t method; //0: determine freq, 1: use max gain method, 2: use phase 180 method

    // updating rate FF variables
    // flag for completion of the initial direction for the feedforward test
    bool first_dir_complete;
    // feedforward gain resulting from testing in the initial direction
    float first_dir_rff;

    // updating max gain variables
    // flag for finding maximum p gain
    bool found_max_p;
    // flag for finding maximum d gain
    bool found_max_d;
    // flag for interpolating to find max response gain
    bool find_middle;

    // updating angle P up variables
    // track the maximum phase
    float phase_max;
    // previous gain
    float sp_prev_gain;
    // flag for finding the peak of the gain response
    bool find_peak;

    // updating angle P up yaw
    // counter value of previous good frequency
    uint8_t sp_prev_good_frq_cnt;

    // updating rate P up
    // counter value of previous good frequency
    uint8_t rp_prev_good_frq_cnt;
    // previous gain
    float rp_prev_gain;

    // updating rate D up
    // counter value of previous good frequency
    uint8_t rd_prev_good_frq_cnt;
    // previous gain
    float rd_prev_gain;

};
