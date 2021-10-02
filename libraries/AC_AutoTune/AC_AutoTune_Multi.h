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
  support for autotune of multirotors. Based on original autotune code from ArduCopter, written by Leonard Hall
  Converted to a library by Andrew Tridgell
 */

#pragma once

#include "AC_AutoTune.h"

class AC_AutoTune_Multi : public AC_AutoTune
{
public:
    // constructor
    AC_AutoTune_Multi();

    // save gained, called on disarm
    void save_tuning_gains() override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // load gains
    void load_test_gains() override;

    // get intra test rate I gain for the specified axis
    float get_intra_test_ri(AxisType test_axis) override;

    // get tuned rate I gain for the specified axis
    float get_tuned_ri(AxisType test_axis) override;

    // get tuned yaw rate D gain
    float get_tuned_yaw_rd() override { return 0.0f; }

    void test_init() override;
    void test_run(AxisType test_axis, const float dir_sign) override;
    void do_gcs_announcements() override;

    // update gains for the rate P up tune type
    void updating_rate_p_up_all(AxisType test_axis) override;

    // update gains for the rate P down tune type
    void updating_rate_p_down_all(AxisType test_axis) override {};

    // update gains for the rate D up tune type
    void updating_rate_d_up_all(AxisType test_axis) override;

    // update gains for the rate D down tune type
    void updating_rate_d_down_all(AxisType test_axis) override;

    // update gains for the rate ff up tune type
    void updating_rate_ff_up_all(AxisType test_axis) override {};

    // update gains for the rate ff down tune type
    void updating_rate_ff_down_all(AxisType test_axis) override {};

    // update gains for the angle P up tune type
    void updating_angle_p_up_all(AxisType test_axis) override;

    // update gains for the angle P down tune type
    void updating_angle_p_down_all(AxisType test_axis) override;

    // update gains for the max gain tune type
    void updating_max_gains_all(AxisType test_axis) override {};

    // returns true if rate P gain of zero is acceptable for this vehicle
    bool allow_zero_rate_p() override { return false; }

    // returns true if max tested accel is used for parameter
    bool set_accel_to_max_test_value() override { return true; }

    // get minimum rate P (for any axis)
    float get_rp_min() const override;

    // get minimum angle P (for any axis)
    float get_sp_min() const override;

    // get minimum yaw rate filter value
    float get_yaw_rate_filt_min() const override;

    // reverse direction for twitch test
    bool twitch_reverse_direction() override { return !positive_direction; }

    void Log_AutoTune() override;
    void Log_AutoTuneDetails() override;
    void Log_AutoTuneSweep() override {};
    void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt);
    void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds);

private:
    // updating_rate_d_up - increase D and adjust P to optimize the D term for a little bounce back
    // optimize D term while keeping the maximum just below the target by adjusting P
    void updating_rate_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max);

    // updating_rate_d_down - decrease D and adjust P to optimize the D term for no bounce back
    // optimize D term while keeping the maximum just below the target by adjusting P
    void updating_rate_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max);

    // updating_rate_p_up_d_down - increase P to ensure the target is reached while checking bounce back isn't increasing
    // P is increased until we achieve our target within a reasonable time while reducing D if bounce back increases above the threshold
    void updating_rate_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max);

    // updating_angle_p_down - decrease P until we don't reach the target before time out
    // P is decreased to ensure we are not overshooting the target
    void updating_angle_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max);

    // updating_angle_p_up - increase P to ensure the target is reached
    // P is increased until we achieve our target within a reasonable time
    void updating_angle_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max);

    void set_tune_sequence() override {
        tune_seq[0] = RD_UP;
        tune_seq[1] = RD_DOWN;
        tune_seq[2] = RP_UP;
        tune_seq[3] = SP_DOWN;
        tune_seq[4] = SP_UP;
        tune_seq[5] = TUNE_COMPLETE;
    }

};
