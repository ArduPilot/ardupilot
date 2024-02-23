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

#include "AC_AutoTune_config.h"

#if AC_AUTOTUNE_ENABLED

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
    //
    // methods to load and save gains
    //

    // backup original gains and prepare for start of tuning
    void backup_gains_and_initialise() override;

    // switch to use original gains
    void load_orig_gains() override;

    // switch to gains found by last successful autotune
    void load_tuned_gains() override;

    // load gains used between tests. called during testing mode's update-gains step to set gains ahead of return-to-level step
    void load_intra_test_gains() override;

    // load test gains
    void load_test_gains() override;

    // reset the test variables for multi
    void reset_vehicle_test_variables() override {};

    // reset the update gain variables for multi
    void reset_update_gain_variables() override {};

    void test_init() override;
    void test_run(AxisType test_axis, const float dir_sign) override;

    // send intermittent updates to user on status of tune
    void do_gcs_announcements() override;

    // send post test updates to user
    void do_post_test_gcs_announcements() override {};

    // report final gains for a given axis to GCS
    void report_final_gains(AxisType test_axis) const override;

    // update gains for the rate P up tune type
    void updating_rate_p_up_all(AxisType test_axis) override;

    // update gains for the rate D up tune type
    void updating_rate_d_up_all(AxisType test_axis) override;

    // update gains for the rate D down tune type
    void updating_rate_d_down_all(AxisType test_axis) override;

    // update gains for the rate ff up tune type
    void updating_rate_ff_up_all(AxisType test_axis) override {
        // this should never happen
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // update gains for the angle P up tune type
    void updating_angle_p_up_all(AxisType test_axis) override;

    // update gains for the angle P down tune type
    void updating_angle_p_down_all(AxisType test_axis) override;

    // update gains for the max gain tune type
    void updating_max_gains_all(AxisType test_axis) override {
        // this should never happen
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // set gains post tune for the tune type
    void set_gains_post_tune(AxisType test_axis) override;

    // reverse direction for twitch test
    bool twitch_reverse_direction() override { return !positive_direction; }

#if HAL_LOGGING_ENABLED
    void Log_AutoTune() override;
    void Log_AutoTuneDetails() override;
    void Log_AutoTuneSweep() override {
        // this should never happen
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt);
    void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds);
#endif

    void set_tune_sequence() override {
        tune_seq[0] = RD_UP;
        tune_seq[1] = RD_DOWN;
        tune_seq[2] = RP_UP;
        tune_seq[3] = SP_DOWN;
        tune_seq[4] = SP_UP;
        tune_seq[5] = TUNE_COMPLETE;
    }

    // get_axis_bitmask accessor
    uint8_t get_axis_bitmask() const override { return axis_bitmask; }

    // get_testing_step_timeout_ms accessor
    uint32_t get_testing_step_timeout_ms() const override;

private:
    // twitch test functions for multicopter
    void twitch_test_init();
    void twitch_test_run(AxisType test_axis, const float dir_sign);

    void twitching_test_rate(float angle, float rate, float rate_target, float &meas_rate_min, float &meas_rate_max, float &meas_angle_min);
    void twitching_abort_rate(float angle, float rate, float angle_max, float meas_rate_min, float angle_min);
    void twitching_test_angle(float angle, float rate, float angle_target, float &meas_angle_min, float &meas_angle_max, float &meas_rate_min, float &meas_rate_max);

    // measure acceleration during twitch test
    void twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max) const;

    // updating_rate_d_up - increase D and adjust P to optimize the D term for a little bounce back
    // optimize D term while keeping the maximum just below the target by adjusting P
    void updating_rate_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max);

    // updating_rate_d_down - decrease D and adjust P to optimize the D term for no bounce back
    // optimize D term while keeping the maximum just below the target by adjusting P
    void updating_rate_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max);

    // updating_rate_p_up_d_down - increase P to ensure the target is reached while checking bounce back isn't increasing
    // P is increased until we achieve our target within a reasonable time while reducing D if bounce back increases above the threshold
    void updating_rate_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float rate_target, float meas_rate_min, float meas_rate_max, bool fail_min_d = true);

    // updating_angle_p_down - decrease P until we don't reach the target before time out
    // P is decreased to ensure we are not overshooting the target
    void updating_angle_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max);

    // updating_angle_p_up - increase P to ensure the target is reached
    // P is increased until we achieve our target within a reasonable time
    void updating_angle_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float angle_target, float meas_angle_max, float meas_rate_min, float meas_rate_max);

    // report gain formatting helper
    void report_axis_gains(const char* axis_string, float rate_P, float rate_I, float rate_D, float angle_P, float max_accel) const;

    // parameters
    AP_Int8  axis_bitmask;        // axes to be tuned
    AP_Float aggressiveness;      // aircraft response aggressiveness to be tuned
    AP_Float min_d;               // minimum rate d gain allowed during tuning
};

#endif  // AC_AUTOTUNE_ENABLED
