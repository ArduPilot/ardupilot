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
  Multirotor implementation of AutoTune. Based on the original ArduCopter
  autotune code by Leonard Hall.
 */

#pragma once

#include "AC_AutoTune_config.h"

#if AC_AUTOTUNE_ENABLED

#include "AC_AutoTune.h"

class AC_AutoTune_Multi : public AC_AutoTune
{
public:
    // Constructor
    AC_AutoTune_Multi();

    // Saves tuned gains to EEPROM on disarm
    void save_tuning_gains() override;

    // Parameter group for AP_Param registration
    static const struct AP_Param::GroupInfo var_info[];

protected:
    //
    // Gain management and initialization
    //

    // Backs up original gains and resets tuning state
    void backup_gains_and_initialise() override;

    // Loads original pre-tune gains
    void load_orig_gains() override;

    // Loads gains from the last successful autotune session
    void load_tuned_gains() override;

    // Loads conservative gains used between twitch tests
    void load_intra_test_gains() override;

    // Loads current test gains before executing a twitch
    void load_test_gains() override;

    // Resets vehicle-specific test state (not used in multirotor)
    void reset_vehicle_test_variables() override {};

    // Resets vehicle-specific gain tracking state (not used in multirotor)
    void reset_update_gain_variables() override {};

    // Maximum/Minimum target angles for the current axis (in centidegrees)
    float target_angle_max_rp_cd() const override;
    float target_angle_max_y_cd() const override;
    float target_angle_min_rp_cd() const override;
    float target_angle_min_y_cd() const override;

    // Absolute and negative angle abort limits for twitch safety (in centidegrees)
    float angle_lim_max_rp_cd() const override;
    float angle_lim_neg_rpy_cd() const override;

    // Prepares state and targets for a new twitch test
    void test_init() override;

    // Executes one twitch step for the specified axis and direction
    void test_run(AxisType test_axis, const float dir_sign) override;

    // Sends regular status messages to the ground station
    void do_gcs_announcements() override;

    // (Unused) Placeholder for post-test announcements
    void do_post_test_gcs_announcements() override {};

    // Reports final tuned gains to the ground station
    void report_final_gains(AxisType test_axis) const override;

    // Update functions for each TuneType
    void updating_rate_p_up_all(AxisType test_axis) override;
    void updating_rate_d_up_all(AxisType test_axis) override;
    void updating_rate_d_down_all(AxisType test_axis) override;
    void updating_angle_p_up_all(AxisType test_axis) override;
    void updating_angle_p_down_all(AxisType test_axis) override;

    // These tune types are not used in multirotors
    void updating_rate_ff_up_all(AxisType) override {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
    void updating_max_gains_all(AxisType) override {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // Applies gain margin (backoff) at the end of a TuneType
    void set_tuning_gains_with_backoff(AxisType test_axis) override;

    // reverse the direction of the next test
    bool reverse_test_direction() override { return !positive_direction; }

#if HAL_LOGGING_ENABLED
    void Log_AutoTune() override;
    void Log_AutoTuneDetails() override;
    void Log_AutoTuneSweep() override {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    void Log_Write_AutoTune(AxisType axis, TuneType tune_step,
                            float meas_target, float meas_min, float meas_max,
                            float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt);
    void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds);
#endif

    // Defines the sequence of tuning steps (P, D, etc.)
    void set_tune_sequence() override {
        tune_seq[0] = TuneType::RATE_D_UP;
        tune_seq[1] = TuneType::RATE_D_DOWN;
        tune_seq[2] = TuneType::RATE_P_UP;
        tune_seq[3] = TuneType::ANGLE_P_DOWN;
        tune_seq[4] = TuneType::ANGLE_P_UP;
        tune_seq[5] = TuneType::TUNE_COMPLETE;
    }

    // Axis tuning mask accessor
    uint8_t get_axis_bitmask() const override { return axis_bitmask; }

    // Timeout for a single twitch test
    uint32_t get_testing_step_timeout_ms() const override;

private:
    // Helpers for twitch-based test monitoring
    
    void twitching_test_rate(float angle, float rate, float rate_target,
                             float &meas_rate_min, float &meas_rate_max, float &meas_angle_min);

    void twitching_abort_rate(float angle, float rate, float angle_max,
                              float meas_rate_min, float angle_min);

    void twitching_test_angle(float angle, float rate, float angle_target,
                              float &meas_angle_min, float &meas_angle_max,
                              float &meas_rate_min, float &meas_rate_max);

    // Calculates average acceleration during twitch
    void twitching_measure_acceleration(float &accel_average, float rate, float &rate_max) const;

// Gain update routines

// Increases D and adjusts P to produce controlled bounce-back behavior.
// Seeks to bring the response peak just below the target rate while inducing
// a small overshoot (bounce) for evaluating D influence.
void updating_rate_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio,
                        float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio,
                        float rate_target, float meas_rate_min, float meas_rate_max);

// Decreases D and adjusts P to eliminate bounce-back in the response.
// Aims to achieve a clean response just below the target rate with no overshoot.
void updating_rate_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio,
                          float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio,
                          float rate_target, float meas_rate_min, float meas_rate_max);

// Increases P to ensure the target rate is reached quickly, while reducing D
// if bounce-back exceeds the aggressiveness threshold.
// Fails if D hits minimum and clean response is not achievable (optional).
void updating_rate_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio,
                                float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio,
                                float rate_target, float meas_rate_min, float meas_rate_max,
                                bool fail_min_d = true);

// Decreases angle P gain to reduce overshoot until the target is no longer reached before timeout.
// Used to back off from aggressive behavior in angle control.
void updating_angle_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio,
                           float angle_target, float meas_angle_max,
                           float meas_rate_min, float meas_rate_max);

// Increases angle P gain to ensure the target is reached within a reasonable time.
// Stops increasing once target response is consistently achieved without overshoot.
void updating_angle_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio,
                         float angle_target, float meas_angle_max,
                         float meas_rate_min, float meas_rate_max);

    // Formats and sends gain reports
    void report_axis_gains(const char* axis_string, float rate_P, float rate_I,
                           float rate_D, float angle_P, float max_accel_radss) const;

    // Parameters
    AP_Int8  axis_bitmask;      // Axis enable mask
    AP_Float aggressiveness;    // Target overshoot ratio (D tuning sensitivity)
    AP_Float min_d;             // Minimum allowed D gain
    AP_Float gain_backoff;      // Fraction by which tuned P and D gains are reduced after each AutoTune stage (rate loop, then angle loop) to provide additional stability margin
    bool     ignore_next;       // Skip next result (used for rate overshoot handling)

    // Measurement and target values for each test step
    float target_angle;
    float target_rate;
    float angle_abort;

    float test_rate_min;
    float test_rate_max;
    float test_angle_min;
    float test_angle_max;

    float accel_measure_rate_max;
};

#endif // AC_AUTOTUNE_ENABLED
