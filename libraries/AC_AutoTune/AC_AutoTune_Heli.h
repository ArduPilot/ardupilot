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
#include <AP_Math/chirp.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_Scheduler/AP_Scheduler.h>

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

    //
    // methods to load and save gains
    //

    // backup original gains and prepare for start of tuning
    void backup_gains_and_initialise() override;

    // load gains
    void load_gain_set(AxisType s_axis, float rate_p, float rate_i, float rate_d, float rate_ff, float angle_p, float max_accel, float rate_fltt, float rate_flte, float smax);

    // switch to use original gains
    void load_orig_gains() override;

    // switch to gains found by last successful autotune
    void load_tuned_gains() override;

    // load gains used between tests. called during testing mode's update-gains step to set gains ahead of return-to-level step
    void load_intra_test_gains() override;

    // load test gains
    void load_test_gains() override;

    // reset the test vaariables for heli
    void reset_vehicle_test_variables() override;

    // reset the update gain variables for heli
    void reset_update_gain_variables() override;

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

    // set gains post tune for the tune type
    void set_gains_post_tune(AxisType test_axis) override;

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

    // send intermittant updates to user on status of tune
    void do_gcs_announcements() override;

    // send post test updates to user
    void do_post_test_gcs_announcements() override;

    // report final gains for a given axis to GCS
    void report_final_gains(AxisType test_axis) const override;

    // set the tuning test sequence
    void set_tune_sequence() override;

    // get_axis_bitmask accessor
    uint8_t get_axis_bitmask() const override { return axis_bitmask; }

    // get_testing_step_timeout_ms accessor
    uint32_t get_testing_step_timeout_ms() const override;

private:
    // max_gain_data type stores information from the max gain test
    struct max_gain_data {
        float freq;
        float phase;
        float gain;
        float max_allowed;
    };

    // max gain data for rate p tuning
    max_gain_data max_rate_p;
    // max gain data for rate d tuning
    max_gain_data max_rate_d;

    // dwell type identifies whether the dwell is ran on rate or angle
    enum DwellType {
        RATE    = 0,
        ANGLE   = 1,
    };

    // Feedforward test used to determine Rate FF gain
    void rate_ff_test_init();
    void rate_ff_test_run(float max_angle_cds, float target_rate_cds, float dir_sign);

    // initialize dwell test or angle dwell test variables
    void dwell_test_init(float start_frq, float stop_frq, float filt_freq, DwellType dwell_type);

    // dwell test used to perform frequency dwells for rate gains
    void dwell_test_run(uint8_t freq_resp_input, float start_frq, float stop_frq, float &dwell_gain, float &dwell_phase, DwellType dwell_type);

    // updating_rate_ff_up - adjust FF to ensure the target is reached
    // FF is adjusted until rate requested is acheived
    void updating_rate_ff_up(float &tune_ff, float rate_target, float meas_rate, float meas_command);

    // updating_rate_p_up - uses maximum allowable gain determined from max_gain test to determine rate p gain that does not exceed exceed max response gain
    void updating_rate_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_p);

    // updating_rate_d_up - uses maximum allowable gain determined from max_gain test to determine rate d gain where the response gain is at a minimum
    void updating_rate_d_up(float &tune_d, float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_d);

    // updating_angle_p_up - determines maximum angle p gain for pitch and roll
    void updating_angle_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt);

   // updating_max_gains: use dwells at increasing frequency to determine gain at which instability will occur
    void updating_max_gains(float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_p, max_gain_data &max_gain_d, float &tune_p, float &tune_d);

    // reset the max_gains update gain variables
    void reset_maxgains_update_gain_variables();

    // reset the sweep variables
    void reset_sweep_variables();

    // exceeded_freq_range - ensures tuning remains inside frequency range
    bool exceeded_freq_range(float frequency);

    // report gain formating helper
    void report_axis_gains(const char* axis_string, float rate_P, float rate_I, float rate_D, float rate_ff, float angle_P, float max_accel) const;

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

    // updating rate P up
    // counter value of previous good frequency
    uint8_t rp_prev_good_frq_cnt;

    // updating rate D up
    // counter value of previous good frequency
    uint8_t rd_prev_good_frq_cnt;
    // previous gain
    float rd_prev_gain;

    uint8_t  ff_test_phase;                         // phase of feedforward test
    float    test_command_filt;                     // filtered commanded output for FF test analysis
    float    test_rate_filt;                        // filtered rate output for FF test analysis
    float    command_out;                           // test axis command output
    float    test_tgt_rate_filt;                    // filtered target rate for FF test analysis
    float    filt_target_rate;                      // filtered target rate
    float    test_gain[20];                         // frequency response gain for each dwell test iteration
    float    test_freq[20];                         // frequency of each dwell test iteration
    float    test_phase[20];                        // frequency response phase for each dwell test iteration
    float    dwell_start_time_ms;                   // start time in ms of dwell test
    uint8_t  freq_cnt_max;                          // counter number for frequency that produced max gain response

    // sweep_info contains information about a specific test's sweep results
    struct sweep_info {
        float freq;
        float gain;
        float phase;
    };
    sweep_info curr_test;

    Vector3f start_angles;                          // aircraft attitude at the start of test
    uint32_t settle_time;                           // time in ms for allowing aircraft to stabilize before initiating test
    uint32_t phase_out_time;                        // time in ms to phase out response
    float    waveform_freq_rads;                    //current frequency for chirp waveform
    float    trim_pff_out;                          // trim output of the PID rate controller for P, I and FF terms
    float    trim_meas_rate;                        // trim measured gyro rate

    //variables from rate FF test
    float trim_command_reading;
    float trim_heading;
    LowPassFilterFloat rate_request_cds;
    LowPassFilterFloat angle_request_cd;

    // variables from dwell test
    LowPassFilterVector2f filt_pit_roll_cd;         // filtered pitch and roll attitude for dwell rate method
    LowPassFilterFloat filt_heading_error_cd;       // filtered heading error for dwell rate method
    LowPassFilterVector2f filt_att_fdbk_from_velxy_cd;
    LowPassFilterFloat filt_command_reading;        // filtered command reading to keep oscillation centered
    LowPassFilterFloat filt_gyro_reading;           // filtered gyro reading to keep oscillation centered
    LowPassFilterFloat filt_tgt_rate_reading;       // filtered target rate reading to keep oscillation centered

    // trim variables for determining trim state prior to test starting
    Vector3f trim_attitude_cd;                      // trim attitude before starting test
    float trim_command;                             // trim target yaw reading before starting test
    float trim_yaw_tgt_reading;                     // trim target yaw reading before starting test
    float trim_yaw_heading_reading;                 // trim heading reading before starting test

    LowPassFilterFloat  command_filt;               // filtered command - filtering intended to remove noise
    LowPassFilterFloat  target_rate_filt;           // filtered target rate in radians/second - filtering intended to remove noise

    // sweep_data tracks the overall characteristics in the response to the frequency sweep
    struct sweep_data {
        sweep_info maxgain;
        sweep_info ph180;
        sweep_info ph270;

        uint8_t  progress;  // set based on phase of frequency response.  0 - start; 1 - reached 180 deg; 2 - reached 270 deg;
    };
    sweep_data sweep;

    // fix the frequency sweep time to 23 seconds
    const float sweep_time_ms = 23000;


    // parameters
    AP_Int8  axis_bitmask;        // axes to be tuned
    AP_Int8  seq_bitmask;       // tuning sequence bitmask
    AP_Float min_sweep_freq;    // minimum sweep frequency
    AP_Float max_sweep_freq;    // maximum sweep frequency
    AP_Float max_resp_gain;     // maximum response gain
    AP_Float vel_hold_gain;     // gain for velocity hold

    // freqresp object for the frequency response tests
    AC_AutoTune_FreqResp freqresp;

    Chirp chirp_input;
};
