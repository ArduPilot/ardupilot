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

class AC_AutoTune_Heli : public AC_AutoTune
{
public:
    // constructor
    AC_AutoTune_Heli()
    {
        //            tune_seq[0] = 4;  // RFF_UP
        //            tune_seq[1] = 8;   // MAX_GAINS
        //            tune_seq[2] = 0; // RD_UP
        //            tune_seq[3] = 2; // RP_UP
        //            tune_seq[2] = 6; // SP_UP
        //            tune_seq[3] = 9; // tune complete
        tune_seq[0] = SP_UP;
        tune_seq[1] = TUNE_COMPLETE;
    };
    // save gained, called on disarm
    void save_tuning_gains() override;

protected:
    void test_init() override;
    void test_run(AxisType test_axis, const float dir_sign) override;
    void do_gcs_announcements() override;
    void load_test_gains() override;

    // generic method used to update gains for the rate p up tune type
    void updating_rate_p_up_all(AxisType test_axis) override;
    // generic method used to update gains for the rate p down tune type
    void updating_rate_p_down_all(AxisType test_axis) override {};
    // generic method used to update gains for the rate d up tune type
    void updating_rate_d_up_all(AxisType test_axis) override;
    // generic method used to update gains for the rate d down tune type
    void updating_rate_d_down_all(AxisType test_axis) override {};
    // generic method used to update gains for the rate ff up tune type
    void updating_rate_ff_up_all(AxisType test_axis) override;
    // generic method used to update gains for the rate ff down tune type
    void updating_rate_ff_down_all(AxisType test_axis) override {};
    // generic method used to update gains for the angle p up tune type
    void updating_angle_p_up_all(AxisType test_axis) override;
    // generic method used to update gains for the angle p down tune type
    void updating_angle_p_down_all(AxisType test_axis) override {};
    // generic method used to update gains for the max gain tune type
    void updating_max_gains_all(AxisType test_axis) override;

    void Log_AutoTune() override;
    void Log_AutoTuneDetails() override;
    void Log_Write_AutoTune(uint8_t _axis, uint8_t tune_step, float dwell_freq, float meas_gain, float meas_phase, float new_gain_rff, float new_gain_rp, float new_gain_rd, float new_gain_sp);
    void Log_Write_AutoTuneDetails(float motor_cmd, float tgt_rate_rads, float rate_rads);
    bool allow_zero_rate_p() override {return true;}
    float get_intra_test_ri(AxisType test_axis) override;
    float get_load_tuned_ri(AxisType test_axis) override;
    float get_load_tuned_yaw_rd() override {return tune_yaw_rd;}
    float get_rp_min() const override;
    float get_sp_min() const override;
    float get_rlpf_min() const override;

private:
    // updating_rate_ff_up - adjust FF to ensure the target is reached
    // FF is adjusted until rate requested is acheived
    void updating_rate_ff_up(float &tune_ff, float rate_target, float meas_rate, float meas_command);
    void updating_rate_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt, float gain_incr, float max_gain);
    void updating_rate_d_up(float &tune_d, float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_d);
    void updating_angle_p_up(float &tune_p, float *freq, float *gain, float *phase, uint8_t &frq_cnt);
   // updating_max_gains: use dwells at increasing frequency to determine gain at which instability will occur
    void updating_max_gains(float *freq, float *gain, float *phase, uint8_t &frq_cnt, max_gain_data &max_gain_p, max_gain_data &max_gain_d, float &tune_p, float &tune_d);

};
