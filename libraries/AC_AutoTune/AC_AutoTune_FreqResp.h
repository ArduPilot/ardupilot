#pragma once

/*
 Gain and phase determination algorithm
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#define AUTOTUNE_DWELL_CYCLES                6

class AC_AutoTune_FreqResp {
public:
    // Constructor
    AC_AutoTune_FreqResp()
{
    // initialize test variables
    meas_peak_info_buffer = new ObjectBuffer<peak_info>(AUTOTUNE_DWELL_CYCLES);
    tgt_peak_info_buffer = new ObjectBuffer<peak_info>(AUTOTUNE_DWELL_CYCLES);
}

    enum InputType {
        DWELL = 0,                 
        SWEEP = 1, 
    };

    // Initialize the Frequency Response Object. 
    // Must be called before running dwell or frequency sweep tests
    void init(InputType input_type);

    // determines the gain and phase based on rate response for a dwell or sweep
    void update_rate(float tgt_rate, float meas_rate, float tgt_freq);

    // determines the gain and phase based on angle response for a dwell or sweep
    void update_angle(float command, float tgt_angle, float meas_angle, float tgt_freq);

    // enable external query if cycle is complete and freq response data are available
    bool is_cycle_complete() { return cycle_complete;}

    // reset cycle_complete flag
    void reset_cycle_complete() { cycle_complete = false; }

    // frequency response data accessors
    float get_freq() { return curr_test_freq; }
    float get_gain() { return curr_test_gain; }
    float get_phase() { return curr_test_phase; }
    float get_accel_max() { return max_accel; }

private:
    float max_target, max_meas, prev_target, prev_meas, prev_tgt_angle, prev_meas_angle;
    float min_target, min_meas, temp_meas_ampl, temp_tgt_ampl;
    float temp_max_target, temp_min_target, target_rate, measured_rate, max_meas_rate, max_command;
    float temp_max_meas, temp_min_meas;
    uint32_t temp_max_tgt_time, temp_max_meas_time;
    uint32_t max_tgt_time, max_meas_time, new_tgt_time_ms, new_meas_time_ms, input_start_time_ms;
    uint16_t min_target_cnt, max_target_cnt, max_meas_cnt, min_meas_cnt;
    bool new_target = false;
    bool new_meas = false;
    bool cycle_complete = false;
    float curr_test_freq, curr_test_gain, curr_test_phase;
    float max_accel;
    InputType excitation;

    // sweep_peak_finding_data tracks the peak data
    struct sweep_peak_finding_data {
        uint16_t count_m1;
        float amplitude_m1;
        float max_time_m1;
    };
    sweep_peak_finding_data sweep_meas;
    sweep_peak_finding_data sweep_tgt;

    //store gain data in ring buffer
    struct peak_info {
        uint16_t curr_count;
        float amplitude;
        uint32_t time_ms; 

    };
    ObjectBuffer<peak_info> *meas_peak_info_buffer;
    ObjectBuffer<peak_info> *tgt_peak_info_buffer;
    void push_to_meas_buffer(uint16_t count, float amplitude, uint32_t time_ms);
    void pull_from_meas_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms);
    void push_to_tgt_buffer(uint16_t count, float amplitude, uint32_t time_ms);
    void pull_from_tgt_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms);

};
