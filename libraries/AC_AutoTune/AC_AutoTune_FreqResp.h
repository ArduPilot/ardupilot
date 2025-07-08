#pragma once

/*
 Gain and phase determination algorithm
*/

#include <AP_Math/AP_Math.h>

class AC_AutoTune_FreqResp {
public:
    // Constructor
    AC_AutoTune_FreqResp()
{
}

    // Enumeration of input type
    enum InputType {
        DWELL = 0,                 
        SWEEP = 1, 
    };

    // Enumeration of type
    enum ResponseType {
        RATE = 0,                 
        ANGLE = 1, 
    };

    // Initialize the Frequency Response Object. 
    // Must be called before running dwell or frequency sweep tests
    void init(InputType input_type, ResponseType response_type, uint8_t cycles);

    // Determines the gain and phase based on angle response for a dwell or sweep
    void update(float command, float tgt_resp, float meas_resp, float tgt_freq);

    // Enable external query if cycle is complete and freq response data are available
    bool is_cycle_complete() { return cycle_complete;}

    // Reset cycle_complete flag
    void reset_cycle_complete() { cycle_complete = false; }

    // Frequency response data accessors
    float get_freq() { return curr_test_freq; }
    float get_gain() { return curr_test_gain; }
    float get_phase() { return curr_test_phase; }
    float get_accel_max() { return max_accel; }

private:
    // time of the start of a new target value search.  keeps noise from prematurely starting the search of a new target value.
    uint32_t new_tgt_time_ms;

    // flag for searching for a new target peak
    bool new_target = false;

    // maximum target value
    float max_target;

    // time of maximum target value in current cycle
    uint32_t max_tgt_time;

    // counter for target value maximums
    uint16_t max_target_cnt;

    // holds previously determined maximum target value while current cycle is running
    float temp_max_target;

    // holds previously determined time of maximum target value while current cycle is running
    uint32_t temp_max_tgt_time;

    // minimum target value
    float min_target;

    // counter for target value minimums
    uint16_t min_target_cnt;

    // holds previously determined minimum target value while current cycle is running
    float temp_min_target;

    // maximum target value from previous cycle
    float prev_target;

    // maximum target response from previous cycle
    float prev_tgt_resp;

    // holds target amplitude for gain calculation
    float temp_tgt_ampl;

    // time of the start of a new measured value search.  keeps noise from prematurely starting the search of a new measured value.
    uint32_t new_meas_time_ms;

    // flag for searching for a new measured peak
    bool new_meas = false;

    // maximum measured value
    float max_meas;

    // time of maximum measured value in current cycle
    uint32_t max_meas_time;

    // counter for measured value maximums
    uint16_t max_meas_cnt;

    // holds previously determined maximum measured value while current cycle is running
    float temp_max_meas;

    // holds previously determined time of maximum measured value while current cycle is running
    uint32_t temp_max_meas_time;

    // minimum measured value
    float min_meas;

    // counter for measured value minimums
    uint16_t min_meas_cnt;

    // holds previously determined minimum measured value while current cycle is running
    float temp_min_meas;

    // maximum measured value from previous cycle
    float prev_meas;

    // maximum measured response from previous cycle
    float prev_meas_resp;

    // holds measured amplitude for gain calculation
    float temp_meas_ampl;

    // calculated target rate from angle data
    float target_rate;

    // calculated measured rate from angle data
    float measured_rate;

    // holds start time of input to track length of time that input in running
    uint32_t input_start_time_ms;

    // flag indicating when one oscillation cycle is complete
    bool cycle_complete = false;

    // number of dwell cycles to complete for dwell excitation
    uint8_t dwell_cycles;

    // current test frequency, gain, and phase
    float curr_test_freq; 
    float curr_test_gain;
    float curr_test_phase;

    // maximum measured rate throughout excitation used for max accel calculation
    float max_meas_rate;

    // maximum command associated with maximum rate used for max accel calculation
    float max_command;

    // maximum acceleration in cdss determined during test
    float max_accel;

    // Input type for frequency response object
    InputType excitation;

    // Response type for frequency response object
    ResponseType response;

    // sweep_peak_finding_data tracks the peak data
    struct sweep_peak_finding_data {
        uint16_t count_m1;
        float amplitude_m1;
        float max_time_m1;
    };

    // Measured data for sweep peak
    sweep_peak_finding_data sweep_meas;

    // Target data for sweep peak
    sweep_peak_finding_data sweep_tgt;

    //store gain data in ring buffer
    struct peak_info {
        uint16_t curr_count;
        float amplitude;
        uint32_t time_ms; 

    };

    // Buffer object for measured peak data
    ObjectBuffer<peak_info> meas_peak_info_buffer{12};

    // Buffer object for target peak data
    ObjectBuffer<peak_info> tgt_peak_info_buffer{12};

    // Push data into measured peak data buffer object
    void push_to_meas_buffer(uint16_t count, float amplitude, uint32_t time_ms);

    // Pull data from measured peak data buffer object
    void pull_from_meas_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms);

    // Push data into target peak data buffer object
    void push_to_tgt_buffer(uint16_t count, float amplitude, uint32_t time_ms);

    // Pull data from target peak data buffer object
    void pull_from_tgt_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms);

};
