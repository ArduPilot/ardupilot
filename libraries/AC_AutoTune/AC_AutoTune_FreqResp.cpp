/*
This function receives time history data during a dwell test or frequency sweep and determines the gain and phase of the response to the input.
Once the designated number of cycles are complete, the average of the gain and phase are determined over the last 5 cycles and the cycles_complete flag
is set.  This function must be reset using the reset flag prior to the next dwell.
*/

#include <AP_HAL/AP_HAL.h>
#include "AC_AutoTune_FreqResp.h"

float AC_AutoTune_FreqResp::update()
{
    float dummy = 0.0f;
    return dummy;
}

// Initialize the Frequency Response methods. Must be called before running dwell or frequency sweep tests
void AC_AutoTune_FreqResp::init(InputType input_type)
{
    excitation = input_type;
    max_target_cnt = 0;
    min_target_cnt = 0;
    max_meas_cnt = 0;
    min_meas_cnt = 0;
    input_start_time_ms = 0;
    new_tgt_time_ms = 0;
    new_meas_time_ms = 0;
    new_target = false;
    new_meas = false;
    curr_test_freq = 0.0f;
    curr_test_gain = 0.0f;
    curr_test_phase = 0.0f;
    max_accel = 0.0f;
    max_meas_rate = 0.0f;
    max_command = 0.0f;
    meas_peak_info_buffer->clear();
    tgt_peak_info_buffer->clear();
    cycle_complete = false;
}

// determine_gain - this function receives time history data during a dwell and frequency sweep tests for max gain,  
// rate P and rate D tuning and determines the gain and phase of the response to the input. For dwell tests once  
// the designated number of cycles are complete, the average of the gain and phase are determined over the last 5 cycles 
// and the cycles_complete flag is set. For frequency sweep tests, phase and gain are determined for every cycle and 
// cycle_complete flag is set to indicate when to pull the phase and gain data.  The flag is reset to enable the next 
// cycle to be analyzed.
void AC_AutoTune_FreqResp::determine_gain(float tgt_rate, float meas_rate, float tgt_freq)
{
    uint32_t now = AP_HAL::millis();
    uint32_t half_cycle_time_ms = 0;
    uint32_t cycle_time_ms = 0;

    if (cycle_complete) {
        return;
    }

    if (!is_zero(tgt_freq)) {
        half_cycle_time_ms = (uint32_t)(300 * 6.28 / tgt_freq);
        cycle_time_ms = (uint32_t)(1000 * 6.28 / tgt_freq);
    }

    if (input_start_time_ms == 0) {
        input_start_time_ms = now;
    }

    // cycles are complete! determine gain and phase and exit
    if (max_meas_cnt > AUTOTUNE_DWELL_CYCLES + 1 && max_target_cnt > AUTOTUNE_DWELL_CYCLES + 1 && excitation == DWELL) {
        float delta_time = 0.0f;
        float sum_gain = 0.0f;
        uint8_t cnt = 0;
        uint8_t gcnt = 0;
        uint16_t meas_cnt, tgt_cnt;
        float meas_ampl = 0.0f;
        float tgt_ampl = 0.0f;
        uint32_t meas_time = 0;
        uint32_t tgt_time = 0;
        for (int i = 0;  i < AUTOTUNE_DWELL_CYCLES; i++) {
            meas_cnt=0;
            tgt_cnt=0;
            pull_from_meas_buffer(meas_cnt, meas_ampl, meas_time);
            pull_from_tgt_buffer(tgt_cnt, tgt_ampl, tgt_time);
            push_to_meas_buffer(0, 0.0f, 0);
            push_to_tgt_buffer(0, 0.0f, 0);
            // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: tgt_cnt=%f meas_cnt=%f", (double)(tgt_cnt), (double)(meas_cnt));

            if (meas_cnt == tgt_cnt && meas_cnt != 0) {
                if (tgt_ampl > 0.0f) {
                    sum_gain += meas_ampl / tgt_ampl;
                    gcnt++;
                }
                float d_time = (float)(meas_time - tgt_time);
                if (d_time < 2.0f * (float)cycle_time_ms) {
                    delta_time += d_time;
                    cnt++;
                }
            } else if (meas_cnt > tgt_cnt) {
                pull_from_tgt_buffer(tgt_cnt, tgt_ampl, tgt_time);
                push_to_tgt_buffer(0, 0.0f, 0);
            } else if (meas_cnt < tgt_cnt) {
                pull_from_meas_buffer(meas_cnt, meas_ampl, meas_time);
                push_to_meas_buffer(0, 0.0f, 0);
            }                

        }
        if (gcnt > 0) {
            curr_test_gain = sum_gain / gcnt;
        }
        if (cnt > 0) {
            delta_time = delta_time / cnt;
        }
        curr_test_phase = tgt_freq * delta_time * 0.001f * 360.0f / 6.28f;
        if (curr_test_phase > 360.0f) {
            curr_test_phase = curr_test_phase - 360.0f;
        }
        curr_test_freq = tgt_freq;
        cycle_complete = true;
        // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: cycles completed");
        return;
    }

    // Indicates when the target(input) is positive or negative half of the cycle to notify when the max or min should be sought
    if (!is_positive(prev_target) && is_positive(tgt_rate) && !new_target && now > new_tgt_time_ms) {
        new_target = true;
        new_tgt_time_ms = now + half_cycle_time_ms;
        // reset max_target
        max_target = 0.0f;
        max_target_cnt++;
        temp_min_target = min_target;
        if (min_target_cnt > 0) {
            sweep_tgt.max_time_m1 = temp_max_tgt_time;
            temp_max_tgt_time = max_tgt_time;
            sweep_tgt.count_m1 = min_target_cnt - 1;
            sweep_tgt.amplitude_m1 = temp_tgt_ampl;
            temp_tgt_ampl = temp_max_target - temp_min_target;
            push_to_tgt_buffer(min_target_cnt,temp_tgt_ampl,temp_max_tgt_time);
        }
                // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: min_tgt_cnt=%f", (double)(min_target_cnt));

    } else if (is_positive(prev_target) && !is_positive(tgt_rate) && new_target && now > new_tgt_time_ms && max_target_cnt > 0) {
        new_target = false;
        new_tgt_time_ms = now + half_cycle_time_ms;
        min_target_cnt++;
        temp_max_target = max_target;
        min_target = 0.0f;
                // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: min_tgt_cnt=%f", (double)(min_target_cnt));
    }

    // Indicates when the measured value (output) is positive or negative half of the cycle to notify when the max or min should be sought
    if (!is_positive(prev_meas) && is_positive(meas_rate) && !new_meas && now > new_meas_time_ms && max_target_cnt > 0) {
        new_meas = true;
        new_meas_time_ms = now + half_cycle_time_ms;
        // reset max_meas
        max_meas = 0.0f;
        max_meas_cnt++;
        temp_min_meas = min_meas;
        if (min_meas_cnt > 0 && min_target_cnt > 0) {
            sweep_meas.max_time_m1 = temp_max_meas_time;
            temp_max_meas_time = max_meas_time;
            sweep_meas.count_m1 = min_meas_cnt - 1;
            sweep_meas.amplitude_m1 = temp_meas_ampl;
            temp_meas_ampl = temp_max_meas - temp_min_meas;
            push_to_meas_buffer(min_meas_cnt,temp_meas_ampl,temp_max_meas_time);

            if (excitation == SWEEP) {
                float tgt_period = 0.001f * (temp_max_tgt_time - sweep_tgt.max_time_m1);
                if (!is_zero(tgt_period)) {
                    curr_test_freq = 6.28f / tgt_period;
                } else {
                    curr_test_freq = 0.0f;
                }
                if (!is_zero(sweep_tgt.amplitude_m1)) {
                    curr_test_gain = sweep_meas.amplitude_m1/sweep_tgt.amplitude_m1;
                } else {
                    curr_test_gain = 0.0f;
                }
                curr_test_phase = curr_test_freq * (float)(sweep_meas.max_time_m1 - sweep_tgt.max_time_m1) * 0.001f * 360.0f / 6.28f;
                // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f sweepgain=%f", (double)(curr_test_freq), (double)(curr_test_gain));
                cycle_complete = true;
            }
        } 
                // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: min_meas_cnt=%f", (double)(min_meas_cnt));
    } else if (is_positive(prev_meas) && !is_positive(meas_rate) && new_meas && now > new_meas_time_ms && max_meas_cnt > 0) {
        new_meas = false;
        new_meas_time_ms = now + half_cycle_time_ms;
        min_meas_cnt++;
        temp_max_meas = max_meas;
        min_meas = 0.0f;
                // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: min_meas_cnt=%f", (double)(min_meas_cnt));
    }

    if (tgt_rate > max_target && new_target) {
        max_target = tgt_rate;
        max_tgt_time = now;
    }

    if (tgt_rate < min_target && !new_target) {
        min_target = tgt_rate;
    }

    if (meas_rate > max_meas && new_meas) {
        max_meas = meas_rate;
        max_meas_time = now;
    }

    if (meas_rate < min_meas && !new_meas) {
        min_meas = meas_rate;
    }

    prev_target = tgt_rate;
    prev_meas = meas_rate;
}

// determine_gain_angle - this function receives time history data during a dwell and frequency sweep tests for angle_p tuning  
// and determines the gain, phase, and max acceleration of the response to the input. For dwell tests once the designated number  
// of cycles are complete, the average of the gain, phase, and max acceleration are determined over the last 5 cycles and the 
// cycles_complete flag is set. For frequency sweep tests, phase and gain are determined for every cycle and cycle_complete flag is set
// to indicate when to pull the phase and gain data.  The flag is reset to enable the next cycle to be analyzed.
void AC_AutoTune_FreqResp::determine_gain_angle(float command, float tgt_angle, float meas_angle, float tgt_freq)
{

    uint32_t now = AP_HAL::millis();
    float dt = 0.0025;
    uint32_t half_cycle_time_ms = 0;
    uint32_t cycle_time_ms = 0;

    if (cycle_complete) {
        return;
    }

    if (!is_zero(tgt_freq)) {
        half_cycle_time_ms = (uint32_t)(300 * 6.28 / tgt_freq);
        cycle_time_ms = (uint32_t)(1000 * 6.28 / tgt_freq);
    }

    if (input_start_time_ms == 0) {
        input_start_time_ms = now;
        prev_tgt_angle = tgt_angle;
        prev_meas_angle = meas_angle;
        prev_target = 0.0f;
        prev_meas = 0.0f;
    }

    target_rate = (tgt_angle - prev_tgt_angle) / dt;
    measured_rate = (meas_angle - prev_meas_angle) / dt;

    // cycles are complete! determine gain and phase and exit
    if (max_meas_cnt > AUTOTUNE_DWELL_CYCLES + 1 && max_target_cnt > AUTOTUNE_DWELL_CYCLES + 1 && excitation == DWELL) {
        float delta_time = 0.0f;
        float sum_gain = 0.0f;
        uint8_t cnt = 0;
        uint8_t gcnt = 0;
        uint16_t meas_cnt, tgt_cnt;
        float meas_ampl = 0.0f;
        float tgt_ampl = 0.0f;
        uint32_t meas_time = 0;
        uint32_t tgt_time = 0;
        for (int i = 0;  i < AUTOTUNE_DWELL_CYCLES; i++) {
            meas_cnt=0;
            tgt_cnt=0;
            pull_from_meas_buffer(meas_cnt, meas_ampl, meas_time);
            pull_from_tgt_buffer(tgt_cnt, tgt_ampl, tgt_time);
            push_to_meas_buffer(0, 0.0f, 0);
            push_to_tgt_buffer(0, 0.0f, 0);
            // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: tgt_cnt=%f meas_cnt=%f", (double)(tgt_cnt), (double)(meas_cnt));

            if (meas_cnt == tgt_cnt && meas_cnt != 0) {
                if (tgt_ampl > 0.0f) {
                    sum_gain += meas_ampl / tgt_ampl;
                    gcnt++;
                }
                float d_time = (float)(meas_time - tgt_time);
                if (d_time < 2.0f * (float)cycle_time_ms) {
                    delta_time += d_time;
                    cnt++;
                }
            } else if (meas_cnt > tgt_cnt) {
                pull_from_tgt_buffer(tgt_cnt, tgt_ampl, tgt_time);
                push_to_tgt_buffer(0, 0.0f, 0);
            } else if (meas_cnt < tgt_cnt) {
                pull_from_meas_buffer(meas_cnt, meas_ampl, meas_time);
                push_to_meas_buffer(0, 0.0f, 0);
            }                

        }
        if (gcnt > 0) {
            curr_test_gain = sum_gain / gcnt;
        }
        if (cnt > 0) {
            delta_time = delta_time / cnt;
        }
        curr_test_phase = tgt_freq * delta_time * 0.001f * 360.0f / 6.28f;
        if (curr_test_phase > 360.0f) {
            curr_test_phase = curr_test_phase - 360.0f;
        }
        float dwell_max_accel = tgt_freq * max_meas_rate * 5730.0f;
        if (!is_zero(max_command)) {
            // normalize accel for input size
            dwell_max_accel = dwell_max_accel / (2.0f * max_command);
        }
        if (dwell_max_accel > max_accel) {
            max_accel = dwell_max_accel;
        }
        curr_test_freq = tgt_freq;
        cycle_complete = true;
        // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: cycles completed");
        return;
    }

    // Indicates when the target(input) is positive or negative half of the cycle to notify when the max or min should be sought
    if (is_positive(prev_target) && !is_positive(target_rate) && !new_target && now > new_tgt_time_ms) {
        new_target = true;
        new_tgt_time_ms = now + half_cycle_time_ms;
        // reset max_target
        max_target = 0.0f;
        max_target_cnt++;
        temp_min_target = min_target;
        if (min_target_cnt > 0) {
            sweep_tgt.max_time_m1 = temp_max_tgt_time;
            temp_max_tgt_time = max_tgt_time;
            sweep_tgt.count_m1 = min_target_cnt - 1;
            sweep_tgt.amplitude_m1 = temp_tgt_ampl;
            temp_tgt_ampl = temp_max_target - temp_min_target;
            push_to_tgt_buffer(min_target_cnt,temp_tgt_ampl,temp_max_tgt_time);
        }
                // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: max_tgt_cnt=%f", (double)(max_target_cnt));

    } else if (!is_positive(prev_target) && is_positive(target_rate) && new_target && now > new_tgt_time_ms && max_target_cnt > 0) {
        new_target = false;
        new_tgt_time_ms = now + half_cycle_time_ms;
        min_target_cnt++;
        temp_max_target = max_target;
        min_target = 0.0f;
                // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: min_tgt_cnt=%f", (double)(min_target_cnt));
    }

    // Indicates when the measured value (output) is positive or negative half of the cycle to notify when the max or min should be sought
    if (is_positive(prev_meas) && !is_positive(measured_rate) && !new_meas && now > new_meas_time_ms && max_target_cnt > 0) {
        new_meas = true;
        new_meas_time_ms = now + half_cycle_time_ms;
        // reset max_meas
        max_meas = 0.0f;
        max_meas_cnt++;
        temp_min_meas = min_meas;
        if (min_meas_cnt > 0 && min_target_cnt > 0) {
            sweep_meas.max_time_m1 = temp_max_meas_time;
            temp_max_meas_time = max_meas_time;
            sweep_meas.count_m1 = min_meas_cnt - 1;
            sweep_meas.amplitude_m1 = temp_meas_ampl;
            temp_meas_ampl = temp_max_meas - temp_min_meas;
            push_to_meas_buffer(min_meas_cnt,temp_meas_ampl,temp_max_meas_time);

            if (excitation == SWEEP) {
                float tgt_period = 0.001f * (temp_max_tgt_time - sweep_tgt.max_time_m1);
                if (!is_zero(tgt_period)) {
                    curr_test_freq = 6.28f / tgt_period;
                } else {
                    curr_test_freq = 0.0f;
                }
                if (!is_zero(sweep_tgt.amplitude_m1)) {
                    curr_test_gain = sweep_meas.amplitude_m1/sweep_tgt.amplitude_m1;
                } else {
                    curr_test_gain = 0.0f;
                }
                curr_test_phase = curr_test_freq * (float)(sweep_meas.max_time_m1 - sweep_tgt.max_time_m1) * 0.001f * 360.0f / 6.28f;
               //gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: freq=%f gain=%f phase=%f", (double)(curr_test_freq), (double)(curr_test_gain), (double)(curr_test_phase));
                cycle_complete = true;
            }
        } 
                // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: max_meas_cnt=%f", (double)(max_meas_cnt));
    } else if (!is_positive(prev_meas) && is_positive(measured_rate) && new_meas && now > new_meas_time_ms && max_meas_cnt > 0) {
        new_meas = false;
        new_meas_time_ms = now + half_cycle_time_ms;
        min_meas_cnt++;
        temp_max_meas = max_meas;
        min_meas = 0.0f;
                // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: min_meas_cnt=%f", (double)(min_meas_cnt));
    }

    if (tgt_angle > max_target && new_target) {
        max_target = tgt_angle;
        max_tgt_time = now;
    }

    if (tgt_angle < min_target && !new_target) {
        min_target = tgt_angle;
    }

    if (meas_angle > max_meas && new_meas) {
        max_meas = meas_angle;
        max_meas_time = now;
    }

    if (meas_angle < min_meas && !new_meas) {
        min_meas = meas_angle;
    }

    if (now > (uint32_t)(input_start_time_ms + 7.0f * cycle_time_ms) && now < (uint32_t)(input_start_time_ms + 9.0f * cycle_time_ms)) {
        if (measured_rate > max_meas_rate) {
            max_meas_rate = measured_rate;
        }
        if (command > max_command) {
            max_command = command;
        }
    }

    prev_target = target_rate;
    prev_meas = measured_rate;
    prev_tgt_angle = tgt_angle;
    prev_meas_angle = meas_angle;
}

// push measured peak info to buffer
void AC_AutoTune_FreqResp::push_to_meas_buffer(uint16_t count, float amplitude, uint32_t time_ms)
{
    peak_info sample;
    sample.curr_count = count;
    sample.amplitude = amplitude;
    sample.time_ms = time_ms;
    meas_peak_info_buffer->push(sample);
}

// pull measured peak info from buffer
void AC_AutoTune_FreqResp::pull_from_meas_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms)
{
    peak_info sample;
    if (!meas_peak_info_buffer->pop(sample)) {
        // no sample
        return;
    }
    count = sample.curr_count;
    amplitude = sample.amplitude;
    time_ms = sample.time_ms;
}

// push target peak info to buffer
void AC_AutoTune_FreqResp::push_to_tgt_buffer(uint16_t count, float amplitude, uint32_t time_ms)
{
    peak_info sample;
    sample.curr_count = count;
    sample.amplitude = amplitude;
    sample.time_ms = time_ms;
    tgt_peak_info_buffer->push(sample);

}

// pull target peak info from buffer
void AC_AutoTune_FreqResp::pull_from_tgt_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms)
{
    peak_info sample;
    if (!tgt_peak_info_buffer->pop(sample)) {
        // no sample
        return;
    }
    count = sample.curr_count;
    amplitude = sample.amplitude;
    time_ms = sample.time_ms;
}
