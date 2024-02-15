/*
This library receives time history data (angular rate or angle) during a dwell test or frequency sweep test and determines the gain and phase of the response to the input. For dwell tests once the designated number of cycles are complete, the average of the gain and phase are determined over the last 5 cycles and the cycle_complete flag is set. For frequency sweep tests, phase and gain are determined for every cycle and cycle_complete flag is set to indicate when to pull the phase and gain data.  The flag is reset to enable the next cycle to be analyzed.  The init function must be used when initializing the dwell or frequency sweep test.
*/

#include <AP_HAL/AP_HAL.h>
#include "AC_AutoTune_FreqResp.h"

// Initialize the Frequency Response Object. Must be called before running dwell or frequency sweep tests
void AC_AutoTune_FreqResp::init(InputType input_type, ResponseType response_type)
{
    excitation = input_type;
    response = response_type;
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
    meas_peak_info_buffer.clear();
    tgt_peak_info_buffer.clear();
    cycle_complete = false;
}

// update_angle - this function receives time history data during a dwell and frequency sweep tests for angle_p tuning
// and determines the gain, phase, and max acceleration of the response to the input. For dwell tests once the designated number
// of cycles are complete, the average of the gain, phase, and max acceleration are determined over the last 5 cycles and the
// cycles_complete flag is set. For frequency sweep tests, phase and gain are determined for every cycle and cycle_complete flag is set
// to indicate when to pull the phase and gain data.  The flag is reset to enable the next cycle to be analyzed.
void AC_AutoTune_FreqResp::update(float command, float tgt_resp, float meas_resp, float tgt_freq)
{

    uint32_t now = AP_HAL::millis();
    float dt = 0.0025;
    uint32_t half_cycle_time_ms = 0;
    uint32_t cycle_time_ms = 0;

    if (cycle_complete) {
        return;
    }

    if (!is_zero(tgt_freq)) {
        half_cycle_time_ms = (uint32_t)(300 * M_2PI / tgt_freq);
        cycle_time_ms = (uint32_t)(1000 * M_2PI / tgt_freq);
    }

    if (input_start_time_ms == 0) {
        input_start_time_ms = now;
        if (response == ANGLE) {
            prev_tgt_resp = tgt_resp;
            prev_meas_resp = meas_resp;
            prev_target = 0.0f;
            prev_meas = 0.0f;
        }
    }

    if (response == ANGLE) {
        target_rate = (tgt_resp - prev_tgt_resp) / dt;
        measured_rate = (meas_resp - prev_meas_resp) / dt;
    } else {
        target_rate = tgt_resp;
        measured_rate = meas_resp;
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
        for (uint8_t i = 0;  i < AUTOTUNE_DWELL_CYCLES; i++) {
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
        curr_test_phase = tgt_freq * delta_time * 0.001f * 360.0f / M_2PI;
        if (curr_test_phase > 360.0f) {
            curr_test_phase = curr_test_phase - 360.0f;
        }

        // determine max accel for angle response type test
        float dwell_max_accel;
        if (response == ANGLE) {
            dwell_max_accel = tgt_freq * max_meas_rate * 5730.0f;
            if (!is_zero(max_command)) {
                // normalize accel for input size
                dwell_max_accel = dwell_max_accel / (2.0f * max_command);
            }
            if (dwell_max_accel > max_accel) {
                max_accel = dwell_max_accel;
            }
        }

        curr_test_freq = tgt_freq;
        cycle_complete = true;
        // gcs().send_text(MAV_SEVERITY_INFO, "AutoTune: cycles completed");
        return;
    }

    // Indicates when the target(input) is positive or negative half of the cycle to notify when the max or min should be sought
    if (((response == ANGLE && is_positive(prev_target) && !is_positive(target_rate))
        || (response == RATE && !is_positive(prev_target) && is_positive(target_rate)))
        && !new_target && now > new_tgt_time_ms) {
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

    } else if (((response == ANGLE && !is_positive(prev_target) && is_positive(target_rate))
               || (response == RATE && is_positive(prev_target) && !is_positive(target_rate)))
               && new_target && now > new_tgt_time_ms && max_target_cnt > 0) {
        new_target = false;
        new_tgt_time_ms = now + half_cycle_time_ms;
        min_target_cnt++;
        temp_max_target = max_target;
        min_target = 0.0f;
    }

    // Indicates when the measured value (output) is positive or negative half of the cycle to notify when the max or min should be sought
    if (((response == ANGLE && is_positive(prev_meas) && !is_positive(measured_rate))
         || (response == RATE && !is_positive(prev_meas) && is_positive(measured_rate)))
         && !new_meas && now > new_meas_time_ms && max_target_cnt > 0) {
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
                    curr_test_freq = M_2PI / tgt_period;
                } else {
                    curr_test_freq = 0.0f;
                }
                if (!is_zero(sweep_tgt.amplitude_m1)) {
                    curr_test_gain = sweep_meas.amplitude_m1/sweep_tgt.amplitude_m1;
                } else {
                    curr_test_gain = 0.0f;
                }
                curr_test_phase = curr_test_freq * (float)(sweep_meas.max_time_m1 - sweep_tgt.max_time_m1) * 0.001f * 360.0f / M_2PI;
                cycle_complete = true;
            }
        } 
    } else if (((response == ANGLE && !is_positive(prev_meas) && is_positive(measured_rate))
                || (response == RATE && is_positive(prev_meas) && !is_positive(measured_rate)))
                && new_meas && now > new_meas_time_ms && max_meas_cnt > 0) {
        new_meas = false;
        new_meas_time_ms = now + half_cycle_time_ms;
        min_meas_cnt++;
        temp_max_meas = max_meas;
        min_meas = 0.0f;
    }

    if (new_target) {
        if (tgt_resp > max_target) {
            max_target = tgt_resp;
            max_tgt_time = now;
        }
    } else {
        if (tgt_resp < min_target) {
            min_target = tgt_resp;
        }
    }

    if (new_meas) {
        if (meas_resp > max_meas) {
            max_meas = meas_resp;
            max_meas_time = now;
        }
    } else {
        if (meas_resp < min_meas) {
            min_meas = meas_resp;
        }
    }

    if (response == ANGLE) {
        if (now > (uint32_t)(input_start_time_ms + 7.0f * cycle_time_ms) && now < (uint32_t)(input_start_time_ms + 9.0f * cycle_time_ms)) {
            if (measured_rate > max_meas_rate) {
                max_meas_rate = measured_rate;
            }
            if (command > max_command) {
                max_command = command;
            }
        }
        prev_tgt_resp = tgt_resp;
        prev_meas_resp = meas_resp;
    }

    prev_target = target_rate;
    prev_meas = measured_rate;
}

// push measured peak info to buffer
void AC_AutoTune_FreqResp::push_to_meas_buffer(uint16_t count, float amplitude, uint32_t time_ms)
{
    peak_info sample;
    sample.curr_count = count;
    sample.amplitude = amplitude;
    sample.time_ms = time_ms;
    meas_peak_info_buffer.push(sample);
}

// pull measured peak info from buffer
void AC_AutoTune_FreqResp::pull_from_meas_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms)
{
    peak_info sample;
    if (!meas_peak_info_buffer.pop(sample)) {
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
    tgt_peak_info_buffer.push(sample);

}

// pull target peak info from buffer
void AC_AutoTune_FreqResp::pull_from_tgt_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms)
{
    peak_info sample;
    if (!tgt_peak_info_buffer.pop(sample)) {
        // no sample
        return;
    }
    count = sample.curr_count;
    amplitude = sample.amplitude;
    time_ms = sample.time_ms;
}
