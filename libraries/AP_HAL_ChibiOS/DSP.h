/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andy Piper and the betaflight team
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_Namespace.h"

#if HAL_WITH_DSP

#include <arm_math.h>

#define DEBUG_FFT   0

// ChibiOS implementation of FFT analysis to run on STM32 processors
class ChibiOS::DSP : public AP_HAL::DSP {
public:
    // initialise an FFT instance
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size) override;
    // start an FFT analysis with an ObjectBuffer
    virtual void fft_start(FFTWindowState* state, FloatBuffer& samples, uint16_t advance) override;
    // perform remaining steps of an FFT analysis
    virtual uint16_t fft_analyse(FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff) override;

    // STM32-based FFT state
    class FFTWindowStateARM : public AP_HAL::DSP::FFTWindowState {
        friend class ChibiOS::DSP;
    public:
        FFTWindowStateARM(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size);
        virtual ~FFTWindowStateARM();

    private:
        // underlying CMSIS data structure for FFT analysis
        arm_rfft_fast_instance_f32 _fft_instance;
    };

protected:
    void vector_max_float(const float* vin, uint16_t len, float* maxValue, uint16_t* maxIndex) const override {
        uint32_t mindex;
        arm_max_f32(vin, len, maxValue, &mindex);
        *maxIndex = mindex;
    }
    void vector_scale_float(const float* vin, float scale, float* vout, uint16_t len) const override {
        arm_scale_f32(vin, scale, vout, len);
    }
    float vector_mean_float(const float* vin, uint16_t len) const override {
        float mean_value;
        arm_mean_f32(vin, len, &mean_value);
        return mean_value;
    }
    void vector_add_float(const float* vin1, const float* vin2, float* vout, uint16_t len) const override {
        arm_add_f32(vin1, vin2, vout, len);
    }

private:
    // following are the six independent steps for calculating an FFT
    void step_hanning(FFTWindowStateARM* fft, FloatBuffer& samples, uint16_t advance);
    void step_arm_cfft_f32(FFTWindowStateARM* fft);
    void step_bitreversal(FFTWindowStateARM* fft);
    void step_stage_rfft_f32(FFTWindowStateARM* fft);
    void step_arm_cmplx_mag_f32(FFTWindowStateARM* fft, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff);
    uint16_t step_calc_frequencies_f32(FFTWindowStateARM* fft, uint16_t start_bin, uint16_t end_bin);
    // candan's frequency interpolator
    float calculate_candans_estimator(const FFTWindowStateARM* fft, uint16_t k) const;

#if DEBUG_FFT
    class StepTimer {
    public:
        uint32_t _timer_total;
        uint32_t _timer_avg;
        uint8_t _time_ticks;

        void time(uint32_t start);
    };

    uint32_t  _output_count;
    StepTimer _hanning_timer;
    StepTimer _arm_cfft_f32_timer;
    StepTimer _bitreversal_timer;
    StepTimer _stage_rfft_f32_timer;
    StepTimer _arm_cmplx_mag_f32_timer;
    StepTimer _step_calc_frequencies;
#endif
};

#endif