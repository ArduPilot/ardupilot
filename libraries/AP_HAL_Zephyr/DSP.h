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
 * Ported from AP_HAL_ChibiOS/DSP.h (Andy Piper and the betaflight team) -
 * see DSP.cpp for the porting notes.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_DSP

#include <arm_math.h>

#define DEBUG_FFT 0

namespace Zephyr {

// CMSIS-DSP-backed FFT analysis, for gyro vibration frequency tracking
// (AP_GyroFFT / dynamic harmonic notch). Cortex-M7 only (RT1176,
// CubeOrangeZephyr) - see DSP.cpp.
class DSP : public AP_HAL::DSP {
public:
    FFTWindowState *fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size) override;
    void fft_start(FFTWindowState *state, FloatBuffer &samples, uint16_t advance) override;
    uint16_t fft_analyse(FFTWindowState *state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff) override;

    class FFTWindowStateARM : public AP_HAL::DSP::FFTWindowState {
        friend class Zephyr::DSP;
    public:
        FFTWindowStateARM(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size);
        virtual ~FFTWindowStateARM();

    private:
        // underlying CMSIS-DSP data structure for FFT analysis
        arm_rfft_fast_instance_f32 _fft_instance;
    };

protected:
    void vector_max_float(const float *vin, uint16_t len, float *maxValue, uint16_t *maxIndex) const override
    {
        uint32_t mindex;
        arm_max_f32(vin, len, maxValue, &mindex);
        *maxIndex = mindex;
    }
    void vector_scale_float(const float *vin, float scale, float *vout, uint16_t len) const override
    {
        arm_scale_f32(vin, scale, vout, len);
    }
    float vector_mean_float(const float *vin, uint16_t len) const override
    {
        float mean_value;
        arm_mean_f32(vin, len, &mean_value);
        return mean_value;
    }
    void vector_add_float(const float *vin1, const float *vin2, float *vout, uint16_t len) const override
    {
        arm_add_f32(vin1, vin2, vout, len);
    }

private:
    void step_hanning(FFTWindowStateARM *fft, FloatBuffer &samples, uint16_t advance);
    // real FFT - a single call into CMSIS-DSP's public arm_rfft_fast_f32(),
    // not ChibiOS's manual cfft+bitreversal+stage_rfft breakdown; see DSP.cpp.
    void step_rfft_f32(FFTWindowStateARM *fft);
    void step_arm_cmplx_mag_f32(FFTWindowStateARM *fft, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff);
    uint16_t step_calc_frequencies_f32(FFTWindowStateARM *fft, uint16_t start_bin, uint16_t end_bin);
    // Candan's frequency interpolator - ported for parity, unused in this
    // file just as it is unused in AP_HAL_ChibiOS/DSP.cpp (nothing calls
    // it there either; step_calc_frequencies() in the shared AP_HAL::DSP
    // base class does its own interpolation).
    float calculate_candans_estimator(const FFTWindowStateARM *fft, uint16_t k) const;

#if DEBUG_FFT
    class StepTimer {
    public:
        uint32_t _timer_total;
        uint32_t _timer_avg;
        uint8_t _time_ticks;

        void time(uint32_t start);
    };

    uint32_t _output_count;
    StepTimer _hanning_timer;
    StepTimer _rfft_f32_timer;
    StepTimer _arm_cmplx_mag_f32_timer;
    StepTimer _step_calc_frequencies;
#endif
};

}  // namespace Zephyr

#endif  // HAL_WITH_DSP
