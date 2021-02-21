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
 * Code by Andy Piper
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_SITL.h"

#include <complex>

typedef std::complex<float> complexf;

// ChibiOS implementation of FFT analysis to run on STM32 processors
class HALSITL::DSP : public AP_HAL::DSP {
public:
    // initialise an FFT instance
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t harmonics) override;
    // start an FFT analysis with an ObjectBuffer
    virtual void fft_start(FFTWindowState* state, FloatBuffer& samples, uint16_t advance) override;
    // perform remaining steps of an FFT analysis
    virtual uint16_t fft_analyse(FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff) override;

    // STM32-based FFT state
    class FFTWindowStateSITL : public AP_HAL::DSP::FFTWindowState {
        friend class HALSITL::DSP;

    public:
        FFTWindowStateSITL(uint16_t window_size, uint16_t sample_rate, uint8_t harmonics);
        virtual ~FFTWindowStateSITL();

    private:
        complexf* buf;
    };

private:
    void step_hanning(FFTWindowStateSITL* fft, FloatBuffer& samples, uint16_t advance);
    void step_fft(FFTWindowStateSITL* fft);
    void mult_f32(const float* v1, const float* v2, float* vout, uint16_t len);
    void vector_max_float(const float* vin, uint16_t len, float* maxValue, uint16_t* maxIndex) const override;
    void vector_scale_float(const float* vin, float scale, float* vout, uint16_t len) const override;
    float vector_mean_float(const float* vin, uint16_t len) const override;
    void calculate_fft(complexf* f, uint16_t length);
};
