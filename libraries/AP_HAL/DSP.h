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
/*
  interface to DSP device
 */
#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"

#define DSP_MEM_REGION AP_HAL::Util::MEM_FAST

class AP_HAL::DSP {
#if HAL_WITH_DSP
public:
    typedef float* FFTSampleWindow;
    class FFTWindowState {
    public:
        // frequency width of a FFT bin
        const float _bin_resolution;
        // number of FFT bins
        const uint16_t _bin_count;
        // size of the FFT window
        const uint16_t _window_size;
        // FFT data
        float* _freq_bins;
        // intermediate real FFT data
        float* _rfft_data;
        // estimate of FFT peak frequency
        float _max_bin_freq;
        // bin with maximum energy
        uint16_t _max_energy_bin;
        // width of the max energy peak
        float _max_noise_width_hz;
        // estimate of FFT second peak frequency
        float _second_bin_freq;
        // bin with second-most energy
        uint16_t _second_energy_bin;
        // width of the second energy peak
        float _second_noise_width_hz;
        // Hanning window for incoming samples, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
        float* _hanning_window;
        // Use in calculating the PS of the signal [Heinz] equations (20) & (21)
        float _window_scale;

        virtual ~FFTWindowState();
        FFTWindowState(uint16_t window_size, uint16_t sample_rate);
    };
    // initialise an FFT instance
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate) = 0;
    // start an FFT analysis
    virtual void fft_start(FFTWindowState* state, const float* samples, uint16_t buffer_index, uint16_t buffer_size) = 0;
    // perform remaining steps of an FFT analysis
    virtual uint16_t fft_analyse(FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, uint8_t harmonics, float noise_att_cutoff) = 0;

protected:
    // step 3: find the magnitudes of the complex data
    void step_cmplx_mag(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin, uint8_t harmonics, float noise_att_cutoff);
    // calculate the noise width of a peak based on the input parameters
    float find_noise_width(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin, uint16_t max_energy_bin, float cutoff, uint16_t& peak_top, uint16_t& peak_bottom) const;
    // step 4: find the bin with the highest energy and interpolate the required frequency
    uint16_t step_calc_frequencies(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin);
    // find the maximum value in an vector of floats
    virtual void vector_max_float(const float* vin, uint16_t len, float* max_value, uint16_t* max_index) const = 0;
    // multiple an vector of floats by a scale factor
    virtual void vector_scale_float(const float* vin, float scale, float* vout, uint16_t len) const = 0;

    // quinn's frequency interpolator
    float calculate_quinns_second_estimator(const FFTWindowState* fft, const float* complex_fft, uint16_t k) const;
    float tau(const float x) const;
#endif // HAL_WITH_DSP
};
