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
#include <AP_HAL/utility/RingBuffer.h>

#define DSP_MEM_REGION AP_HAL::Util::MEM_FAST
// Maximum tolerated number of cycles with missing signal
#define FFT_MAX_MISSED_UPDATES 5

class AP_HAL::DSP {
#if HAL_WITH_DSP
public:
    enum FrequencyPeak : uint8_t {
        CENTER = 0,
        LOWER_SHOULDER = 1,
        UPPER_SHOULDER = 2,
        MAX_TRACKED_PEAKS = 3,
        NONE = 4
    };

    struct FrequencyPeakData {
        // estimate of FFT peak frequency
        float _freq_hz;
        // FFT bin with maximum energy
        uint16_t _bin;
        // width of the peak
        float _noise_width_hz;
    };

    class FFTWindowState {
    public:
        // frequency width of a FFT bin
        const float _bin_resolution;
        // number of FFT bins
        const uint16_t _bin_count;
        // size of the FFT window
        const uint16_t _window_size;
        // number of harmonics of interest
        const uint8_t _harmonics;
        // FFT data
        float* _freq_bins;
        // derivative real data scratch space
        float* _derivative_freq_bins;
        // intermediate real FFT data
        float* _rfft_data;
        // three highest peaks
        FrequencyPeakData _peak_data[MAX_TRACKED_PEAKS];
        // Hanning window for incoming samples, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
        float* _hanning_window;
        // Use in calculating the PS of the signal [Heinz] equations (20) & (21)
        float _window_scale;

        virtual ~FFTWindowState();
        FFTWindowState(uint16_t window_size, uint16_t sample_rate, uint8_t harmonics);
    };
    // initialise an FFT instance
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t harmonics) = 0;
    // start an FFT analysis with an ObjectBuffer
    virtual void fft_start(FFTWindowState* state, FloatBuffer& samples, uint16_t advance) = 0;
    // perform remaining steps of an FFT analysis
    virtual uint16_t fft_analyse(FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff) = 0;

protected:
    // step 3: find the magnitudes of the complex data
    void step_cmplx_mag(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff);
    // calculate the noise width of a peak based on the input parameters
    float find_noise_width(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin, uint16_t max_energy_bin, float cutoff, uint16_t& peak_top, uint16_t& peak_bottom) const;
    // step 4: find the bin with the highest energy and interpolate the required frequency
    uint16_t step_calc_frequencies(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin);
    // calculate a single frequency
    uint16_t calc_frequency(FFTWindowState* fft, uint16_t start_bin, uint16_t peak_bin, uint16_t end_bin);
    // find the maximum value in an vector of floats
    virtual void vector_max_float(const float* vin, uint16_t len, float* max_value, uint16_t* max_index) const = 0;
    // find the mean value in an vector of floats
    virtual float vector_mean_float(const float* vin, uint16_t len) const = 0;
    // multiple an vector of floats by a scale factor
    virtual void vector_scale_float(const float* vin, float scale, float* vout, uint16_t len) const = 0;
    // algorithm for finding peaks in noisy data as per https://terpconnect.umd.edu/~toh/spectrum/PeakFindingandMeasurement.htm
    uint16_t find_peaks(const float* input, uint16_t length, float* output, uint16_t* peaks, uint16_t peaklen, 
        float slopeThreshold, float ampThreshold, uint16_t smoothwidth, uint16_t peakgroup) const;
    uint16_t val2index(const float* vector, uint16_t n, float val) const;
    void derivative(const float* input, float* output, uint16_t n) const;
    void fastsmooth(float* input, uint16_t n, uint16_t smoothwidth) const;

    // quinn's frequency interpolator
    float calculate_quinns_second_estimator(const FFTWindowState* fft, const float* complex_fft, uint16_t k) const;
    float tau(const float x) const;
#endif // HAL_WITH_DSP
};
