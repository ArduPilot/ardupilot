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

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_DSP

#include "AP_HAL_SITL.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "DSP.h"
#include <cmath>
#include <assert.h>

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

// The algorithms originally came from betaflight but are now substantially modified based on theory and experiment.
// https://holometer.fnal.gov/GH_FFT.pdf "Spectrum and spectral density estimation by the Discrete Fourier transform (DFT),
// including a comprehensive list of window functions and some new flat-top windows." - Heinzel et. al is a great reference
// for understanding the underlying theory although we do not use spectral density here since time resolution is equally
// important as frequency resolution. Referred to as [Heinz] throughout the code.

// initialize the FFT state machine
AP_HAL::DSP::FFTWindowState* DSP::fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size)
{
    DSP::FFTWindowStateSITL* fft = new DSP::FFTWindowStateSITL(window_size, sample_rate, sliding_window_size);
    if (fft == nullptr || fft->_hanning_window == nullptr || fft->_rfft_data == nullptr || fft->_freq_bins == nullptr || fft->_derivative_freq_bins == nullptr) {
        delete fft;
        return nullptr;
    }
    return fft;
}

// start an FFT analysis
void DSP::fft_start(AP_HAL::DSP::FFTWindowState* state, FloatBuffer& samples, uint16_t advance)
{
    step_hanning((FFTWindowStateSITL*)state, samples, advance);
}

// perform remaining steps of an FFT analysis
uint16_t DSP::fft_analyse(AP_HAL::DSP::FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff)
{
    FFTWindowStateSITL* fft = (FFTWindowStateSITL*)state;
    step_fft(fft);
    step_cmplx_mag(fft, start_bin, end_bin, noise_att_cutoff);
    return step_calc_frequencies(fft, start_bin, end_bin);
}

// create an instance of the FFT state machine
DSP::FFTWindowStateSITL::FFTWindowStateSITL(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size)
    : AP_HAL::DSP::FFTWindowState::FFTWindowState(window_size, sample_rate, sliding_window_size)
{
    if (_freq_bins == nullptr || _hanning_window == nullptr || _rfft_data == nullptr || _derivative_freq_bins == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Failed to allocate window for DSP");
        return;
    }

    buf = new complexf[window_size];
}

DSP::FFTWindowStateSITL::~FFTWindowStateSITL()
{
    delete[] buf;
}

// step 1: filter the incoming samples through a Hanning window
void DSP::step_hanning(FFTWindowStateSITL* fft, FloatBuffer& samples, uint16_t advance)
{
    // 5us
    // apply hanning window to gyro samples and store result in _freq_bins
    // hanning starts and ends with 0, could be skipped for minor speed improvement
    uint32_t read_window = samples.peek(&fft->_freq_bins[0], fft->_window_size);
    if (read_window != fft->_window_size) {
        return;
    }
    samples.advance(advance);
    mult_f32(&fft->_freq_bins[0], &fft->_hanning_window[0], &fft->_freq_bins[0], fft->_window_size);
}

// step 2: perform an in-place FFT on the windowed data
void DSP::step_fft(FFTWindowStateSITL* fft)
{
    for (uint16_t i = 0; i < fft->_window_size; i++) {
        fft->buf[i] = complexf(fft->_freq_bins[i], 0);
    }

    calculate_fft(fft->buf, fft->_window_size);

    for (uint16_t i = 0; i < fft->_bin_count; i++) {
        fft->_freq_bins[i] = std::norm(fft->buf[i]);
    }

    // components at the nyquist frequency are real only
    for (uint16_t i = 0, j = 0; i <= fft->_bin_count; i++, j += 2) {
        fft->_rfft_data[j] = fft->buf[i].real();
        fft->_rfft_data[j+1] = fft->buf[i].imag();
    }
}

void DSP::mult_f32(const float* v1, const float* v2, float* vout, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        vout[i] = v1[i] * v2[i];
    }
}

void DSP::vector_max_float(const float* vin, uint16_t len, float* maxValue, uint16_t* maxIndex) const
{
    *maxValue = vin[0];
    *maxIndex = 0;
    for (uint16_t i = 1; i < len; i++) {
        if (vin[i] > *maxValue) {
            *maxValue = vin[i];
            *maxIndex = i;
        }
    }
}

void DSP::vector_scale_float(const float* vin, float scale, float* vout, uint16_t len) const
{
    for (uint16_t i = 0; i < len; i++) {
        vout[i] = vin[i] * scale;
    }
}

void DSP::vector_add_float(const float* vin1, const float* vin2, float* vout, uint16_t len) const
{
    for (uint16_t i = 0; i < len; i++) {
        vout[i] = vin1[i] + vin2[i];
    }
}

float DSP::vector_mean_float(const float* vin, uint16_t len) const
{
    float mean_value = 0.0f;
    for (uint16_t i = 0; i < len; i++) {
        mean_value += vin[i];
    }
    mean_value /= len;
    return mean_value;
}

// simple integer log2
static uint16_t fft_log2(uint16_t n)
{
    uint16_t k = n, i = 0;
    while (k) {
        k >>= 1;
        i++;
    }
    return i - 1;
}

// calculate the in-place FFT of the input using the Cooley–Tukey algorithm
// this is a translation of Ron Nicholson's version in http://www.nicholson.com/dsp.fft1.html
void DSP::calculate_fft(complexf *samples, uint16_t fftlen)
{
    uint16_t m = fft_log2(fftlen);
    // shuffle data using bit reversed addressing ***
    for (uint16_t k = 0; k < fftlen; k++) {
        // generate a bit reversed address for samples[k] ***
        uint16_t ki = k, kr = 0;
        for (uint16_t i=1; i<=m; i++) {
            kr <<= 1; //  left shift result kr by 1 bit
            if (ki % 2 == 1) {
                kr++;
            }
            ki >>= 1; // right shift temp ki by 1 bit
        }
        // swap data samples[k] to bit reversed address samples[kr]
        if (kr > k) {
            complexf t = samples[kr];
            samples[kr] = samples[k];
            samples[k] = t;
        }
    }

    // do fft butterflys in place
    uint16_t istep = 2;
    while (istep <= fftlen) {// layers 2,4,8,16, ... ,n
        uint16_t is2 = istep / 2;
        uint16_t astep = fftlen / istep;
        for (uint16_t km = 0; km < is2; km++) { // outer row loop
            uint16_t a  = km * astep; // twiddle angle index
            complexf w(sinf(2 * M_PI * (a+(fftlen/4)) / fftlen), sinf(2 * M_PI * a / fftlen));
            for (uint16_t ki = 0; ki <= (fftlen - istep); ki += istep) { // inner column loop
                uint16_t i = km + ki;
                uint16_t j = is2 + i;
                complexf t = w * samples[j];
                complexf q = samples[i];
                samples[j] = q - t;
                samples[i] = q + t;
            }
        }
        istep <<= 1;
    }
}

#endif
