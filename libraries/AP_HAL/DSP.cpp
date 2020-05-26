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

#include <AP_Math/AP_Math.h>
#include "AP_HAL.h"
#include "DSP.h"

#if HAL_WITH_DSP

using namespace AP_HAL;

extern const AP_HAL::HAL &hal;

#define SQRT_2_3 0.816496580927726f
#define SQRT_6   2.449489742783178f

DSP::FFTWindowState::FFTWindowState(uint16_t window_size, uint16_t sample_rate)
    : _window_size(window_size),
    _bin_count(window_size / 2),
    _bin_resolution((float)sample_rate / (float)window_size)
{
    // includes DC ad Nyquist components and needs to be large enough for intermediate steps
    _freq_bins = (float*)hal.util->malloc_type(sizeof(float) * (window_size), DSP_MEM_REGION);
    _hanning_window = (float*)hal.util->malloc_type(sizeof(float) * (window_size), DSP_MEM_REGION);
    // allocate workspace, including Nyquist component
    _rfft_data = (float*)hal.util->malloc_type(sizeof(float) * (_window_size + 2), DSP_MEM_REGION);

    if (_freq_bins == nullptr || _hanning_window == nullptr || _rfft_data == nullptr) {
        hal.util->free_type(_freq_bins, sizeof(float) * (_window_size), DSP_MEM_REGION);
        hal.util->free_type(_hanning_window, sizeof(float) * (_window_size), DSP_MEM_REGION);
        hal.util->free_type(_rfft_data, sizeof(float) * (_window_size + 2), DSP_MEM_REGION);

        _freq_bins = nullptr;
        _hanning_window = nullptr;
        _rfft_data = nullptr;
        return;
    }

    // create the Hanning window
    // https://holometer.fnal.gov/GH_FFT.pdf - equation 19
    for (uint16_t i = 0; i < window_size; i++) {
        _hanning_window[i] = (0.5f - 0.5f * cosf(2.0f * M_PI * i / ((float)window_size - 1)));
        _window_scale += _hanning_window[i];
    }
    // Calculate the inverse of the Effective Noise Bandwidth
    _window_scale = 2.0f / sq(_window_scale);
}

DSP::FFTWindowState::~FFTWindowState()
{
    hal.util->free_type(_freq_bins, sizeof(float) * (_window_size), DSP_MEM_REGION);
    _freq_bins = nullptr;
    hal.util->free_type(_hanning_window, sizeof(float) * (_window_size), DSP_MEM_REGION);
    _hanning_window = nullptr;
    hal.util->free_type(_rfft_data, sizeof(float) * (_window_size + 2), DSP_MEM_REGION);
    _rfft_data = nullptr;
}

// step 3: find the magnitudes of the complex data
void DSP::step_cmplx_mag(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin, uint8_t harmonics, float noise_att_cutoff)
{
    // find the maximum power in the range we are interested in
    float max_value = 0, max_value2 = 0, max_value3 = 0;
    uint16_t max_bin2 = 0, max_bin3 = 0;
    uint16_t bin_range = (end_bin - start_bin) + 1;
    // calculate highest two peaks in the range of interest. we cannot simply find the
    // maximum in two halves since the primary peak could extend over multiple bins
    // instead move outwards looking for the 3dB points and then search from there

    // first find the highest peak
    vector_max_float(&fft->_freq_bins[start_bin], bin_range, &max_value, &fft->_max_energy_bin);
    fft->_max_energy_bin += start_bin;

    // calculate the width of the peak
    uint16_t top = 0, bottom = 0;
    fft->_max_noise_width_hz = find_noise_width(fft, start_bin, end_bin, fft->_max_energy_bin, noise_att_cutoff, top, bottom);

    // if requested calculate another harmonic
    if (harmonics > 1) {
        // search for peaks above the 3db point
        if (top < end_bin) {
            vector_max_float(&fft->_freq_bins[top], end_bin - top + 1, &max_value2, &max_bin2);
        }
        max_bin2 += top;
        // search for peaks below the 3db point
        if (bottom > start_bin) {
            vector_max_float(&fft->_freq_bins[start_bin], bottom - start_bin + 1, &max_value3, &max_bin3);
        }
        max_bin3 += start_bin;

        // pick the highest
        if (fft->_freq_bins[max_bin2] > fft->_freq_bins[max_bin3]) {
            fft->_second_energy_bin = max_bin2;
            // calculate the noise width of the second bin
            fft->_second_noise_width_hz = find_noise_width(fft, top, end_bin, fft->_second_energy_bin, noise_att_cutoff, top, bottom);
        } else {
            fft->_second_energy_bin = max_bin3;
            // calculate the noise width of the second bin
            fft->_second_noise_width_hz = find_noise_width(fft, start_bin, bottom, fft->_second_energy_bin, noise_att_cutoff, top, bottom);
        }
    } else {
        fft->_second_energy_bin = 0;
        fft->_second_noise_width_hz = 0;
    }

    // scale the power to account for the input window
    vector_scale_float(fft->_freq_bins, fft->_window_scale, fft->_freq_bins, fft->_bin_count);
}

// calculate the noise width of a peak based on the input parameters
float DSP::find_noise_width(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin, uint16_t max_energy_bin, float cutoff, uint16_t& peak_top, uint16_t& peak_bottom) const
{
    peak_top = peak_bottom = start_bin;

    if (max_energy_bin == 0) {
        return  fft->_bin_resolution;
    }

    if (max_energy_bin == fft->_bin_count) {
        peak_top = peak_bottom = fft->_bin_count;
        return  fft->_bin_resolution;
    }

    // calculate the width of the peak
    float noise_width_hz = 1;

    // -attenuation/2 dB point above the center bin
    for (uint16_t b = max_energy_bin + 1; b <= end_bin; b++) {
        if (fft->_freq_bins[b] < fft->_freq_bins[max_energy_bin] * cutoff) {
            // we assume that the 3dB point is in the middle of the final shoulder bin
            noise_width_hz += (b - max_energy_bin - 0.5f);
            peak_top = b;
            break;
        }
    }
    // -attenuation/2 dB point below the center bin
    for (uint16_t b = max_energy_bin - 1; b >= start_bin; b--) {
        if (fft->_freq_bins[b] < fft->_freq_bins[max_energy_bin] * cutoff) {
            // we assume that the 3dB point is in the middle of the final shoulder bin
            noise_width_hz += (max_energy_bin - b - 0.5f);
            peak_bottom = b;
            break;
        }
    }
    noise_width_hz *= fft->_bin_resolution;

    return noise_width_hz;
}

// step 4: find the bin with the highest energy and interpolate the required frequency
uint16_t DSP::step_calc_frequencies(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin)
{
    if (is_zero(fft->_freq_bins[fft->_max_energy_bin])) {
        fft->_max_bin_freq = start_bin * fft->_bin_resolution;
    } else {
        // It turns out that Jain is pretty good and works with only magnitudes, but Candan is significantly better
        // if you have access to the complex values and Quinn is a little better still. Quinn is computationally
        // more expensive, but compared to the overall FFT cost seems worth it.
        fft->_max_bin_freq = (fft->_max_energy_bin + calculate_quinns_second_estimator(fft, fft->_rfft_data, fft->_max_energy_bin)) * fft->_bin_resolution;
    }

    // calculate second frequency as required
    if (fft->_second_energy_bin > 0) {
        // find second highest bin frequency
        if (is_zero(fft->_freq_bins[fft->_second_energy_bin])) {
            fft->_second_bin_freq = start_bin * fft->_bin_resolution;
        } else {
            fft->_second_bin_freq = (fft->_second_energy_bin + calculate_quinns_second_estimator(fft, fft->_rfft_data, fft->_second_energy_bin)) * fft->_bin_resolution;
        }
    }

    return fft->_max_energy_bin;
}

// Interpolate center frequency using https://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/
float DSP::calculate_quinns_second_estimator(const FFTWindowState* fft, const float* complex_fft, uint16_t k_max) const
{
    if (k_max <= 1 || k_max >= fft->_bin_count) {
        return 0.0f;
    }

    const uint16_t k_m1 = (k_max - 1) * 2;
    const uint16_t k_p1 = (k_max + 1) * 2;
    const uint16_t k = k_max * 2;

    const float divider = complex_fft[k] * complex_fft[k] + complex_fft[k+1] * complex_fft[k+1];
    const float ap = (complex_fft[k_p1] * complex_fft[k] + complex_fft[k_p1 + 1] * complex_fft[k+1]) / divider;
    const float am = (complex_fft[k_m1] * complex_fft[k] + complex_fft[k_m1 + 1] * complex_fft[k + 1]) / divider;

    // sanity check
    if (fabsf(1.0f - ap) < 0.01f || fabsf(1.0f - am) < 0.01f) {
        return 0.0f;
    }

    const float dp = -ap / (1.0f - ap);
    const float dm = am / (1.0f - am);

    float d = (dp + dm) * 0.5f + tau(dp * dp) - tau(dm * dm);

    // -0.5 < d < 0.5 which is the fraction of the sample spacing about the center element
    return constrain_float(d, -0.5f, 0.5f);
}

static const float TAU_FACTOR = SQRT_6 / 24.0f;

// Helper function used for Quinn's frequency estimation
float DSP::tau(const float x) const
{
    float p1 = logf(3.0f * sq(x) + 6.0f * x + 1.0f);
    float part1 = x + 1.0f - SQRT_2_3;
    float part2 = x + 1.0f + SQRT_2_3;
    float p2 = logf(part1 / part2);
    return (0.25f * p1 - TAU_FACTOR * p2);
}

#endif // HAL_WITH_DSP
