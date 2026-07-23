/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HAL_DEBUG_BUILD
#define AP_INLINE_VECTOR_OPS
#pragma GCC optimize("O2")
#endif

#include "NotchFilter.h"
#include <AP_Logger/AP_Logger.h>

const static float NOTCH_MAX_SLEW       = 0.05f;
const static float NOTCH_MAX_SLEW_LOWER = 1.0f - NOTCH_MAX_SLEW;
const static float NOTCH_MAX_SLEW_UPPER = 1.0f / NOTCH_MAX_SLEW_LOWER;

/*
  constructor
 */
template <class T>
NotchFilter<T>::NotchFilter() :
    initialised(false),
    need_reset(false),
    b0(0), b1(0), b2(0), a1(0), a2(0),
    _center_freq_hz(0),
    _sample_freq_hz(0),
    _A(0),
    _harmonics(1),
    _num_harmonics(1),
    _harmonic_filters(nullptr)
{
}

/*
  destructor
 */
template <class T>
NotchFilter<T>::~NotchFilter()
{
    if (_harmonic_filters != nullptr) {
        delete[] _harmonic_filters;
        _harmonic_filters = nullptr;
    }
}

/*
   calculate the attenuation and quality factors of the filter
 */
template <class T>
void NotchFilter<T>::calculate_A_and_Q(float center_freq_hz, float bandwidth_hz, float attenuation_dB, float& A, float& Q) {
    A = powf(10, -attenuation_dB / 40.0f);
    if (center_freq_hz > 0.5 * bandwidth_hz) {
        const float octaves = log2f(center_freq_hz / (center_freq_hz - bandwidth_hz / 2.0f)) * 2.0f;
        Q = sqrtf(powf(2, octaves)) / (powf(2, octaves) - 1.0f);
    } else {
        Q = 0.0;
    }
}

/*
  compute coefficients for a single biquad notch
 */
template <class T>
void NotchFilter<T>::compute_coefficients(float &b0, float &b1, float &b2, float &a1, float &a2,
                                          float sample_freq_hz, float center_freq_hz, float A, float Q)
{
    float omega = 2.0 * M_PI * center_freq_hz / sample_freq_hz;
    float alpha = sinf(omega) / (2 * Q);
    b0 =  1.0 + alpha*sq(A);
    b1 = -2.0 * cosf(omega);
    b2 =  1.0 - alpha*sq(A);
    a1 = b1;
    a2 =  1.0 - alpha;

    const float a0_inv =  1.0/(1.0 + alpha);

    // Pre-multiply to save runtime calc
    b0 *= a0_inv;
    b1 *= a0_inv;
    b2 *= a0_inv;
    a1 *= a0_inv;
    a2 *= a0_inv;
}

/*
  initialise filter, optionally with a bitmask of harmonics (default: fundamental only)
 */
template <class T>
void NotchFilter<T>::init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB, uint16_t harmonics)
{
    // free any existing multi-harmonic filters
    if (_harmonic_filters != nullptr) {
        delete[] _harmonic_filters;
        _harmonic_filters = nullptr;
    }
    _harmonics = harmonics;
    _num_harmonics = 0;
    initialised = false;

    if (harmonics == 0) {
        return;
    }

    if (harmonics == 1) {
        // single notch at fundamental - use slew-limited single-biquad path
        _num_harmonics = 1;
        float A, Q;
        calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB, A, Q);
        init_with_A_and_Q(sample_freq_hz, center_freq_hz, A, Q);
        return;
    }

    // multiple harmonics - allocate one NotchFilter per enabled harmonic
    float A, Q;
    calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB, A, Q);
    if (Q <= 0.0f) {
        _harmonics = 0;
        return;
    }

    const uint8_t alloc_count = __builtin_popcount(harmonics);
    _harmonic_filters = NEW_NOTHROW NotchFilter<T>[alloc_count];
    if (_harmonic_filters == nullptr) {
        _harmonics = 0;
        return;
    }

    uint8_t active_count = 0;
    for (uint8_t i = 0; i < max_harmonics; i++) {
        if ((harmonics & (1U << i)) == 0) {
            continue;
        }
        const float harmonic_center = center_freq_hz * (i + 1);
        if (is_positive(harmonic_center) && (harmonic_center < 0.5 * sample_freq_hz)) {
            _harmonic_filters[active_count].init_with_A_and_Q(sample_freq_hz, harmonic_center, A, Q);
            active_count++;
        }
    }

    _num_harmonics = active_count;
    if (_num_harmonics == 0) {
        delete[] _harmonic_filters;
        _harmonic_filters = nullptr;
        _harmonics = 0;
        return;
    }

    _center_freq_hz = center_freq_hz;
    _sample_freq_hz = sample_freq_hz;
    _A = A;
    initialised = true;
}

template <class T>
void NotchFilter<T>::init_with_A_and_Q(float sample_freq_hz, float center_freq_hz, float A, float Q)
{
    // free any existing multi-harmonic filters: this path manages a single biquad only.
    // init() is the right entry-point for harmonics > 1; calling this directly on a
    // multi-harmonic filter downgrades it to single-notch and frees the old array.
    if (_harmonic_filters != nullptr) {
        delete[] _harmonic_filters;
        _harmonic_filters = nullptr;
        _harmonics = 1;
        _num_harmonics = 1;
    }

    // don't update if no updates required
    if (initialised &&
        is_equal(center_freq_hz, _center_freq_hz) &&
        is_equal(sample_freq_hz, _sample_freq_hz) &&
        is_equal(A, _A)) {
        return;
    }

    float new_center_freq = center_freq_hz;

    // constrain the new center frequency by a percentage of the old frequency
    if (initialised && !need_reset && !is_zero(_center_freq_hz)) {
        new_center_freq = constrain_float(new_center_freq, _center_freq_hz * NOTCH_MAX_SLEW_LOWER,
                                          _center_freq_hz * NOTCH_MAX_SLEW_UPPER);
    }

    if (is_positive(new_center_freq) && (new_center_freq < 0.5 * sample_freq_hz) && (Q > 0.0)) {
        compute_coefficients(b0, b1, b2, a1, a2, sample_freq_hz, new_center_freq, A, Q);

        _center_freq_hz = new_center_freq;
        _sample_freq_hz = sample_freq_hz;
        _A = A;
        initialised = true;
    } else {
        // leave center_freq_hz at last value
        initialised = false;
    }
}

/*
  apply a new input sample, returning new output
 */
template <class T>
T NotchFilter<T>::apply(const T &sample)
{
    if (!initialised || need_reset) {
        // if we have not been initialised when return the input
        // sample as output and update delayed samples
        if (_harmonic_filters != nullptr) {
            for (uint8_t i = 0; i < _num_harmonics; i++) {
                _harmonic_filters[i].reset();
            }
        } else {
            signal1 = sample;
            signal2 = sample;
            ntchsig1 = sample;
            ntchsig2 = sample;
        }
        need_reset = false;
        return sample;
    }

    if (_harmonic_filters != nullptr) {
        T output = sample;
        for (uint8_t i = 0; i < _num_harmonics; i++) {
            output = _harmonic_filters[i].apply(output);
        }
        return output;
    }

    T output = sample*b0 + ntchsig1*b1 + ntchsig2*b2 - signal1*a1 - signal2*a2;

    ntchsig2 = ntchsig1;
    ntchsig1 = sample;

    signal2 = signal1;
    signal1 = output;
    return output;
}

template <class T>
void NotchFilter<T>::reset()
{
    need_reset = true;
}

#if HAL_LOGGING_ENABLED
// return the frequency to log for the notch
template <class T>
float NotchFilter<T>::logging_frequency() const
{
    return initialised ? _center_freq_hz : AP_Logger::quiet_nanf();
}
#endif

/*
   instantiate template classes
 */
template class NotchFilter<float>;
template class NotchFilter<Vector2f>;
template class NotchFilter<Vector3f>;
