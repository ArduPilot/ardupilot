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
  initialise filter
 */
template <class T>
void NotchFilter<T>::init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB)
{
    // check center frequency is in the allowable range
    if ((center_freq_hz > 0.5 * bandwidth_hz) && (center_freq_hz < 0.5 * sample_freq_hz)) {
        float A, Q;
        calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB, A, Q);
        init_with_A_and_Q(sample_freq_hz, center_freq_hz, A, Q);
    } else {
        initialised = false;
    }
}

template <class T>
void NotchFilter<T>::init_with_A_and_Q(float sample_freq_hz, float center_freq_hz, float A, float Q)
{
    if ((center_freq_hz > 0.0) && (center_freq_hz < 0.5 * sample_freq_hz) && (Q > 0.0)) {
        float omega = 2.0 * M_PI * center_freq_hz / sample_freq_hz;
        float alpha = sinf(omega) / (2 * Q);
        b0 =  1.0 + alpha*sq(A);
        b1 = -2.0 * cosf(omega);
        b2 =  1.0 - alpha*sq(A);
        a0_inv =  1.0/(1.0 + alpha);
        a1 = b1;
        a2 =  1.0 - alpha;
        initialised = true;
    } else {
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
        ntchsig2 = ntchsig1;
        ntchsig1 = ntchsig;
        ntchsig = sample;
        signal2 = signal1;
        signal1 = sample;
        need_reset = false;
        return sample;
    }
    ntchsig2 = ntchsig1;
    ntchsig1 = ntchsig;
    ntchsig = sample;
    T output = (ntchsig*b0 + ntchsig1*b1 + ntchsig2*b2 - signal1*a1 - signal2*a2) * a0_inv;
    signal2 = signal1;
    signal1 = output;
    return output;
}

template <class T>
void NotchFilter<T>::reset()
{
    need_reset = true;
}

/*
   instantiate template classes
 */
template class NotchFilter<float>;
template class NotchFilter<Vector2f>;
template class NotchFilter<Vector3f>;
