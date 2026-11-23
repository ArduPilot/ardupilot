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
   calculate the sine and cosine of an angle from 0 to pi radians, the range
   of a notch's centre frequency in radians per sample, without calling libm.

   A dynamic harmonic notch recalculates its coefficients for every notch at
   up to half the gyro rate, and needs both results each time. sinf() and
   cosf() each square the angle, and each reduce it when it is above pi/4.
   Here both results share that work.

   The method has three steps:

   1. Reduce the angle to x in [-pi/4, pi/4], where the polynomials in step 3
      are accurate, using the angle sum identities:

        angle           x             sin(angle)   cos(angle)
        0 to pi/4       angle         sin(x)       cos(x)
        pi/4 to 3pi/4   angle - pi/2  cos(x)       -sin(x)
        3pi/4 to pi     pi - angle    sin(x)       -cos(x)

   2. pi/2 and pi cannot be held exactly in a float, and subtracting a rounded
      value would lose most of the accuracy just where x is close to zero. So
      each is held as its nearest float (HIGH) and the small remainder (LOW)
      and subtracted in two steps, the method of Cody and Waite [1]. In each
      branch the angle is within a factor of two of HIGH, so the first
      subtraction is exact by Sterbenz's lemma [2]. This keeps 80-90% of
      notch coefficient sets bit-identical to those calculated with glibc,
      depending on how their centre frequencies are spread.

   3. Evaluate the single precision polynomials from the Cephes library
      [3][4], with their published coefficients:

        sin(x) = x + x^3 * (SIN_X3 + SIN_X5 * x^2 + SIN_X7 * x^4)
        cos(x) = 1 - x^2 / 2 + x^4 * (COS_X4 + COS_X6 * x^2 + COS_X8 * x^4)

   Inaccuracy, measured on every float angle from 1e-4 to pi against double
   precision, with and without fused multiply-add:

        sin: error up to 7.0e-8, and up to 1.4 units in the last place (ulp)
        cos: error up to 7.8e-8, and up to 1.5 ulp
        glibc's sinf() and cosf(), for comparison: 3.3e-8, and 0.6 ulp

   In a notch this is about the size of rounding its coefficients to float.
   Over 60 million random notches (sample rate 400 Hz to 8 kHz, centre from
   5 Hz to 0.49 of the rate, 15-60 dB, Q 1-100), compared with glibc:

   - None is unstable, and the centre is within about 0.02 Hz of the design
     with either calculation.
   - For typical notches (1-8 kHz, from 30 Hz, up to 40 dB, Q up to 11.7) the
     gain differs by less than 1 dB, and by under 0.02 dB in 99.9% of the
     notches whose coefficients differ at all.
   - Close to the centre of a very deep notch at a low frequency or high Q, a
     move that small can change the gain by tens of dB in either direction.
     With glibc these notches are just as far from the exact design.

   An angle outside 0 to pi is not reduced and can give a wrong result. The
   only caller checks that the centre frequency is above zero and below
   Nyquist.

   [1] W. J. Cody and W. Waite, Software Manual for the Elementary Functions,
       Prentice-Hall, 1980.
   [2] P. H. Sterbenz, Floating-Point Computation, Prentice-Hall, 1974,
       Theorem 4.3.1.
   [3] S. L. Moshier, Methods and Programs for Mathematical Functions,
       Prentice-Hall, 1989.
   [4] Cephes Mathematical Library, single precision sinf.c,
       https://netlib.org/cephes/single.tgz
 */
static void calculate_sinf_cosf(float angle_rad, float &sin_angle, float &cos_angle)
{
    static constexpr float QUARTER_PI_RAD = 0.785398163f;
    static constexpr float THREE_QUARTER_PI_RAD = 2.35619449f;
    // pi/2 and pi as their nearest float and the remainder to the exact value
    static constexpr float HALF_PI_HIGH_RAD = 1.57079637f;
    static constexpr float HALF_PI_LOW_RAD = -4.37113900e-08f;
    static constexpr float PI_HIGH_RAD = 3.14159274f;
    static constexpr float PI_LOW_RAD = -8.74227801e-08f;
    // Cephes sinf.c, sincof[] and coscof[]
    static constexpr float SIN_X3 = -1.6666654611e-1f;
    static constexpr float SIN_X5 = 8.3321608736e-3f;
    static constexpr float SIN_X7 = -1.9515295891e-4f;
    static constexpr float COS_X4 = 4.166664568298827e-2f;
    static constexpr float COS_X6 = -1.388731625493765e-3f;
    static constexpr float COS_X8 = 2.443315711809948e-5f;

    const bool upper_quarter = angle_rad >= THREE_QUARTER_PI_RAD;
    const bool middle_half = !upper_quarter && angle_rad > QUARTER_PI_RAD;

    float x_rad = angle_rad;
    if (middle_half) {
        x_rad = (angle_rad - HALF_PI_HIGH_RAD) - HALF_PI_LOW_RAD;
    } else if (upper_quarter) {
        x_rad = (PI_HIGH_RAD - angle_rad) + PI_LOW_RAD;
    }

    const float x_sq = x_rad * x_rad;
    const float sin_x = ((SIN_X7 * x_sq + SIN_X5) * x_sq + SIN_X3) * x_sq * x_rad + x_rad;
    const float cos_x = ((COS_X8 * x_sq + COS_X6) * x_sq + COS_X4) * x_sq * x_sq - 0.5f * x_sq + 1.0f;

    if (middle_half) {
        sin_angle = cos_x;
        cos_angle = -sin_x;
    } else if (upper_quarter) {
        sin_angle = sin_x;
        cos_angle = -cos_x;
    } else {
        sin_angle = sin_x;
        cos_angle = cos_x;
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
  initialise filter
 */
template <class T>
void NotchFilter<T>::init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB)
{
    // check center frequency is in the allowable range
    initialised = false;
    if ((center_freq_hz > 0.5 * bandwidth_hz) && (center_freq_hz < 0.5 * sample_freq_hz)) {
        float A, Q;
        calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB, A, Q);
        init_with_A_and_Q(sample_freq_hz, center_freq_hz, A, Q);
    }
}

template <class T>
void NotchFilter<T>::init_with_A_and_Q(float sample_freq_hz, float center_freq_hz, float A, float Q)
{
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
        float omega = 2.0 * M_PI * new_center_freq / sample_freq_hz;
        float sin_omega, cos_omega;
        calculate_sinf_cosf(omega, sin_omega, cos_omega);
        float alpha = sin_omega / (2 * Q);
        b0 =  1.0 + alpha*sq(A);
        b1 = -2.0 * cos_omega;
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
        signal1 = sample;
        signal2 = sample;
        ntchsig1 = sample;
        ntchsig2 = sample;
        need_reset = false;
        return sample;
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
