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

#include "ComplementaryFilter.h"

/*
  complementary filter. Used to combine a reading that is long term
  stable but has high freq noise with another reading that has less
  high frequency noise and poor long term stability.
 */

/*
  set crossover frequency between low frequency and high frequency
  components
 */
void ComplementaryFilter::set_cutoff_frequency(float freq_hz)
{
    cutoff_freq = freq_hz;
}

/*
  reset the filter to initial values
 */
void ComplementaryFilter::reset()
{
    lp.reset();
    hp.reset();
}

/*
  apply a low freqency and high freqency input to give a complementary
  filter result where signal below the cutoff comes from the low_freq
  input and above the cutoff from high freqency input. We take a
  timestamp on each input as the input data may vary somewhat in
  frequency
 */
float ComplementaryFilter::apply(float low_freq, float high_freq, uint32_t time_us)
{
    if (!is_positive(cutoff_freq)) {
        reset();
        return low_freq;
    }
    const uint32_t dt_us = time_us - last_sample_us;

    if (dt_us > 1e6) {
        // if we have not updated for 1s then assume we've had a
        // sensor outage and reset
        reset();
    }
    last_sample_us = time_us;

    const float dt = MIN(dt_us * 1.0e-6, 1);

    // keep a low pass filter of the sample rate. Rapidly changing
    // sample frequency in bi-quad filters leads to very nasty spikes
    if (!is_positive(sample_freq)) {
        sample_freq = 1.0/dt;
    } else {
        sample_freq = 0.99 * sample_freq + 0.01 * (1.0/dt);
    }

    lp.compute_params(sample_freq, cutoff_freq, params);

    float hp_out = hp.apply(high_freq, params);
    float lp_out = lp.apply(low_freq, params);

    out = (high_freq - hp_out) + lp_out;

    return out;
}

/*
  return the current value
 */
float ComplementaryFilter::get(void)
{
    return out;
}
