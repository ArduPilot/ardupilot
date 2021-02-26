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

#pragma once

#include <AP_Math/AP_Math.h>
#include "LowPassFilter2p.h"

/*
  complementary filter. Used to combine a reading that is long term
  stable but has high freq noise with another reading that has less
  high frequency noise and poor long term stability.
 */

class ComplementaryFilter
{
public:
    void set_cutoff_frequency(float freq_hz);
    void reset();
    float apply(float low_freq, float high_freq, uint32_t time_us);
    float get(void);

private:
    uint32_t last_sample_us;
    float cutoff_freq;
    float sample_freq;
    // use 2-pole low pass filters to get a reasonably sharp cutoff
    DigitalBiquadFilter<float>::biquad_params params;
    DigitalBiquadFilter<float> lp, hp;
    float out;
};


