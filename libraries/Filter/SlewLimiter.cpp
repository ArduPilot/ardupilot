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
/*
  The SlewLimiter filter provides an actuator slew rate limiter for
  PID controllers. It is used to reduce the P and D gains when the
  filter detects that the P+D components are pushing the actuator
  beyond the configured actuator rate limit. This can prevent
  oscillations that are caused by the output actuation rate going
  beyond the actuator maximum physical rate, which causes the
  actuation demand and achieved rate to get out of phase.

  this filter was originally written by Paul Riseborough for fixed
  wing use. It was adapted for wider use in AC_PID by Andrew Tridgell
 */
#include "SlewLimiter.h"

SlewLimiter::SlewLimiter(const float &_slew_rate_max, const float &_slew_rate_tau) :
    slew_rate_max(_slew_rate_max),
    slew_rate_tau(_slew_rate_tau)
{
    slew_filter.set_cutoff_frequency(10.0);
    slew_filter.reset(0.0);
}

/*
  apply filter to sample, returning multiplier between 0 and 1 to keep
  output within slew rate
 */
float SlewLimiter::modifier(float sample, float dt)
{
    if (slew_rate_max <= 0) {
        return 1.0;
    }

    // calculate a low pass filtered slew rate
    const float slew_rate = slew_filter.apply((sample - last_sample) / dt, dt);
    last_sample = sample;

    // Rectify and apply a decaying envelope filter. The 10 in the
    // constrain limits the modifier to be between 0.1 and 1.0, so we
    // never drop PID gains below 10% of configured value
    // Do this separately for the positive and negative direction and use the smaller of
    // these to avoid single direction transients causing unwanted gain compression
    if (slew_rate > _max_pos_slew_rate) {
        _max_pos_slew_rate = fminf(slew_rate, 10.0f * slew_rate_max);
    } else {
        _max_pos_slew_rate *= (1.0f - fminf(dt, slew_rate_tau) / slew_rate_tau);
    }
    if (slew_rate < -_max_neg_slew_rate) {
        _max_neg_slew_rate = fminf(-slew_rate, 10.0f * slew_rate_max);
    } else {
        _max_neg_slew_rate *= (1.0f - fminf(dt, slew_rate_tau) / slew_rate_tau);
    }
    const float slew_rate_amplitude = fminf(_max_pos_slew_rate,_max_neg_slew_rate);

    // Calculate the gain adjustment
    float mod;
    if (slew_rate_amplitude > slew_rate_max) {
        mod = slew_rate_max / fmaxf(slew_rate_amplitude, slew_rate_max);
    } else {
        mod = 1.0f;
    }

    return mod;
}
