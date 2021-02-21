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

    // Calculate the slew rate amplitude produced by the unmodified sample
    // calculate a low pass filtered slew rate
    float Pterm_slew_rate = slew_filter.apply((fabsf(sample - last_sample)/ dt), dt);

    // rectify and apply a decaying envelope filter. The 10 in the
    // constrain limits the modifier to be between 0.1 and 1.0, so we
    // never drop PID gains below 10% of configured value
    float alpha = 1.0 - constrain_float(dt/slew_rate_tau, 0.0, 1.0);
    slew_amplitude = constrain_float(Pterm_slew_rate, alpha * slew_amplitude, 10 * slew_rate_max);

    // Calculate the gain adjustment
    float mod = slew_rate_max / fmaxf(slew_amplitude, slew_rate_max);
    last_sample = mod * sample;

    return mod;
}
