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
    slew_filter.set_cutoff_frequency(DERIVATIVE_CUTOFF_FREQ);
    slew_filter.reset(0.0);
}

/*
  apply filter to sample, returning multiplier between 0 and 1 to keep
  output within slew rate
 */
float SlewLimiter::modifier(float sample, float dt)
{
    if (slew_rate_max <= 0) {
        _output_slew_rate = 0.0;
        return 1.0;
    }

    // Calculate a low pass filtered slew rate
    const float slew_rate = slew_filter.apply((sample - last_sample) / dt, dt);
    last_sample = sample;

    uint32_t now_ms = AP_HAL::millis();
    const float decay_alpha = fminf(dt, slew_rate_tau) / slew_rate_tau;

    // Store a series of positive slew rate exceedance events
    if (!_pos_event_stored && slew_rate > slew_rate_max) {
        if (_pos_event_index >= N_EVENTS) {
            _pos_event_index = 0;
        }
        _pos_event_ms[_pos_event_index] = now_ms;
        _pos_event_index++;
        _pos_event_stored = true;
        _neg_event_stored = false;
    }

    // Store a series of negative slew rate exceedance events
    if (!_neg_event_stored && slew_rate < - slew_rate_max) {
        if (_neg_event_index >= N_EVENTS) {
            _neg_event_index = 0;
        }
        _neg_event_ms[_neg_event_index] = now_ms;
        _neg_event_index++;
        _neg_event_stored = true;
        _pos_event_stored = false;
    }

    // Find the oldest event time
    uint32_t oldest_ms = now_ms;
    for (uint8_t index = 0; index < N_EVENTS; index++) {
        if (_pos_event_ms[index] < oldest_ms) {
            oldest_ms = _pos_event_ms[index];
        }
        if (_neg_event_ms[index] < oldest_ms) {
            oldest_ms = _neg_event_ms[index];
        }
    }

    // Decay the peak positive and negative slew rate if they are outside the window
    // Never drop PID gains below 10% of configured value
    if (slew_rate > _max_pos_slew_rate) {
        _max_pos_slew_rate = fminf(slew_rate, 10.0f * slew_rate_max);
        _max_pos_slew_event_ms = now_ms;
    } else if (now_ms - _max_pos_slew_event_ms > WINDOW_MS) {
        _max_pos_slew_rate *= (1.0f - decay_alpha);
    }

    if (slew_rate < -_max_neg_slew_rate) {
        _max_neg_slew_rate = fminf(-slew_rate, 10.0f * slew_rate_max);
        _max_neg_slew_event_ms = now_ms;
    } else if (now_ms - _max_neg_slew_event_ms > WINDOW_MS) {
        _max_neg_slew_rate *= (1.0f - decay_alpha);
    }

    const float raw_slew_rate = 0.5f*(_max_pos_slew_rate + _max_neg_slew_rate);

    // Apply a further reduction when the oldest exceedance event falls outside the window required for the
    // specified number of exceedance events. This prevents spikes due to control mode changed, etc causing
    // unwanted gain reduction and is only applied to the slew rate used for gain reduction
    float modifier_input = raw_slew_rate;
    if (now_ms - oldest_ms > (N_EVENTS + 1) * WINDOW_MS) {
        const float oldest_time_from_window = 0.001f*(float)(now_ms - oldest_ms - (N_EVENTS + 1) * WINDOW_MS);
        modifier_input *= expf(-oldest_time_from_window / slew_rate_tau);
    }

    // Apply a filter to increases in slew rate only to reduce the effect of gusts and large controller
    // setpoint changes
    const float attack_alpha = fminf(2.0f * decay_alpha, 1.0f);

    _modifier_slew_rate = (1.0f - attack_alpha) * _modifier_slew_rate + attack_alpha * modifier_input;
    _modifier_slew_rate = fminf(_modifier_slew_rate, modifier_input);

    _output_slew_rate = (1.0f - attack_alpha) * _output_slew_rate + attack_alpha * raw_slew_rate;
    _output_slew_rate = fminf(_output_slew_rate, raw_slew_rate);

    // Calculate the gain adjustment
    float mod;
    if (_modifier_slew_rate > slew_rate_max) {
        mod = slew_rate_max / (slew_rate_max + MODIFIER_GAIN * (_modifier_slew_rate - slew_rate_max));
    } else {
        mod = 1.0f;
    }

    return mod;
}
