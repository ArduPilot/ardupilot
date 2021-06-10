#pragma once

/* slew rate limiting filter. This is used to prevent oscillation of a
 * controller by modifying the controllers output based on a maximum
 * slew rate
*/

#include <stdint.h>
#include "LowPassFilter.h"

#define N_EVENTS 2 // number of positive and negative consecutive slew rate exceedance events recorded where a value of 2 corresponds to a complete cycle 
#define WINDOW_MS 300 // time in msec required for a half cycle of the slowest oscillation frequency expected
#define MODIFIER_GAIN 1.5f // ratio of modifier reduction to slew rate exceedance ratio
#define DERIVATIVE_CUTOFF_FREQ 25.0f

class SlewLimiter {
public:
    SlewLimiter(const float &slew_rate_max, const float &slew_rate_tau);

    CLASS_NO_COPY(SlewLimiter);

    /*
      apply filter to sample, returning multiplier between 0 and 1 to keep
      output within slew rate
    */
    float modifier(float sample, float dt);

    /*
      get last oscillation slew rate
     */
    float get_slew_rate(void) const {
        return _output_slew_rate;
    }

private:
    const float &slew_rate_max;
    const float &slew_rate_tau;
    LowPassFilterFloat slew_filter;
    float _output_slew_rate;
    float _modifier_slew_rate;
    float last_sample;
    float _max_pos_slew_rate;
    float _max_neg_slew_rate;
    uint32_t _max_pos_slew_event_ms;
    uint32_t _max_neg_slew_event_ms;
    uint8_t _pos_event_index;
    uint8_t _neg_event_index;
    uint32_t _pos_event_ms[N_EVENTS];
    uint32_t _neg_event_ms[N_EVENTS];
    bool _pos_event_stored;
    bool _neg_event_stored;
};
