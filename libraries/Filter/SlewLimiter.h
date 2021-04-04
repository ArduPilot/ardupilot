#pragma once

/* slew rate limiting filter. This is used to prevent oscillation of a
 * controller by modifying the controllers output based on a maximum
 * slew rate
*/

#include <stdint.h>
#include "LowPassFilter.h"

class SlewLimiter {
public:
    SlewLimiter(const float &slew_rate_max, const float &slew_rate_tau);

    /*
      apply filter to sample, returning multiplier between 0 and 1 to keep
      output within slew rate
    */
    float modifier(float sample, float dt);

    /*
      get last slew rate
     */
    float get_slew_rate(void) const {
        return slew_filter.get();
    }

private:
    const float &slew_rate_max;
    const float &slew_rate_tau;
    LowPassFilterFloat slew_filter;
    float slew_amplitude;
    float last_sample;
};
