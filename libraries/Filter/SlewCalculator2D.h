#pragma once

/* slew rate limiting filter. This is used to prevent oscillation of a
 * controller by modifying the controllers output based on a maximum
 * slew rate
*/

#include <AP_Math/AP_Math.h>
#include <stdint.h>
#include "SlewLimiter.h"

class SlewCalculator2D {
public:
    SlewCalculator2D();

    CLASS_NO_COPY(SlewCalculator2D);

    // apply filter to sample and calculate slewrate
    void update(const Vector2f& sample, float dt);

    // get last oscillation slew rate
    float get_slew_rate() const;

private:
    SlewLimiter xlimiter;   // X-axis 1D slew rate limiter
    SlewLimiter ylimiter;   // Y-axis 1D slew rate limiter

    float slew_rate_max = 0;    // slew rate max (always zero)
    float slew_rate_tau = 1.0;  // slew rate tau (always 1.0)
};
