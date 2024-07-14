#include "DataRateLimit.h"

#include <AP_HAL/AP_HAL.h>

// Return the max number of bytes that can be sent since the last call given byte/s rate limit
uint32_t DataRateLimit::max_bytes(const float bytes_per_sec)
{
    // Time since last call
    const uint32_t now_us = AP_HAL::micros();
    const float dt = (now_us - last_us) * 1.0e-6;
    last_us = now_us;

    // Maximum number of bytes that could be transferred in that time
    float max_bytes = bytes_per_sec * dt;

    // Add on the remainder from the last call, this prevents cumulative rounding errors
    max_bytes += remainder;

    // Get integer number of bytes and store the remainder
    float max_bytes_int;
    remainder = modf(max_bytes, &max_bytes_int);

    // Add 0.5 to make sure the float rounds to the correct int
    return uint32_t(max_bytes_int + 0.5);
}
