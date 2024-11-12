// Example to test functionality of SlewLimiter

#include <AP_HAL/AP_HAL.h>
#include <Filter/SlewLimiter.h>
#include <AP_Math/chirp.h>

/* on Linux run with
    ./waf configure --board linux
    ./waf --targets examples/SlewLimiter
    ./build/linux/examples/SlewLimiter
*/

void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

const float slew_rate_max = 50.0;
const float slew_rate_tau = 1.0;

// 400 hz loop
const uint16_t dt_us = 2500;
const float dt_s = dt_us * 1.0e-6;

// Slew limiter object to be tested
SlewLimiter slew_limiter{slew_rate_max, slew_rate_tau};

// Chirp object to generate test signal
Chirp chirp;

// Chirp setup params
const float magnitude = 5.0;
const float duration = 60.0;
const float frequency_start = 5.0;
const float frequency_stop = 100;
const float time_fade_in = 15;
const float time_fade_out = 10;
const float time_const_freq = 2.0 / frequency_start;

uint64_t waveform_time_us = 0;

static void setup()
{
    hal.console->printf("SlewLimiter - rate max: %f, tau: %f, loop rate: %fHz\n", slew_rate_max, slew_rate_tau, 1 / dt_s);

    chirp.init(duration, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);
    hal.console->printf("Chirp - duration: %fs, magnitude: %f, start: %fHz, stop: %fHz, fade in: %fs, fade out:%fs, hold: %fs\n", duration, magnitude, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);

    hal.console->printf("Time (s), input, slew rate, mod\n");

    hal.scheduler->stop_clock(waveform_time_us);

}

void loop()
{
    const float waveform_time_s = waveform_time_us * 1.0e-6;

    const float input = chirp.update(waveform_time_s, magnitude);

    const float mod = slew_limiter.modifier(input, dt_s);

    const float slew_rate = slew_limiter.get_slew_rate();

    hal.console->printf("%f, %f, %f, %f\n", waveform_time_s, input, slew_rate, mod);

    if (chirp.completed()) {
        exit(1);
    }

    // Force clock times so we can run faster than real time
    waveform_time_us += dt_us;
    hal.scheduler->stop_clock(waveform_time_us);
}

AP_HAL_MAIN();
