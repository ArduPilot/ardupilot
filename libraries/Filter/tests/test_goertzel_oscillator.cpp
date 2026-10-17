#include <AP_gtest.h>

#include "GoertzelOscillator.h"

static double max_abs(double a, double b)
{
    return a > b ? a : b;
}

/*
  compare the oscillator against real sin() calls over a realistic
  run length, at a few different frequencies including low and
  near-Nyquist edge cases
 */
TEST(GoertzelOscillatorTest, MatchesRealSin)
{
    const double rate_hz = 2000.0;
    const double test_freqs[] = { 1.0, 47.0, 250.0, 999.0 };

    for (double freq_hz : test_freqs) {
        Goertzel_Oscillator osc;
        osc.init(freq_hz, rate_hz);
        const double omega = 2.0 * M_PI * freq_hz / rate_hz;
        double max_err = 0;
        for (uint32_t n=0; n<50000; n++) {
            const double expected = sin(omega * n);
            const double got = osc.next();
            max_err = max_abs(max_err, fabs(got - expected));
        }
        EXPECT_LE(max_err, 1e-9) << "freq_hz=" << freq_hz;
    }
}

/*
  a non-zero starting phase should be reproduced exactly as well
 */
TEST(GoertzelOscillatorTest, NonZeroPhase)
{
    const double rate_hz = 2000.0;
    const double freq_hz = 47.0;
    const double phase_rad = 1.3;

    Goertzel_Oscillator osc;
    osc.init(freq_hz, rate_hz, phase_rad);
    const double omega = 2.0 * M_PI * freq_hz / rate_hz;
    double max_err = 0;
    for (uint32_t n=0; n<50000; n++) {
        const double expected = sin(omega * n + phase_rad);
        const double got = osc.next();
        max_err = max_abs(max_err, fabs(got - expected));
    }
    EXPECT_LE(max_err, 1e-9);
}

/*
  run for far longer than any single test_notchfilter sweep point
  (50000 samples) to confirm accumulated rounding error stays
  negligible over a long run
 */
TEST(GoertzelOscillatorTest, NoLongRunDrift)
{
    const double rate_hz = 2000.0;
    const double freq_hz = 47.0;

    Goertzel_Oscillator osc;
    osc.init(freq_hz, rate_hz);
    const double omega = 2.0 * M_PI * freq_hz / rate_hz;
    double max_err = 0;
    const uint32_t n_samples = 2000000;
    for (uint32_t n=0; n<n_samples; n++) {
        const double expected = sin(omega * n);
        const double got = osc.next();
        max_err = max_abs(max_err, fabs(got - expected));
    }
    EXPECT_LE(max_err, 1e-6);
}

/*
  reset() (via a second init()) should restart the sequence from
  n=0 rather than carrying over any state
 */
TEST(GoertzelOscillatorTest, ReinitRestartsSequence)
{
    const double rate_hz = 2000.0;
    const double freq_hz = 47.0;
    const double omega = 2.0 * M_PI * freq_hz / rate_hz;

    Goertzel_Oscillator osc;
    osc.init(freq_hz, rate_hz);
    for (uint32_t n=0; n<1234; n++) {
        osc.next();
    }
    osc.init(freq_hz, rate_hz);
    for (uint32_t n=0; n<100; n++) {
        EXPECT_NEAR(osc.next(), sin(omega * n), 1e-9);
    }
}

AP_GTEST_MAIN()
