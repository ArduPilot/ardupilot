#include <AP_gtest.h>

#include <Filter/Filter.h>
#include <Filter/NotchFilter.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  test if a reset of a notch filter results in no glitch in the following samples
  with constant input
 */
TEST(NotchFilterTest, ResetTest)
{
    NotchFilter<float> filter;
    filter.init(1000, 60, 33, 41);
    const float const_sample = -0.512;
    filter.reset();
    for (uint32_t i=0; i<100; i++) {
        float v = filter.apply(const_sample);
        EXPECT_FLOAT_EQ(v, const_sample);
    }
}

/*
  test with a sine input
 */
TEST(NotchFilterTest, SineTest)
{
    NotchFilter<float> filter;
    const float test_freq = 47;
    const float attenuation_dB = 43;
    const float rate_hz = 2000;
    double dt = 1.0 / rate_hz;
    const uint32_t period_samples = rate_hz / test_freq;
    const uint32_t periods = 1000;
    const float test_amplitude = 0.7;
    const float expected_ratio = powf(10, (attenuation_dB/2)/10.0);
    double integral1_in = 0;
    double integral1_out = 0;
    double integral2_in = 0;
    double integral2_out = 0;
    filter.init(rate_hz, test_freq, test_freq*0.5, attenuation_dB);
    filter.reset();
    for (uint32_t i=0; i<periods * period_samples; i++) {
        const double t = i * dt;
        const double sample = sin(test_freq * t * 2 * M_PI) * test_amplitude;
        const float v = filter.apply(sample);
        if (i >= 2*period_samples) {
            integral1_in += sample * dt;
            integral2_in += fabsf(sample) * dt;
            integral1_out += v * dt;
            integral2_out += fabsf(v) * dt;
        }
    }

    // we expect both absolute integrals to be zero
    EXPECT_LE(fabsf(integral1_in), 0.01);
    EXPECT_LE(fabsf(integral1_out), 0.01);

    // we expect the output abs integral to be smaller than input
    // integral by the attenuation
    const float ratio1 = integral2_in / integral2_out;
    ::printf("ratio1=%f expected_ratio=%f\n", ratio1, expected_ratio);
    const float err_pct = 100 * fabsf(ratio1 - expected_ratio) / ratio1;
    EXPECT_LE(err_pct, 1);
}

AP_GTEST_MAIN()
