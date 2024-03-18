#include <AP_gtest.h>

#include <Filter/Filter.h>
#include <Filter/NotchFilter.h>
#include <Filter/HarmonicNotchFilter.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static double ratio_to_dB(double ratio)
{
    return 10*log(ratio);
}

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
    const double dt = 1.0 / rate_hz;
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
            integral2_in += fabsF(sample) * dt;
            integral1_out += v * dt;
            integral2_out += fabsF(v) * dt;
        }
    }

    // we expect both absolute integrals to be zero
    EXPECT_LE(fabsF(integral1_in), 0.01);
    EXPECT_LE(fabsF(integral1_out), 0.01);

    // we expect the output abs integral to be smaller than input
    // integral by the attenuation
    const float ratio1 = integral2_in / integral2_out;
    ::printf("ratio1=%f expected_ratio=%f\n", ratio1, expected_ratio);
    const float err_pct = 100 * fabsF(ratio1 - expected_ratio) / ratio1;
    EXPECT_LE(err_pct, 1);
}

/*
  test attentuation versus frequency
  This is a way to get a graph of the attenuation and phase lag for a complex filter setup
 */
TEST(NotchFilterTest, HarmonicNotchTest)
{
    const uint8_t num_test_freq = 150;
    const uint8_t harmonics = 15;
    const float base_freq = 46;
    const float bandwidth = base_freq/2;
    const float attenuation_dB = 60;
    // number of identical filters chained together, simulating
    // usage of per-motor notch filtering
    const uint8_t chained_filters = 8;
    const uint16_t rate_hz = 2000;
    const uint32_t samples = 50000;
    const float test_amplitude = 0.7;
    const double dt = 1.0 / rate_hz;

    HarmonicNotchFilter<float> filters[num_test_freq][chained_filters] {};
    struct {
        double in;
        double out;
        double last_in;
        double last_out;
        uint32_t last_crossing;
        uint32_t total_lag_samples;
        uint32_t lag_count;
        float get_lag_degrees(const float freq) const {
            const float lag_avg = total_lag_samples/float(lag_count);
            return (360.0 * lag_avg * freq) / rate_hz;
        }
    } integrals[num_test_freq] {};

    for (uint8_t i=0; i<num_test_freq; i++) {
        for (uint8_t c=0; c<chained_filters; c++) {
            auto &f = filters[i][c];
            HarmonicNotchFilterParams notch_params {};
            notch_params.set_options(uint16_t(HarmonicNotchFilterParams::Options::TreatLowAsMin) |
                                     uint16_t(HarmonicNotchFilterParams::Options::DoubleNotch));
            notch_params.set_attenuation(attenuation_dB);
            notch_params.set_bandwidth_hz(bandwidth);
            notch_params.set_center_freq_hz(base_freq);
            notch_params.set_freq_min_ratio(1.0);
            f.allocate_filters(1, harmonics, notch_params.num_composite_notches());
            f.init(rate_hz, notch_params);
            f.update(base_freq);
        }
    }

    for (uint32_t s=0; s<samples; s++) {
        const double t = s * dt;

        for (uint8_t i=0; i<num_test_freq; i++) {
            const float freq = i+1;
            const double sample = sin(freq * t * 2 * M_PI) * test_amplitude;
            float v = sample;
            for (uint8_t c=0; c<chained_filters; c++) {
                auto &f = filters[i][c];
                v = f.apply(v);
            }
            if (s >= s/10) {
                integrals[i].in += fabsF(sample) * dt;
                integrals[i].out += fabsF(v) * dt;
            }
            if (sample >= 0 && integrals[i].last_in < 0) {
                integrals[i].last_crossing = s;
            }
            if (v >= 0 && integrals[i].last_out < 0 && integrals[i].last_crossing != 0) {
                integrals[i].total_lag_samples += s - integrals[i].last_crossing;
                integrals[i].lag_count++;
            }
            integrals[i].last_in = sample;
            integrals[i].last_out = v;
        }
    }
    const char *csv_file = "harmonicnotch_test.csv";
    FILE *f = fopen(csv_file, "w");
    fprintf(f, "Freq(Hz),Ratio,Lag(deg)\n");
    for (uint8_t i=0; i<num_test_freq; i++) {
        const float freq = i+1;
        const float lag_degrees = integrals[i].get_lag_degrees(freq);
        fprintf(f, "%.3f,%f,%f\n", freq, integrals[i].out/integrals[i].in, lag_degrees);
    }
    fclose(f);
    printf("Wrote %s\n", csv_file);
    ::printf("Lag at 1Hz %.2f degrees\n", integrals[0].get_lag_degrees(1));
    ::printf("Lag at 2Hz %.2f degrees\n", integrals[1].get_lag_degrees(2));
    ::printf("Lag at 5Hz %.2f degrees\n", integrals[4].get_lag_degrees(5));
    ::printf("Lag at 10Hz %.2f degrees\n", integrals[9].get_lag_degrees(10));
    EXPECT_NEAR(integrals[0].get_lag_degrees(1), 11.0, 0.5);
    EXPECT_NEAR(integrals[1].get_lag_degrees(2), 22.03, 0.5);
    EXPECT_NEAR(integrals[4].get_lag_degrees(5), 55.23, 0.5);
    EXPECT_NEAR(integrals[9].get_lag_degrees(10), 112.23, 0.5);
}


/*
  calculate attenuation and phase lag for a single harmonic notch filter
 */
static void test_one_filter(float base_freq, float attenuation_dB,
                            float bandwidth, float test_freq, float source_freq,
                            uint16_t harmonics, uint16_t options,
                            float &phase_lag, float &out_attenuation_dB)
{
    const uint16_t rate_hz = 2000;
    const uint32_t samples = 50000;
    const float test_amplitude = 1.0;
    const double dt = 1.0 / rate_hz;

    HarmonicNotchFilter<float> filter {};
    struct {
        double last_in;
        double last_out;
        double v_max;
        uint32_t last_crossing;
        uint32_t total_lag_samples;
        uint32_t lag_count;
        float get_lag_degrees(const float freq) const {
            const float lag_avg = total_lag_samples/float(lag_count);
            return (360.0 * lag_avg * freq) / rate_hz;
        }
    } integral {};

    auto &f = filter;


    HarmonicNotchFilterParams notch_params {};
    notch_params.set_options(options);
    notch_params.set_attenuation(attenuation_dB);
    notch_params.set_bandwidth_hz(bandwidth);
    notch_params.set_center_freq_hz(base_freq);
    notch_params.set_freq_min_ratio(1.0);
    f.allocate_filters(1, harmonics, notch_params.num_composite_notches());
    f.init(rate_hz, notch_params);
    f.update(source_freq);

    for (uint32_t s=0; s<samples; s++) {
        const double t = s * dt;

        const double sample = sin(test_freq * t * 2 * M_PI) * test_amplitude;
        float v = sample;
        v = f.apply(v);
        if (s >= samples/10) {
            integral.v_max = MAX(integral.v_max, v);
        }
        if (sample >= 0 && integral.last_in < 0) {
            integral.last_crossing = s;
        }
        if (v >= 0 && integral.last_out < 0 && integral.last_crossing != 0) {
            integral.total_lag_samples += s - integral.last_crossing;
            integral.lag_count++;
        }
        integral.last_in = sample;
        integral.last_out = v;
        f.update(source_freq);
    }
    phase_lag = integral.get_lag_degrees(test_freq);
    out_attenuation_dB = ratio_to_dB(integral.v_max/test_amplitude);
}

/*
  test the test_one_filter function
 */
TEST(NotchFilterTest, HarmonicNotchTest2)
{
    const float min_freq = 1.0;
    const float max_freq = 200;
    const uint16_t steps = 2000;

    const char *csv_file = "harmonicnotch_test2.csv";
    FILE *f = fopen(csv_file, "w");
    fprintf(f, "Freq(Hz),Attenuation(dB),Lag(deg)\n");

    for (uint16_t i=0; i<steps; i++) {
        float attenuation_dB, phase_lag;
        const float test_freq = min_freq + i*(max_freq-min_freq)/steps;
        test_one_filter(50, 30, 25, test_freq, 50, 3, uint16_t(HarmonicNotchFilterParams::Options::TripleNotch), phase_lag, attenuation_dB);
        fprintf(f, "%.3f,%f,%f\n", test_freq, attenuation_dB, phase_lag);
    }
    fclose(f);
}

/*
  test behaviour with TreatLowAsMin, we expect attenuation to decrease
  as we get close to zero source frequency and phase lag to approach
  zero
 */
TEST(NotchFilterTest, HarmonicNotchTest3)
{
    const float min_freq = 1.0;
    const float max_freq = 200;
    const uint16_t steps = 1000;

    const char *csv_file = "harmonicnotch_test3.csv";
    FILE *f = fopen(csv_file, "w");
    fprintf(f, "Freq(Hz),Attenuation1(dB),Lag1(deg),Attenuation2(dB),Lag2(deg)\n");

    const float source_freq = 35;
    for (uint16_t i=0; i<steps; i++) {
        float attenuation_dB1, phase_lag1;
        float attenuation_dB2, phase_lag2;
        const float test_freq = min_freq + i*(max_freq-min_freq)/steps;
        // first with TreatLowAsMin
        test_one_filter(50, 30, 25, test_freq, source_freq, 1, uint16_t(HarmonicNotchFilterParams::Options::TreatLowAsMin),
                        phase_lag1,
                        attenuation_dB1);
        // and without
        test_one_filter(50, 30, 25, test_freq, source_freq, 1, 0,
                        phase_lag2,
                        attenuation_dB2);
        fprintf(f, "%.3f,%f,%f,%f,%f\n", test_freq, attenuation_dB1, phase_lag1, attenuation_dB2, phase_lag2);

        // the phase lag with the attenuation adjustment should not be
        // more than without for frequencies below min freq
        if (test_freq < 50) {
            EXPECT_LE(phase_lag2, phase_lag1);
        }
    }
    fclose(f);
}

/*
  show the progress of a multi-harmonic notch as the source frequency drops below the min frequency
 */
TEST(NotchFilterTest, HarmonicNotchTest4)
{
    const float min_freq = 1.0;
    const float max_freq = 250;
    const uint16_t steps = 1000;

    const char *csv_file = "harmonicnotch_test4.csv";
    FILE *f = fopen(csv_file, "w");
    fprintf(f, "Freq(Hz),60Hz(dB),50Hz(dB),40Hz(dB),30Hz(dB),20Hz(dB),10Hz(dB),0Hz(dB)\n");

    for (uint16_t i=0; i<steps; i++) {
        float phase_lag;
        const float source_freq[7] { 60, 50, 40, 30, 20, 10, 0 };
        float attenuations[7];
        const float test_freq = min_freq + i*(max_freq-min_freq)/steps;
        for (uint8_t F = 0; F < ARRAY_SIZE(source_freq); F++) {
            test_one_filter(50, 30, 25, test_freq, source_freq[F], 15, 0,
                            phase_lag,
                            attenuations[F]);
        }
        fprintf(f, "%.3f,%f,%f,%f,%f,%f,%f,%f\n",
                test_freq,
                attenuations[0], attenuations[1], attenuations[2],
                attenuations[3], attenuations[4], attenuations[5],
                attenuations[6]);
    }
    fclose(f);
}

AP_GTEST_MAIN()
