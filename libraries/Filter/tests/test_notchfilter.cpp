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
    filter.init(1000, 60, 33, 41, 1);
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
    filter.init(rate_hz, test_freq, test_freq*0.5, attenuation_dB, 1);
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
  helper to measure output amplitude of a notch filter at a given frequency
 */
static float measure_attenuation(NotchFilter<float> &filter, float freq, float rate_hz, float test_amplitude=1.0)
{
    const double dt = 1.0 / rate_hz;
    const uint32_t period_samples = MAX(uint32_t(rate_hz / freq), 1U);
    const uint32_t periods = 200;
    // Use enough warmup samples for all cascaded-notch transients to decay.
    // A cascade whose lowest notch is at f_low with quality Q has poles at
    // |z| = sqrt(a2) close to 1; 200 samples covers τ ≈ 5×22 for the
    // common Q≈1.7 / 50 Hz case.
    const uint32_t warmup = MAX(2*period_samples, 200U);
    double v_max = 0;
    filter.reset();
    for (uint32_t i=0; i<periods * period_samples + warmup; i++) {
        const double t = i * dt;
        const double sample = sin(freq * t * 2 * M_PI) * test_amplitude;
        const float v = filter.apply(sample);
        if (i >= warmup) {
            v_max = MAX(v_max, fabsF(v));
        }
    }
    return ratio_to_dB(v_max / test_amplitude);
}

/*
  test multi-harmonic notch filter
 */
TEST(NotchFilterTest, MultiHarmonicNotchTest)
{
    const float base_freq = 50;
    const float bandwidth = base_freq / 2;
    const float attenuation_dB = 40;
    const float rate_hz = 2000;

    NotchFilter<float> filter;
    filter.init(rate_hz, base_freq, bandwidth, attenuation_dB, 0x03); // 1st and 2nd harmonic

    // check the filter reports the correct harmonics and fundamental frequency
    EXPECT_EQ(filter.harmonics(), 0x03U);
    EXPECT_FLOAT_EQ(filter.center_freq_hz(), base_freq);

    // we should see strong attenuation at the fundamental and 2nd harmonic
    EXPECT_LT(measure_attenuation(filter, base_freq, rate_hz), -attenuation_dB + 5);
    EXPECT_LT(measure_attenuation(filter, base_freq * 2, rate_hz), -attenuation_dB + 5);

    // a non-harmonic frequency should not be strongly attenuated.
    // With bandwidth = base_freq/2 the 100 Hz notch's -3 dB band spans ~71-129 Hz,
    // so 75 Hz (= 1.5×base) sits inside it; steady-state cascade gain is ≈ -3.8 dB
    // (in 10·ln units).  Use -6 as the threshold to allow for this overlap.
    EXPECT_GT(measure_attenuation(filter, base_freq * 1.5, rate_hz), -6);

    // a higher harmonic that is not enabled should not be strongly attenuated
    EXPECT_GT(measure_attenuation(filter, base_freq * 3, rate_hz), -3);
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
        double last_crossing;
        double total_lag_samples;
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
            // Always interpolating the value at 0.0
            // crossing happened some fraction before the current sample
            // result in the range -1.0 to 0.0
            // linear interpolation: ((0.0 - last_in) / (sample - last_in)) - 1.0 is the same as:
            // sample / (last_in - sample)
            integral.last_crossing = (double)s + (sample / (integral.last_in - sample));
        }
        if (v >= 0 && integral.last_out < 0 && integral.last_crossing > 0) {
            const double crossing = (double)s + (v / (integral.last_out - v));
            integral.total_lag_samples += crossing - integral.last_crossing;
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

/*
  show phase lag versus attenuation
 */
TEST(NotchFilterTest, HarmonicNotchTest5)
{
    const float min_attenuation = 0;
    const float max_attenuation = 50;
    const uint16_t steps = 200;

    const char *csv_file = "harmonicnotch_test5.csv";
    FILE *f = fopen(csv_file, "w");
    fprintf(f, "Attenuation(dB),Lag(10Hz),Lag(20Hz),Lag(30Hz),Lag(40Hz),Lag(50Hz),Lag(60Hz),Lag(70Hz)\n");

    for (uint16_t i=0; i<steps; i++) {
        const float test_freq[7] { 10, 20, 30, 40, 50, 60, 70 };
        float phase_lags[7];
        float attenuations[7];
        const float test_attenuation = min_attenuation + i*(max_attenuation-min_attenuation)/steps;
        for (uint8_t F = 0; F < ARRAY_SIZE(test_freq); F++) {
            test_one_filter(60, test_attenuation, 60, test_freq[F], 50, 1, 0,
                            phase_lags[F],
                            attenuations[F]);
        }
        fprintf(f, "%.3f,%f,%f,%f,%f,%f,%f,%f\n",
                test_attenuation,
                phase_lags[0], phase_lags[1], phase_lags[2],
                phase_lags[3], phase_lags[4], phase_lags[5],
                phase_lags[6]);
    }
    fclose(f);
}

// -----------------------------------------------------------------------
// NotchFilter multi-harmonic feature tests
// -----------------------------------------------------------------------

/*
  harmonics=0 must leave the filter uninitialised and pass every sample through unchanged.
 */
TEST(NotchFilterTest, MultiHarmonicZeroDisablesFilter)
{
    NotchFilter<float> filter;
    filter.init(1000, 50, 25, 40, 0);
    EXPECT_EQ(filter.harmonics(), 0U);
    // pass-through: apply() must return the input sample when not initialised
    for (uint32_t i = 0; i < 100; i++) {
        const float sample = sinf(50.0f * i * 2.0f * M_PI / 1000.0f);
        EXPECT_FLOAT_EQ(filter.apply(sample), sample);
    }
}

/*
  when the center frequency is below half the bandwidth, Q is zero and
  the multi-harmonic init must disable the filter.
 */
TEST(NotchFilterTest, MultiHarmonicBadQDisablesFilter)
{
    NotchFilter<float> filter;
    // center_freq_hz=10 < bandwidth_hz/2=15 → Q=0
    filter.init(1000, 10, 30, 40, 0x03);
    EXPECT_EQ(filter.harmonics(), 0U);
    for (uint32_t i = 0; i < 50; i++) {
        const float sample = sinf(10.0f * i * 2.0f * M_PI / 1000.0f);
        EXPECT_FLOAT_EQ(filter.apply(sample), sample);
    }
}

/*
  when every selected harmonic falls above the Nyquist frequency none
  of them can be initialised and the filter must be disabled.
 */
TEST(NotchFilterTest, MultiHarmonicAllAboveNyquistDisablesFilter)
{
    // sample_rate=2000 Hz → Nyquist=1000 Hz
    // base_freq=600 Hz, harmonics bits 1&2 → 1200 Hz and 1800 Hz, both > Nyquist
    NotchFilter<float> filter;
    filter.init(2000, 600, 200, 40, 0x06);
    EXPECT_EQ(filter.harmonics(), 0U);
    for (uint32_t i = 0; i < 50; i++) {
        const float sample = sinf(300.0f * i * 2.0f * M_PI / 2000.0f);
        EXPECT_FLOAT_EQ(filter.apply(sample), sample);
    }
}

/*
  reset() on a multi-harmonic filter must not produce a glitch: constant
  input after reset must yield constant output on every sample.
 */
TEST(NotchFilterTest, MultiHarmonicResetNoGlitch)
{
    NotchFilter<float> filter;
    filter.init(1000, 60, 30, 40, 0x03); // 1st and 2nd harmonic
    const float const_sample = -0.512f;
    filter.reset();
    for (uint32_t i = 0; i < 100; i++) {
        EXPECT_FLOAT_EQ(filter.apply(const_sample), const_sample);
    }
}

/*
  bit 1 selects the 2nd harmonic (2×base) only.  The filter must
  attenuate there but leave the fundamental unaffected.
 */
TEST(NotchFilterTest, MultiHarmonicSecondHarmonicOnly)
{
    const float rate_hz = 2000;
    const float base_freq = 50;
    const float attenuation_dB = 40;

    NotchFilter<float> filter;
    filter.init(rate_hz, base_freq, base_freq / 2, attenuation_dB, 0x02); // bit 1 = 2nd harmonic
    EXPECT_EQ(filter.harmonics(), 0x02U);

    // 2nd harmonic must be strongly attenuated
    EXPECT_LT(measure_attenuation(filter, base_freq * 2, rate_hz), -attenuation_dB + 5);
    // fundamental must NOT be strongly attenuated
    EXPECT_GT(measure_attenuation(filter, base_freq, rate_hz), -3);
}

/*
  non-consecutive harmonic selection: bits 0 and 2 → 1st and 3rd harmonics.
  Only those two frequencies must see significant attenuation.
 */
TEST(NotchFilterTest, MultiHarmonicNonConsecutive)
{
    const float rate_hz = 2000;
    const float base_freq = 50;
    const float attenuation_dB = 40;

    NotchFilter<float> filter;
    // bits 0,2 → 1×50=50 Hz, 3×50=150 Hz
    filter.init(rate_hz, base_freq, base_freq / 2, attenuation_dB, 0x05);
    EXPECT_EQ(filter.harmonics(), 0x05U);

    // 1st and 3rd harmonics must be strongly attenuated
    EXPECT_LT(measure_attenuation(filter, base_freq * 1, rate_hz), -attenuation_dB + 5);
    EXPECT_LT(measure_attenuation(filter, base_freq * 3, rate_hz), -attenuation_dB + 5);

    // 2nd harmonic (not selected) must NOT be strongly attenuated
    EXPECT_GT(measure_attenuation(filter, base_freq * 2, rate_hz), -3);
}

/*
  a second call to init() with a different harmonics bitmask must cleanly
  replace the previous filter bank without crash or leak.
 */
TEST(NotchFilterTest, MultiHarmonicReinit)
{
    const float rate_hz = 2000;
    const float base_freq = 50;
    const float bandwidth = base_freq / 2;
    const float attenuation_dB = 40;

    NotchFilter<float> filter;

    // first init: 1st and 2nd harmonics
    filter.init(rate_hz, base_freq, bandwidth, attenuation_dB, 0x03);
    EXPECT_EQ(filter.harmonics(), 0x03U);
    EXPECT_LT(measure_attenuation(filter, base_freq,     rate_hz), -attenuation_dB + 5);
    EXPECT_LT(measure_attenuation(filter, base_freq * 2, rate_hz), -attenuation_dB + 5);

    // second init: 2nd and 3rd harmonics only
    filter.init(rate_hz, base_freq, bandwidth, attenuation_dB, 0x06);
    EXPECT_EQ(filter.harmonics(), 0x06U);
    EXPECT_GT(measure_attenuation(filter, base_freq,     rate_hz), -3);      // 1st: no longer filtered
    EXPECT_LT(measure_attenuation(filter, base_freq * 2, rate_hz), -attenuation_dB + 5);
    EXPECT_LT(measure_attenuation(filter, base_freq * 3, rate_hz), -attenuation_dB + 5);
}

/*
  when some selected harmonics fall above Nyquist, only those below it
  are active; the filter is still considered initialised (harmonics()
  returns the originally requested bitmask).
 */
TEST(NotchFilterTest, MultiHarmonicPartialAboveNyquist)
{
    // sample_rate=2000 Hz → Nyquist=1000 Hz
    // base=400 Hz, harmonics=0x07 (bits 0,1,2) → 400, 800, 1200 Hz
    // Only 400 Hz and 800 Hz are below Nyquist
    const float rate_hz = 2000;
    const float base_freq = 400;
    const float attenuation_dB = 40;

    NotchFilter<float> filter;
    filter.init(rate_hz, base_freq, 150, attenuation_dB, 0x07);
    EXPECT_EQ(filter.harmonics(), 0x07U); // requested bitmask is preserved

    // the two active harmonics must be attenuated
    EXPECT_LT(measure_attenuation(filter, base_freq * 1, rate_hz), -attenuation_dB + 5);
    EXPECT_LT(measure_attenuation(filter, base_freq * 2, rate_hz), -attenuation_dB + 5);

    // a frequency between the two active harmonics must NOT be attenuated
    EXPECT_GT(measure_attenuation(filter, base_freq * 1.5f, rate_hz), -3);
}

AP_GTEST_MAIN()
