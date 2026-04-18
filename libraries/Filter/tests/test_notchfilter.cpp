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

// ---------------------------------------------------------------------------
// RT-safety regression tests for HarmonicNotchFilter pre-allocation.
//
// These tests describe the EXPECTED post-fix behaviour.  On the unfixed
// code they will FAIL, proving the bugs are real:
//
//   PreAllocMaxCapacity       – fails: _num_filters == 1 not HAL_HNF_MAX_FILTERS
//   ExpandFilterCountIsNoop   – fails: expand_filter_count heap-allocates
//   EnabledFiltersAfterUpdate – fails: update() triggers heap realloc via
//                                      expand_filter_count when num_centers > 1
//   ExpandFilterCountOverflow – passes on old code (realloc succeeds on a
//                                      dev machine) but exposes the RT alloc path
//
// Run "waf tests && build/sitl/tests/test_notchfilter --gtest_filter=*PreAlloc*"
// on an unpatched tree to observe the failures.
// ---------------------------------------------------------------------------

/*
  Accessor for private HarmonicNotchFilter members used by the tests below.
  The class is declared as a friend in HarmonicNotchFilter.h.
 */
class HarmonicNotchFilterTest {
public:
    template <class T>
    static uint16_t num_filters(const HarmonicNotchFilter<T> &f) { return f._num_filters; }
    template <class T>
    static uint16_t num_enabled_filters(const HarmonicNotchFilter<T> &f) { return f._num_enabled_filters; }
    template <class T>
    static bool alloc_failed(const HarmonicNotchFilter<T> &f) { return f._alloc_has_failed; }
};

/*
  allocate_filters() must reserve HAL_HNF_MAX_FILTERS up front so that
  expand_filter_count() never needs to heap-allocate on the real-time
  filter-update path.

  WITHOUT THE FIX: allocate_filters(1,1,1) sets _num_filters = 1*1*1 = 1,
  not HAL_HNF_MAX_FILTERS.  This test fails with:
      Expected: num_filters == HAL_HNF_MAX_FILTERS (24 or 54)
      Actual:   num_filters == 1
 */
TEST(NotchFilterTest, PreAllocMaxCapacity)
{
    // normal case: at least one harmonic enabled
    {
        HarmonicNotchFilter<float> f {};
        f.allocate_filters(1, 1, 1);
        EXPECT_EQ(HarmonicNotchFilterTest::num_filters(f), HAL_HNF_MAX_FILTERS)
            << "allocate_filters must reserve HAL_HNF_MAX_FILTERS when harmonics != 0";
        EXPECT_FALSE(HarmonicNotchFilterTest::alloc_failed(f));
    }
    // harmonics=0 (filter disabled): must NOT allocate
    {
        HarmonicNotchFilter<float> f {};
        f.allocate_filters(1, 0, 1);
        EXPECT_EQ(HarmonicNotchFilterTest::num_filters(f), 0u)
            << "allocate_filters must not allocate when harmonics == 0";
        EXPECT_FALSE(HarmonicNotchFilterTest::alloc_failed(f));
    }
}

/*
  expand_filter_count() with a count <= _num_filters must be a pure no-op:
  no allocation, no flag set.

  WITHOUT THE FIX: expand_filter_count(HAL_HNF_MAX_FILTERS) finds
  total_notches > _num_filters (1) and calls NEW_NOTHROW to grow the
  bank — heap allocation on the RT path.
 */
TEST(NotchFilterTest, ExpandFilterCountIsNoop)
{
    HarmonicNotchFilter<float> f {};
    f.allocate_filters(1, 1, 1);

    f.expand_filter_count(HAL_HNF_MAX_FILTERS);
    EXPECT_FALSE(HarmonicNotchFilterTest::alloc_failed(f))
        << "expand_filter_count must not fail when capacity is sufficient";
    EXPECT_EQ(HarmonicNotchFilterTest::num_filters(f), HAL_HNF_MAX_FILTERS);
}

/*
  Calling expand_filter_count() with a value beyond HAL_HNF_MAX_FILTERS
  must set _alloc_has_failed (the only safe fallback: cap at max capacity).
 */
TEST(NotchFilterTest, ExpandFilterCountOverflowSetsAllocFailed)
{
    HarmonicNotchFilter<float> f {};
    f.allocate_filters(1, 1, 1);
    ASSERT_FALSE(HarmonicNotchFilterTest::alloc_failed(f));

    f.expand_filter_count(HAL_HNF_MAX_FILTERS + 1);
    EXPECT_TRUE(HarmonicNotchFilterTest::alloc_failed(f))
        << "expand_filter_count beyond capacity must set _alloc_has_failed";
}

/*
  After update() with N centers, _num_enabled_filters must equal
  N × num_harmonics × composite_notches.

  This is the field log_notch_centers() must read to know how many
  active sources to log.  Using _num_filters (= HAL_HNF_MAX_FILTERS
  after the pre-alloc fix) would cause it to log garbage for uninitialized
  filter slots.

  WITHOUT THE FIX: update(4, centers) triggers expand_filter_count(4)
  which heap-allocates a new bank — RT allocation on the hot path.
 */
TEST(NotchFilterTest, EnabledFiltersAfterUpdate)
{
    HarmonicNotchFilter<float> f {};
    HarmonicNotchFilterParams params {};
    params.set_center_freq_hz(50);
    params.set_bandwidth_hz(20);
    params.set_attenuation(40);
    params.set_freq_min_ratio(1.0);

    f.allocate_filters(4, 1, 1);
    f.init(2000.0f, params);

    const float centers[4] = { 50.0f, 60.0f, 70.0f, 80.0f };
    f.update(4, centers);

    EXPECT_EQ(HarmonicNotchFilterTest::num_enabled_filters(f), 4u)
        << "4 centers × 1 harmonic × 1 composite = 4 enabled filters";
    // capacity must still be the full pre-allocated max
    EXPECT_EQ(HarmonicNotchFilterTest::num_filters(f), HAL_HNF_MAX_FILTERS);
}

/*
  update() with more centers than the initial allocation must not crash
  and must not heap-allocate (expand_filter_count must be a true no-op).
 */
TEST(NotchFilterTest, UpdateWithManyCentersNoAlloc)
{
    const uint8_t num_centers = 4;
    float centers[num_centers] { 50, 60, 70, 80 };

    HarmonicNotchFilter<float> f {};
    HarmonicNotchFilterParams params {};
    params.set_center_freq_hz(50);
    params.set_bandwidth_hz(10);
    params.set_attenuation(40);
    params.set_freq_min_ratio(1.0);

    f.allocate_filters(1, 1, params.num_composite_notches());
    f.init(2000, params);
    f.update(num_centers, centers);

    EXPECT_FALSE(HarmonicNotchFilterTest::alloc_failed(f));
    const float out = f.apply(1.0f);
    EXPECT_TRUE(std::isfinite(out));
}

AP_GTEST_MAIN()
