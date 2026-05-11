/*
  Stand-alone tests for RC_Channel RANGE input normalisation.

  Two areas are covered:

  1. pwm_to_range_dz() / get_control_in() equivalence
     The old get_control_in() for RANGE channels returned
         int16_t(pwm_to_range_dz(dead_zone) * 1000)
     The replacement is
         int16_t(norm_input_dz() * 1000)
     Both implementations are inlined here so the test has no external
     dependencies beyond AP_Math and gtest.

  2. get_throttle_mid() correctness
     The original get_control_mid() used the geometric midpoint
     (radio_min + radio_max) / 2 as the reference PWM value.
     A bug introduced during the normalisation refactor substituted
     radio_trim for that midpoint, shifting the throttle-expo centre
     for any pilot whose trim was not exactly at the geometric centre.
     The fixed version and the buggy version are both inlined so the
     test can demonstrate both the correct behaviour and the regression.
*/

#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// ---------------------------------------------------------------------------
// Standalone implementations — no RC_Channel object required
// ---------------------------------------------------------------------------

/*
 * Old pwm_to_range_dz() from master:libraries/RC_Channel/RC_Channel.cpp.
 * Returns [0, 1]: 0 at (radio_min + dead_zone), 1 at radio_max.
 */
static float pwm_to_range_dz(uint16_t radio_in, uint16_t radio_min, uint16_t radio_max,
                              uint16_t dead_zone)
{
    int16_t r_in = constrain_int16(int16_t(radio_in), int16_t(radio_min), int16_t(radio_max));
    float in_min = float(radio_min);
    if (dead_zone > 0) {
        if (radio_max - radio_min <= 2 * dead_zone) {
            return (r_in > (radio_max + radio_min) / 2) ? 1.0f : 0.0f;
        }
        in_min += float(dead_zone);
    }
    return constrain_float((float(r_in) - in_min) / (float(radio_max) - in_min), 0.0f, 1.0f);
}

/*
 * Old get_control_in() for RANGE: pwm_to_range_dz * high_in, high_in = 1000.
 */
static int16_t get_control_in_range(uint16_t radio_in, uint16_t radio_min, uint16_t radio_max,
                                    uint16_t dead_zone)
{
    return int16_t(pwm_to_range_dz(radio_in, radio_min, radio_max, dead_zone) * 1000.0f);
}

/*
 * New norm_input_dz() for RANGE channels, from RC_Channel::update().
 * Identical semantics to pwm_to_range_dz but written without high_in.
 */
static float norm_input_dz_range(uint16_t radio_in, uint16_t radio_min, uint16_t radio_max,
                                  uint16_t dead_zone)
{
    const int16_t r_in = constrain_int16(int16_t(radio_in), int16_t(radio_min), int16_t(radio_max));
    const int16_t dead_zone_top = int16_t(radio_min) + int16_t(dead_zone);
    if (r_in <= dead_zone_top) {
        return 0.0f;
    }
    return constrain_float(float(r_in - dead_zone_top) / float(int16_t(radio_max) - dead_zone_top),
                           0.0f, 1.0f);
}

/*
 * Original get_control_mid() from master:RC_Channel.cpp.
 * Uses the geometric midpoint (radio_min + radio_max) / 2 as reference PWM.
 * Returns a value in [0, 1000] representing where that midpoint sits in the
 * normalised throttle range.
 */
static int16_t get_control_mid(uint16_t radio_min, uint16_t radio_max, uint16_t dead_zone)
{
    const int16_t r_in = int16_t((radio_min + radio_max) / 2);
    const int16_t radio_trim_low = int16_t(radio_min) + int16_t(dead_zone);
    return int16_t(((int32_t)1000 * int32_t(r_in - radio_trim_low))
                   / int32_t(int16_t(radio_max) - radio_trim_low));
}

/*
 * Fixed get_throttle_mid() from ArduCopter/radio.cpp (after the bug fix).
 * Also uses the geometric midpoint — should match get_control_mid() exactly.
 */
static int16_t get_throttle_mid_fixed(uint16_t radio_min, uint16_t radio_max, uint16_t dead_zone)
{
    const int16_t r_min = int16_t(radio_min);
    const int16_t r_max = int16_t(radio_max);
    const int16_t r_mid = (r_min + r_max) / 2;
    const int16_t dead_zone_top = r_min + int16_t(dead_zone);
    if (dead_zone_top >= r_max || r_mid <= dead_zone_top) {
        return 500;
    }
    return int16_t(1000.0f * float(r_mid - dead_zone_top) / float(r_max - dead_zone_top));
}

/*
 * Buggy get_throttle_mid() — the version that was introduced during the
 * normalisation refactor.  Uses radio_trim instead of the geometric midpoint,
 * so the expo-curve centre drifts whenever the pilot's trim is not exactly
 * at (radio_min + radio_max) / 2.
 */
static int16_t get_throttle_mid_buggy(uint16_t radio_min, uint16_t radio_max,
                                      uint16_t radio_trim, uint16_t dead_zone)
{
    const int16_t r_max = int16_t(radio_max);
    const int16_t r_trim = int16_t(radio_trim);
    const int16_t dead_zone_top = int16_t(radio_min) + int16_t(dead_zone);
    if (dead_zone_top >= r_max || r_trim <= dead_zone_top) {
        return 500;
    }
    return int16_t(1000.0f * float(r_trim - dead_zone_top) / float(r_max - dead_zone_top));
}

// ---------------------------------------------------------------------------
// Test parameters
// ---------------------------------------------------------------------------

struct RCConfig {
    uint16_t radio_min;
    uint16_t radio_max;
    uint16_t radio_trim;   // used only by buggy variant
    uint16_t dead_zone;
    const char *label;
};

static const RCConfig configs[] = {
    // radio_min  radio_max  radio_trim  dead_zone   label
    {  1000,      2000,      1500,        30,  "typical centred" },
    {   982,      2006,      1500,        30,  "wide range centred trim" },
    {   988,      2012,      1500,        30,  "common asymmetric range" },
    {  1000,      2000,      1500,         0,  "no dead zone" },
    {  1000,      2000,      1500,       100,  "large dead zone" },
    {  1000,      1800,      1400,        30,  "short throw" },
    // Trim offset cases — these expose the get_throttle_mid bug:
    {  1000,      2000,      1200,        30,  "trim 300 below centre" },
    {  1000,      2000,      1700,        30,  "trim 200 above centre" },
    {  1000,      2000,      1000,        30,  "trim at minimum (heli style)" },
    {  1000,      2000,      2000,        30,  "trim at maximum" },
    // Wider real-world range with off-centre trim:
    {   982,      2006,      1100,        30,  "wide range, low trim" },
    {   982,      2006,      1900,        30,  "wide range, high trim" },
};

// ---------------------------------------------------------------------------
// Helper
// ---------------------------------------------------------------------------

// The two integer implementations use different rounding (floor integer
// division vs float truncation); allow ±1 count tolerance.
static constexpr int16_t MID_TOLERANCE = 1;

// ---------------------------------------------------------------------------
// Tests: pwm_to_range_dz vs norm_input_dz
// ---------------------------------------------------------------------------

// norm_input_dz() for RANGE must give the same [0,1000] reading as the old
// get_control_in() across the full stick travel.
TEST(RCRangeNorm, EquivalentToOldGetControlIn)
{
    for (const auto& c : configs) {
        for (uint16_t pwm = c.radio_min; pwm <= c.radio_max; pwm += 5) {
            const int16_t old_val = get_control_in_range(pwm, c.radio_min, c.radio_max, c.dead_zone);
            const int16_t new_val = int16_t(norm_input_dz_range(pwm, c.radio_min, c.radio_max, c.dead_zone) * 1000.0f);
            EXPECT_EQ(old_val, new_val)
                << c.label
                << " min=" << c.radio_min << " max=" << c.radio_max
                << " dz=" << c.dead_zone << " pwm=" << pwm;
        }
    }
}

// At stick bottom the output must be zero (deadzone clamps it).
TEST(RCRangeNorm, BottomStickIsZero)
{
    for (const auto& c : configs) {
        EXPECT_EQ(0, int16_t(norm_input_dz_range(c.radio_min, c.radio_min, c.radio_max, c.dead_zone) * 1000.0f))
            << c.label;
    }
}

// At stick top the output must be 1000.
TEST(RCRangeNorm, TopStickIs1000)
{
    for (const auto& c : configs) {
        EXPECT_EQ(1000, int16_t(norm_input_dz_range(c.radio_max, c.radio_min, c.radio_max, c.dead_zone) * 1000.0f))
            << c.label;
    }
}

// Output must be non-decreasing as PWM increases.
TEST(RCRangeNorm, Monotonic)
{
    for (const auto& c : configs) {
        int16_t prev = 0;
        for (uint16_t pwm = c.radio_min; pwm <= c.radio_max; pwm += 5) {
            const int16_t val = int16_t(norm_input_dz_range(pwm, c.radio_min, c.radio_max, c.dead_zone) * 1000.0f);
            EXPECT_GE(val, prev) << c.label << " pwm=" << pwm;
            prev = val;
        }
    }
}

// ---------------------------------------------------------------------------
// Tests: get_throttle_mid
// ---------------------------------------------------------------------------

// Fixed get_throttle_mid() must match the original get_control_mid() for all
// configs.  Note: get_control_mid uses integer division; get_throttle_mid_fixed
// uses float division — allow ±1 for rounding differences.
TEST(ThrottleMid, FixedMatchesOriginalGetControlMid)
{
    for (const auto& c : configs) {
        const int16_t original = get_control_mid(c.radio_min, c.radio_max, c.dead_zone);
        const int16_t fixed    = get_throttle_mid_fixed(c.radio_min, c.radio_max, c.dead_zone);
        EXPECT_NEAR(original, fixed, MID_TOLERANCE)
            << c.label
            << " min=" << c.radio_min << " max=" << c.radio_max
            << " dz=" << c.dead_zone;
    }
}

// Midpoint must always lie within [0, 1000].
TEST(ThrottleMid, OutputInValidRange)
{
    for (const auto& c : configs) {
        const int16_t mid = get_throttle_mid_fixed(c.radio_min, c.radio_max, c.dead_zone);
        EXPECT_GE(mid, 0)   << c.label;
        EXPECT_LE(mid, 1000) << c.label;
    }
}

// Symmetric range with no dead zone: midpoint must be exactly 500.
TEST(ThrottleMid, SymmetricNoDzGives500)
{
    EXPECT_EQ(500, get_throttle_mid_fixed(1000, 2000, 0));
    EXPECT_EQ(500, get_throttle_mid_fixed( 800, 1800, 0));
    EXPECT_EQ(500, get_throttle_mid_fixed(1100, 1900, 0));
}

// The buggy variant agrees with the correct one only when radio_trim equals
// the geometric centre of the PWM range.  When trim is offset the buggy
// variant produces a different (wrong) midpoint.
TEST(ThrottleMid, BuggyVariantDiffersWhenTrimIsOffCentre)
{
    for (const auto& c : configs) {
        const int16_t correct = get_throttle_mid_fixed(c.radio_min, c.radio_max, c.dead_zone);
        const int16_t buggy   = get_throttle_mid_buggy(c.radio_min, c.radio_max,
                                                        c.radio_trim, c.dead_zone);
        const int16_t geometric_centre = int16_t((c.radio_min + c.radio_max) / 2);

        if (c.radio_trim == geometric_centre) {
            // When trim is at centre both versions must agree.
            EXPECT_NEAR(correct, buggy, MID_TOLERANCE)
                << c.label << " (trim at centre — both variants should agree)";
        } else {
            // When trim is off-centre the buggy variant drifts from the
            // correct midpoint.  The difference is proportional to how far
            // trim sits from (radio_min + radio_max) / 2.
            EXPECT_NE(correct, buggy)
                << c.label
                << " (trim=" << c.radio_trim << " != centre " << geometric_centre
                << " — buggy variant should give wrong midpoint)";
        }
    }
}

// Quantify the error introduced by the bug for a concrete case:
// min=1000 max=2000 trim=1700 dz=30
//   correct: based on geometric centre 1500 → ~484
//   buggy:   based on trim 1700         → ~690
// That is a 206-count error on a 0-1000 scale (~20% of full range).
TEST(ThrottleMid, BugMagnitudeExample)
{
    const int16_t correct = get_throttle_mid_fixed(1000, 2000, 30);
    const int16_t buggy   = get_throttle_mid_buggy(1000, 2000, 1700, 30);
    EXPECT_NEAR(correct, 484, 1);
    EXPECT_NEAR(buggy,   690, 1);
    EXPECT_GT(std::abs(correct - buggy), 100)
        << "bug should produce a large midpoint shift, got correct="
        << correct << " buggy=" << buggy;
}

AP_GTEST_MAIN()
