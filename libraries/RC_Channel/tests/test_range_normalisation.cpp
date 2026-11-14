/*
  Tests for RC_Channel input normalisation.

  Real RC_Channel objects are configured through their parameters and
  driven with set_radio_in().  The results are compared against the
  formulas RC_Channel used before input was normalised.  Those formulas
  are copied verbatim from RC_Channel.cpp at the merge base, with
  members replaced by plain fields; do not "fix" them.

  Covered:
  - RANGE channels: calc_norm_input_dz()*1000 vs get_control_in()
    (pwm_to_range_dz() with high_in = 1000)
  - ANGLE channels: calc_norm_input_dz()*4500 vs get_control_in()
    (pwm_to_angle_dz_trim() with high_in = 4500)
  - norm_input_mid()*1000 vs get_control_mid()
  - norm_input_dz_trim(0, trim) vs the old norm_input(), which was
    trim-centred regardless of channel type (Sub and Plane rely on it)
  - norm_input_centered_on_trim() and norm_input_dz_centered_on_trim() vs the old
    norm_input() and norm_input_dz() on both channel types, and their
    immunity to set_norm_in()
 */

#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <RC_Channel/RC_Channel.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// ---------------------------------------------------------------------------
// Test configurations
// ---------------------------------------------------------------------------

struct RCConfig {
    uint16_t radio_min;
    uint16_t radio_max;
    uint16_t radio_trim;
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
    {  1000,      2000,      1200,        30,  "trim 300 below centre" },
    {  1000,      2000,      1700,        30,  "trim 200 above centre" },
    {  1000,      2000,      1000,        30,  "trim at minimum (heli style)" },
    {  1000,      2000,      2000,        30,  "trim at maximum" },
    {   982,      2006,      1100,        30,  "wide range, low trim" },
    {   982,      2006,      1900,        30,  "wide range, high trim" },
};

// PWM sweep extends beyond the calibrated range to exercise clamping
static constexpr int16_t SWEEP_MARGIN = 100;

// ---------------------------------------------------------------------------
// Old formulas, copied from origin/master RC_Channel.cpp
// ---------------------------------------------------------------------------

struct OldChannel {
    int16_t radio_in;
    int16_t radio_min;
    int16_t radio_max;
    int16_t radio_trim;
    uint16_t dead_zone;
    bool reversed;
    int16_t high_in;

    float pwm_to_range_dz(uint16_t _dead_zone) const
    {
        int16_t r_in = constrain_int16(radio_in, radio_min, radio_max);

        if (reversed) {
            r_in = radio_max - (r_in - radio_min);
        }

        int16_t radio_trim_low  = radio_min + _dead_zone;

        if (r_in > radio_trim_low) {
            return (((float)(high_in) * (float)(r_in - radio_trim_low)) / (float)(radio_max - radio_trim_low));
        }
        return 0;
    }

    float pwm_to_angle_dz_trim(uint16_t _dead_zone, uint16_t _trim) const
    {
        int16_t radio_trim_high = _trim + _dead_zone;
        int16_t radio_trim_low  = _trim - _dead_zone;

        float reverse_mul = (reversed?-1:1);

        // don't allow out of range values
        int16_t r_in = constrain_int16(radio_in, radio_min, radio_max);

        if (r_in > radio_trim_high && radio_max != radio_trim_high) {
            return reverse_mul * ((float)high_in * (float)(r_in - radio_trim_high)) / (float)(radio_max  - radio_trim_high);
        } else if (r_in < radio_trim_low && radio_trim_low != radio_min) {
            return reverse_mul * ((float)high_in * (float)(r_in - radio_trim_low)) / (float)(radio_trim_low - radio_min);
        } else {
            return 0;
        }
    }

    // RANGE channels only
    int16_t get_control_mid() const
    {
        int16_t r_in = (radio_min + radio_max)/2;

        int16_t radio_trim_low  = radio_min + dead_zone;

        return (((int32_t)(high_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    }

    float norm_input() const
    {
        float ret;
        int16_t reverse_mul = (reversed?-1:1);
        if (radio_in < radio_trim) {
            if (radio_min >= radio_trim) {
                return 0.0f;
            }
            ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
        } else {
            if (radio_max <= radio_trim) {
                return 0.0f;
            }
            ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
        }
        return constrain_float(ret, -1.0f, 1.0f);
    }

    float norm_input_dz() const
    {
        int16_t dz_min = radio_trim - dead_zone;
        int16_t dz_max = radio_trim + dead_zone;
        float ret;
        int16_t reverse_mul = (reversed?-1:1);
        if (radio_in < dz_min && dz_min > radio_min) {
            ret = reverse_mul * (float)(radio_in - dz_min) / (float)(dz_min - radio_min);
        } else if (radio_in > dz_max && radio_max > dz_max) {
            ret = reverse_mul * (float)(radio_in - dz_max) / (float)(radio_max  - dz_max);
        } else {
            ret = 0;
        }
        return constrain_float(ret, -1.0f, 1.0f);
    }

    // what update() stored in control_in
    int16_t get_control_in(bool range) const
    {
        if (range) {
            return pwm_to_range_dz(dead_zone);
        }
        return pwm_to_angle_dz_trim(dead_zone, radio_trim);
    }
};

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

class TestChannel : public RC_Channel {
public:
    void configure(const RCConfig &c, bool _reversed, bool range)
    {
        ASSERT_TRUE(AP_Param::set_object_value(this, var_info, "MIN", c.radio_min));
        ASSERT_TRUE(AP_Param::set_object_value(this, var_info, "MAX", c.radio_max));
        ASSERT_TRUE(AP_Param::set_object_value(this, var_info, "TRIM", c.radio_trim));
        ASSERT_TRUE(AP_Param::set_object_value(this, var_info, "DZ", c.dead_zone));
        ASSERT_TRUE(AP_Param::set_object_value(this, var_info, "REVERSED", _reversed ? 1 : 0));
        if (range) {
            set_range();
        } else {
            set_angle();
        }
    }
};

static OldChannel old_channel(const RCConfig &c, bool reversed, int16_t high_in)
{
    return OldChannel{0, int16_t(c.radio_min), int16_t(c.radio_max), int16_t(c.radio_trim), c.dead_zone, reversed, high_in};
}

// old int16 control_in vs new float scaled back up; allow one count of
// float rounding
static constexpr int16_t COUNT_TOLERANCE = 1;

// ---------------------------------------------------------------------------
// RANGE channels
// ---------------------------------------------------------------------------

TEST(RCChannelNorm, RangeMatchesOldGetControlIn)
{
    for (const auto &c : configs) {
        for (bool reversed : {false, true}) {
            TestChannel ch;
            ch.configure(c, reversed, true);
            OldChannel old = old_channel(c, reversed, 1000);
            for (int16_t pwm = c.radio_min - SWEEP_MARGIN; pwm <= c.radio_max + SWEEP_MARGIN; pwm++) {
                ch.set_radio_in(pwm);
                old.radio_in = pwm;
                EXPECT_NEAR(int16_t(ch.calc_norm_input_dz() * 1000.0f), old.get_control_in(true), COUNT_TOLERANCE)
                    << c.label << " reversed=" << reversed << " pwm=" << pwm;
            }
        }
    }
}

TEST(RCChannelNorm, RangeEndpoints)
{
    for (const auto &c : configs) {
        TestChannel ch;
        ch.configure(c, false, true);
        ch.set_radio_in(c.radio_min);
        EXPECT_FLOAT_EQ(ch.calc_norm_input_dz(), 0.0f) << c.label;
        ch.set_radio_in(c.radio_max);
        EXPECT_FLOAT_EQ(ch.calc_norm_input_dz(), 1.0f) << c.label;

        TestChannel rev;
        rev.configure(c, true, true);
        rev.set_radio_in(c.radio_min);
        EXPECT_FLOAT_EQ(rev.calc_norm_input_dz(), 1.0f) << c.label << " reversed";
        rev.set_radio_in(c.radio_max);
        EXPECT_FLOAT_EQ(rev.calc_norm_input_dz(), 0.0f) << c.label << " reversed";
    }
}

TEST(RCChannelNorm, RangeMonotonic)
{
    for (const auto &c : configs) {
        TestChannel ch;
        ch.configure(c, false, true);
        float prev = -1.0f;
        for (int16_t pwm = c.radio_min - SWEEP_MARGIN; pwm <= c.radio_max + SWEEP_MARGIN; pwm++) {
            ch.set_radio_in(pwm);
            const float v = ch.calc_norm_input_dz();
            EXPECT_GE(v, prev) << c.label << " pwm=" << pwm;
            prev = v;
        }
    }
}

// ---------------------------------------------------------------------------
// ANGLE channels
// ---------------------------------------------------------------------------

TEST(RCChannelNorm, AngleMatchesOldGetControlIn)
{
    for (const auto &c : configs) {
        for (bool reversed : {false, true}) {
            TestChannel ch;
            ch.configure(c, reversed, false);
            OldChannel old = old_channel(c, reversed, 4500);
            for (int16_t pwm = c.radio_min - SWEEP_MARGIN; pwm <= c.radio_max + SWEEP_MARGIN; pwm++) {
                ch.set_radio_in(pwm);
                old.radio_in = pwm;
                EXPECT_NEAR(int16_t(ch.calc_norm_input_dz() * 4500.0f), old.get_control_in(false), COUNT_TOLERANCE)
                    << c.label << " reversed=" << reversed << " pwm=" << pwm;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Stick centre
// ---------------------------------------------------------------------------

TEST(RCChannelMid, MatchesOldGetControlMid)
{
    for (const auto &c : configs) {
        TestChannel ch;
        ch.configure(c, false, true);
        const OldChannel old = old_channel(c, false, 1000);
        EXPECT_NEAR(int16_t(ch.norm_input_mid() * 1000.0f), old.get_control_mid(), COUNT_TOLERANCE) << c.label;
    }
}

TEST(RCChannelMid, AngleIsZero)
{
    for (const auto &c : configs) {
        TestChannel ch;
        ch.configure(c, false, false);
        EXPECT_FLOAT_EQ(ch.norm_input_mid(), 0.0f) << c.label;
    }
}

// old get_control_mid() went to zero or negative once the deadzone
// reached mid-stick; the new code returns 0.5 instead
TEST(RCChannelMid, DeadzoneReachingMidStickGivesHalf)
{
    const RCConfig c { 1000, 2000, 1500, 500, "deadzone reaches mid-stick" };
    TestChannel ch;
    ch.configure(c, false, true);
    EXPECT_FLOAT_EQ(ch.norm_input_mid(), 0.5f);
    EXPECT_EQ(old_channel(c, false, 1000).get_control_mid(), 0);
}

// ---------------------------------------------------------------------------
// Trim-centred input on a RANGE channel (old norm_input() semantics)
// ---------------------------------------------------------------------------

TEST(RCChannelNorm, TrimCentredMatchesOldNormInput)
{
    for (const auto &c : configs) {
        for (bool reversed : {false, true}) {
            TestChannel ch;
            ch.configure(c, reversed, true);
            OldChannel old = old_channel(c, reversed, 1000);
            for (int16_t pwm = c.radio_min - SWEEP_MARGIN; pwm <= c.radio_max + SWEEP_MARGIN; pwm++) {
                ch.set_radio_in(pwm);
                old.radio_in = pwm;
                EXPECT_NEAR(ch.norm_input_dz_trim(0, c.radio_trim) * 0.5f + 0.5f,
                            (old.norm_input() + 1.0f) / 2.0f, 1e-5f)
                    << c.label << " reversed=" << reversed << " pwm=" << pwm;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Trim-centred inputs for auxiliary consumers (old norm_input() and
// norm_input_dz() semantics on any channel type)
// ---------------------------------------------------------------------------

TEST(RCChannelNorm, CenteredMatchesOldNormInputs)
{
    for (const auto &c : configs) {
        for (bool reversed : {false, true}) {
            for (bool range : {false, true}) {
                TestChannel ch;
                ch.configure(c, reversed, range);
                OldChannel old = old_channel(c, reversed, range ? 1000 : 4500);
                for (int16_t pwm = c.radio_min - SWEEP_MARGIN; pwm <= c.radio_max + SWEEP_MARGIN; pwm++) {
                    ch.set_radio_in(pwm);
                    old.radio_in = pwm;
                    EXPECT_FLOAT_EQ(ch.norm_input_centered_on_trim(), old.norm_input())
                        << c.label << " reversed=" << reversed << " range=" << range << " pwm=" << pwm;
                    EXPECT_FLOAT_EQ(ch.norm_input_dz_centered_on_trim(), old.norm_input_dz())
                        << c.label << " reversed=" << reversed << " range=" << range << " pwm=" << pwm;
                }
            }
        }
    }
}

// the tailsitter swaps roll and yaw with set_norm_in(); consumers of the
// centered inputs must keep seeing the physical stick
TEST(RCChannelNorm, CenteredIgnoresSetNormIn)
{
    const RCConfig &c = configs[0];
    TestChannel ch;
    ch.configure(c, false, false);
    ch.set_radio_in(1900);
    const float physical = ch.norm_input_dz_centered_on_trim();
    ch.set_norm_in(-0.25f);
    EXPECT_FLOAT_EQ(ch.norm_input_dz(), -0.25f);
    EXPECT_FLOAT_EQ(ch.norm_input_dz_centered_on_trim(), physical);
    EXPECT_FLOAT_EQ(ch.norm_input_centered_on_trim(), 0.8f);
}

AP_GTEST_MAIN()
