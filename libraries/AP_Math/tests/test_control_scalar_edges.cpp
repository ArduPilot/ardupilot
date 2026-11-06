#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/control.h>

// Verifies jerk-limited accel changes in both positive and negative directions.
TEST(ControlScalarEdges, ShapeAccelJerkLimit)
{
    float accel = 0.0f;
    shape_accel(10.0f, accel, 2.0f, 0.5f);
    EXPECT_FLOAT_EQ(accel, 1.0f);

    shape_accel(-10.0f, accel, 2.0f, 0.5f);
    EXPECT_FLOAT_EQ(accel, 0.0f);
}

// Ensures non-positive dt does not update the tracked acceleration state.
TEST(ControlScalarEdges, ShapeAccelNoUpdateWhenDtNonPositive)
{
    float accel = 3.0f;
    shape_accel(10.0f, accel, 2.0f, 0.0f);
    EXPECT_FLOAT_EQ(accel, 3.0f);

    shape_accel(-10.0f, accel, 2.0f, -0.1f);
    EXPECT_FLOAT_EQ(accel, 3.0f);
}

// Confirms limit_total toggles whether acceleration is clamped to accel_max.
TEST(ControlScalarEdges, ShapeVelAccelLimitTotalSwitch)
{
    float accel_limit_off = 0.0f;
    shape_vel_accel(0.0f, 10.0f,
                    0.0f, accel_limit_off,
                    -2.0f, 2.0f,
                    10.0f, 1.0f, false);
    EXPECT_FLOAT_EQ(accel_limit_off, 10.0f);

    float accel_limit_on = 0.0f;
    shape_vel_accel(0.0f, 10.0f,
                    0.0f, accel_limit_on,
                    -2.0f, 2.0f,
                    10.0f, 1.0f, true);
    EXPECT_FLOAT_EQ(accel_limit_on, 2.0f);
}

// Covers low-pass alpha edge branches and a nominal RC-derived alpha case.
TEST(ControlScalarEdges, CalcLowpassAlphaDtBranches)
{
    EXPECT_FLOAT_EQ(calc_lowpass_alpha_dt(0.01f, 0.0f), 1.0f);
    EXPECT_FLOAT_EQ(calc_lowpass_alpha_dt(0.0f, 10.0f), 0.0f);

    const float dt = 0.01f;
    const float cutoff = 10.0f;
    const float rc = 1.0f / (M_2PI * cutoff);
    const float expected = dt / (dt + rc);
    EXPECT_NEAR(calc_lowpass_alpha_dt(dt, cutoff), expected, 1.0e-6f);
}

// Checks interpolation behavior for upper clamp and reversed input ranges.
TEST(ControlScalarEdges, LinearInterpolateUpperAndReversedPolarity)
{
    EXPECT_FLOAT_EQ(linear_interpolate(0.0f, 100.0f, 20.0f, 0.0f, 10.0f), 100.0f);
    EXPECT_FLOAT_EQ(linear_interpolate(0.0f, 100.0f, 12.0f, 10.0f, 0.0f), 0.0f);
}

// Validates negative-angle wrapping for degree and centi-degree variants.
TEST(ControlScalarEdges, WrapNegativeAngles)
{
    EXPECT_FLOAT_EQ(wrap_360(-10.0f), 350.0f);
    EXPECT_EQ(wrap_360(-10), 350);
    EXPECT_FLOAT_EQ(wrap_360_cd(-100.0f), 35900.0f);
}

// Exercises sign and non-sign branches in two's-complement conversion.
TEST(ControlScalarEdges, GetTwosComplementSignBranch)
{
    EXPECT_EQ(get_twos_complement(0xFFu, 8), -1);
    EXPECT_EQ(get_twos_complement(0x7Fu, 8), 127);
}

int hal;

AP_GTEST_MAIN()