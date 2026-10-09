#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/control.h>

// Confirms limit_total controls clamping in scalar position-velocity shaping.
TEST(ControlPositionLimitXY, ShapePosVelAccelLimitTotalSwitch)
{
    float accel_limit_off = 0.0f;
    shape_pos_vel_accel(0.0f, 0.0f, 10.0f,
                        0.0f, 0.0f, accel_limit_off,
                        -5.0f, 5.0f,
                        -2.0f, 2.0f,
                        10.0f, 1.0f, false);
    EXPECT_FLOAT_EQ(accel_limit_off, 10.0f);

    float accel_limit_on = 0.0f;
    shape_pos_vel_accel(0.0f, 0.0f, 10.0f,
                        0.0f, 0.0f, accel_limit_on,
                        -5.0f, 5.0f,
                        -2.0f, 2.0f,
                        10.0f, 1.0f, true);
    EXPECT_FLOAT_EQ(accel_limit_on, 2.0f);
}

// Ensures dt=0 leaves acceleration unchanged in scalar shaping helper.
TEST(ControlPositionLimitXY, ShapePosVelAccelNoUpdateWhenDtZero)
{
    float accel = 1.0f;
    shape_pos_vel_accel(5.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, accel,
                        -5.0f, 5.0f,
                        -2.0f, 2.0f,
                        10.0f, 0.0f, true);
    EXPECT_FLOAT_EQ(accel, 1.0f);
}

// Verifies invalid accel_max exits early without mutating accel_xy.
TEST(ControlPositionLimitXY, LimitAccelXyInvalidLimit)
{
    Vector2f accel(3.0f, 4.0f);
    EXPECT_FALSE(limit_accel_xy(Vector2f(1.0f, 0.0f), accel, 0.0f));
    EXPECT_FLOAT_EQ(accel.x, 3.0f);
    EXPECT_FLOAT_EQ(accel.y, 4.0f);
}

// Verifies values already inside limit remain unchanged and return false.
TEST(ControlPositionLimitXY, LimitAccelXyWithinLimit)
{
    Vector2f accel(1.0f, 1.0f);
    EXPECT_FALSE(limit_accel_xy(Vector2f(1.0f, 0.0f), accel, 2.0f));
    EXPECT_FLOAT_EQ(accel.x, 1.0f);
    EXPECT_FLOAT_EQ(accel.y, 1.0f);
}

// Covers zero-velocity branch where limiting is applied by vector magnitude.
TEST(ControlPositionLimitXY, LimitAccelXyZeroVelocityBranch)
{
    Vector2f accel(3.0f, 4.0f);
    EXPECT_TRUE(limit_accel_xy(Vector2f(), accel, 4.0f));
    EXPECT_NEAR(accel.length(), 4.0f, 1.0e-6f);
}

// Validates cross-track priority when along-track and cross-track compete.
TEST(ControlPositionLimitXY, LimitAccelXyCrossTrackPriority)
{
    Vector2f accel(4.0f, 4.0f);
    EXPECT_TRUE(limit_accel_xy(Vector2f(10.0f, 0.0f), accel, 4.0f));
    EXPECT_NEAR(accel.x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(accel.y, 4.0f, 1.0e-6f);
}

// Confirms dominant cross-track demand gets retained at the accel limit.
TEST(ControlPositionLimitXY, LimitAccelXyCrossTrackDominant)
{
    Vector2f accel(1.0f, 10.0f);
    EXPECT_TRUE(limit_accel_xy(Vector2f(10.0f, 0.0f), accel, 4.0f));
    EXPECT_NEAR(accel.x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(accel.y, 4.0f, 1.0e-6f);
}

int hal;

AP_GTEST_MAIN()