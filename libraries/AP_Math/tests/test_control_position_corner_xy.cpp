#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/control.h>

// Confirms XY shaping respects limit_total toggle for accel magnitude.
TEST(ControlPositionCornerXY, ShapePosVelAccelXyLimitTotalSwitch)
{
    Vector2f accel_limit_off;
    shape_pos_vel_accel_xy(Vector2p(), Vector2f(), Vector2f(10.0f, 0.0f),
                           Vector2p(), Vector2f(), accel_limit_off,
                           5.0f, 2.0f,
                           10.0f, 1.0f, false);
    EXPECT_FLOAT_EQ(accel_limit_off.x, 10.0f);
    EXPECT_FLOAT_EQ(accel_limit_off.y, 0.0f);

    Vector2f accel_limit_on;
    shape_pos_vel_accel_xy(Vector2p(), Vector2f(), Vector2f(10.0f, 0.0f),
                           Vector2p(), Vector2f(), accel_limit_on,
                           5.0f, 2.0f,
                           10.0f, 1.0f, true);
    EXPECT_FLOAT_EQ(accel_limit_on.x, 2.0f);
    EXPECT_FLOAT_EQ(accel_limit_on.y, 0.0f);
}

// Ensures dt=0 leaves XY acceleration untouched in vector shaping helper.
TEST(ControlPositionCornerXY, ShapePosVelAccelXyNoUpdateWhenDtZero)
{
    Vector2f accel(1.0f, -2.0f);
    shape_pos_vel_accel_xy(Vector2p(5.0f, 0.0f), Vector2f(), Vector2f(),
                           Vector2p(), Vector2f(), accel,
                           5.0f, 2.0f,
                           10.0f, 0.0f, true);
    EXPECT_FLOAT_EQ(accel.x, 1.0f);
    EXPECT_FLOAT_EQ(accel.y, -2.0f);
}

// Verifies invalid accel_max exits corner limiter without changing accel.
TEST(ControlPositionCornerXY, LimitAccelCornerXyInvalidLimit)
{
    Vector2f accel(3.0f, 4.0f);
    EXPECT_FALSE(limit_accel_corner_xy(Vector2f(1.0f, 0.0f), accel, 0.0f));
    EXPECT_FLOAT_EQ(accel.x, 3.0f);
    EXPECT_FLOAT_EQ(accel.y, 4.0f);
}

// Covers zero-velocity corner case where limiter falls back to vector clamp.
TEST(ControlPositionCornerXY, LimitAccelCornerXyZeroVelocityBranch)
{
    Vector2f accel(3.0f, 4.0f);
    EXPECT_TRUE(limit_accel_corner_xy(Vector2f(), accel, 4.0f));
    EXPECT_NEAR(accel.length(), 4.0f, 1.0e-6f);
}

// Verifies non-braking path favors cross-track acceleration when saturated.
TEST(ControlPositionCornerXY, LimitAccelCornerXyNonBrakingPriority)
{
    Vector2f accel(4.0f, 4.0f);
    EXPECT_TRUE(limit_accel_corner_xy(Vector2f(1.0f, 0.0f), accel, 4.0f));
    EXPECT_NEAR(accel.x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(accel.y, 4.0f, 1.0e-6f);
}

// Verifies braking path preserves along-track decel before cross-track accel.
TEST(ControlPositionCornerXY, LimitAccelCornerXyBrakingPriority)
{
    Vector2f accel(-5.0f, 4.0f);
    EXPECT_TRUE(limit_accel_corner_xy(Vector2f(1.0f, 0.0f), accel, 4.0f));
    EXPECT_NEAR(accel.x, -4.0f, 1.0e-6f);
    EXPECT_NEAR(accel.y, 0.0f, 1.0e-6f);
}

int hal;

AP_GTEST_MAIN()