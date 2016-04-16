#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

#define SQRT_2 1.4142135623730951f

TEST(VectorTest, Rotations)
{
    unsigned rotation_count = 0;

#define TEST_ROTATION(rotation, _x, _y, _z) { \
    const float accuracy = 1.0e-6; \
    Vector3f v(1, 1, 1); \
    v.rotate(rotation); \
    Vector3f expected(_x, _y, _z); \
    EXPECT_NEAR(expected.length(), v.length(), accuracy); \
    EXPECT_FLOAT_EQ(expected.x, v.x); \
    EXPECT_FLOAT_EQ(expected.y, v.y); \
    EXPECT_FLOAT_EQ(expected.z, v.z); \
    rotation_count++; \
}

    TEST_ROTATION(ROTATION_NONE, 1, 1, 1);
    TEST_ROTATION(ROTATION_YAW_45, 0, SQRT_2, 1);
    TEST_ROTATION(ROTATION_YAW_90, -1, 1, 1);
    TEST_ROTATION(ROTATION_YAW_135, -SQRT_2, 0, 1);
    TEST_ROTATION(ROTATION_YAW_180, -1, -1, 1);
    TEST_ROTATION(ROTATION_YAW_225, 0, -SQRT_2, 1);
    TEST_ROTATION(ROTATION_YAW_270, 1, -1, 1);
    TEST_ROTATION(ROTATION_YAW_315, SQRT_2, 0, 1);
    TEST_ROTATION(ROTATION_ROLL_180, 1, -1, -1);
    TEST_ROTATION(ROTATION_ROLL_180_YAW_45, SQRT_2, 0, -1);
    TEST_ROTATION(ROTATION_ROLL_180_YAW_90, 1, 1, -1);
    TEST_ROTATION(ROTATION_ROLL_180_YAW_135, 0, SQRT_2, -1);
    TEST_ROTATION(ROTATION_PITCH_180, -1, 1, -1);
    TEST_ROTATION(ROTATION_ROLL_180_YAW_225, -SQRT_2, 0, -1);
    TEST_ROTATION(ROTATION_ROLL_180_YAW_270, -1, -1, -1);
    TEST_ROTATION(ROTATION_ROLL_180_YAW_315, 0, -SQRT_2, -1);
    TEST_ROTATION(ROTATION_ROLL_90, 1, -1, 1);
    TEST_ROTATION(ROTATION_ROLL_90_YAW_45, SQRT_2, 0, 1);
    TEST_ROTATION(ROTATION_ROLL_90_YAW_90, 1, 1, 1);
    TEST_ROTATION(ROTATION_ROLL_90_YAW_135, 0, SQRT_2, 1);
    TEST_ROTATION(ROTATION_ROLL_270, 1, 1, -1);
    TEST_ROTATION(ROTATION_ROLL_270_YAW_45, 0, SQRT_2, -1);
    TEST_ROTATION(ROTATION_ROLL_270_YAW_90, -1, 1, -1);
    TEST_ROTATION(ROTATION_ROLL_270_YAW_135, -SQRT_2, 0, -1);
    TEST_ROTATION(ROTATION_PITCH_90, 1, 1, -1);
    TEST_ROTATION(ROTATION_PITCH_270, -1, 1, 1);
    TEST_ROTATION(ROTATION_PITCH_180_YAW_90, -1, -1, -1);
    TEST_ROTATION(ROTATION_PITCH_180_YAW_270, 1, 1, -1);
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_90, 1, -1, -1);
    TEST_ROTATION(ROTATION_ROLL_180_PITCH_90, -1, -1, -1);
    TEST_ROTATION(ROTATION_ROLL_270_PITCH_90, -1, 1, -1);
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_180, -1, -1, -1);
    TEST_ROTATION(ROTATION_ROLL_270_PITCH_180, -1, 1, 1);
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_270, -1, -1, 1);
    TEST_ROTATION(ROTATION_ROLL_180_PITCH_270, 1, -1, 1);
    TEST_ROTATION(ROTATION_ROLL_270_PITCH_270, 1, 1, 1);
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_180_YAW_90, 1, -1, -1);
    TEST_ROTATION(ROTATION_ROLL_90_YAW_270, -1, -1, 1);
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_68_YAW_293, -0.4066309f, -1.5839677f, -0.5706992f);

    EXPECT_EQ(ROTATION_MAX, rotation_count) << "All rotations are expect to be tested";
}

TEST(MathTest, IsZero)
{
#define TEST_IS_ZERO(_result, _test) { \
    bool bZero = is_zero(_test); \
    EXPECT_EQ(_result, bZero) << "is_zero(): floating point is_zero comparator failure"; \
}

    TEST_IS_ZERO(false, 0.1);
    TEST_IS_ZERO(false, 0.0001);
    TEST_IS_ZERO(true,  0.f);
    TEST_IS_ZERO(true,  FLT_MIN);
    TEST_IS_ZERO(true, -FLT_MIN);
}

TEST(MathTest, IsEqual)
{
#define TEST_IS_EQUAL(_result, _test1, _test2) { \
    bool bEqual = is_equal(_test1, _test2); \
    EXPECT_EQ(_result, bEqual) << "is_equal(): floating point is_equal comparator failure"; \
}

    TEST_IS_EQUAL(false, 0.1,  0.10001);
    TEST_IS_EQUAL(false, 0.1, -0.1001);
    TEST_IS_EQUAL(true,  0.f,   0.0f);
    TEST_IS_EQUAL(false, 1.f,  1.f + FLT_EPSILON);
}

TEST(MathTest, Square)
{
    float sq_0 = sq(0);
    float sq_1 = sq(1);
    float sq_2 = sq(2);

    EXPECT_EQ(0.f, sq_0);
    EXPECT_EQ(1.f, sq_1);
    EXPECT_EQ(4.f, sq_2);
}

TEST(MathTest, Norm)
{
    float norm_5 = pythagorous2(3, 4);
    float norm_6 = pythagorous3(4, 3, 12);

    EXPECT_EQ(5.f, norm_5);
    EXPECT_EQ(13.f, norm_6);
}


TEST(MathTest, Constrain)
{
    for (int i = 0; i < 1000; i++) {
        if (i < 250) {
            EXPECT_EQ(250, constrain_float(i, 250, 500));
            EXPECT_EQ(250, constrain_int16(i, 250, 500));
            EXPECT_EQ(250, constrain_int32(i, 250, 500));
        } else if (i > 500) {
            EXPECT_EQ(500, constrain_float(i, 250, 500));
            EXPECT_EQ(500, constrain_int16(i, 250, 500));
            EXPECT_EQ(500, constrain_int32(i, 250, 500));
        } else {
            EXPECT_EQ(i, constrain_float(i, 250, 500));
            EXPECT_EQ(i, constrain_int16(i, 250, 500));
            EXPECT_EQ(i, constrain_int32(i, 250, 500));
        }
    }

    for (int i = 0; i <= 1000; i++) {
        int c = i - 1000;
        if (c < -250) {
            EXPECT_EQ(-250, constrain_float(c, -250, -50));
            EXPECT_EQ(-250, constrain_int16(c, -250, -50));
            EXPECT_EQ(-250, constrain_int32(c, -250, -50));
        } else if(c > -50) {
            EXPECT_EQ(-50, constrain_float(c, -250, -50));
            EXPECT_EQ(-50, constrain_int16(c, -250, -50));
            EXPECT_EQ(-50, constrain_int32(c, -250, -50));
        } else {
            EXPECT_EQ(c, constrain_float(c, -250, -50));
            EXPECT_EQ(c, constrain_int16(c, -250, -50));
            EXPECT_EQ(c, constrain_int32(c, -250, -50));
        }
    }

    for (int i = 0; i <= 2000; i++) {
        int c = i - 1000;
        if (c < -250) {
            EXPECT_EQ(-250, constrain_float(c, -250, 50));
            EXPECT_EQ(-250, constrain_int16(c, -250, 50));
            EXPECT_EQ(-250, constrain_int32(c, -250, 50));
        } else if(c > 50) {
            EXPECT_EQ(50, constrain_float(c, -250, 50));
            EXPECT_EQ(50, constrain_int16(c, -250, 50));
            EXPECT_EQ(50, constrain_int32(c, -250, 50));
        } else {
            EXPECT_EQ(c, constrain_float(c, -250, 50));
            EXPECT_EQ(c, constrain_int16(c, -250, 50));
            EXPECT_EQ(c, constrain_int32(c, -250, 50));
        }
    }
}

TEST(MathWrapTest, Angle180)
{
    // Full circle test
    for (int32_t i = 0; i < 36000; i += 100) {
        if (i < 18000) {
            // smaller pole position
            EXPECT_EQ(i, wrap_180_cd(i));
            EXPECT_EQ(-i, wrap_180_cd(-i));
        } else if (i == 18000) {
            // hit pole position -180/+180 degree
            EXPECT_EQ(i, wrap_180_cd(i));
            EXPECT_EQ(i, wrap_180_cd(-i));
        } else {
            // bigger pole position
            EXPECT_EQ(-(36000 - i), wrap_180_cd(i));
            EXPECT_EQ(36000 - i, wrap_180_cd(-i));
        }
    }

    EXPECT_EQ(4500.f,   wrap_180_cd_float(4500.f));
    EXPECT_EQ(9000.f,   wrap_180_cd_float(9000.f));
    EXPECT_EQ(18000.f,  wrap_180_cd_float(18000.f));
    EXPECT_EQ(-17990.f, wrap_180_cd_float(18010.f));
    EXPECT_EQ(-9000.f,  wrap_180_cd_float(27000.f));
    EXPECT_EQ(0.f,      wrap_180_cd_float(36000.f));
    EXPECT_EQ(0.f,      wrap_180_cd_float(72000.f));
    EXPECT_EQ(0.f,      wrap_180_cd_float(360000.f));
    EXPECT_EQ(0.f,      wrap_180_cd_float(720000.f));
    EXPECT_EQ(0.f,      wrap_180_cd_float(-3600000000.f));

    EXPECT_EQ(-4500.f, wrap_180_cd_float(-4500.f));
    EXPECT_EQ(-9000.f, wrap_180_cd_float(-9000.f));
    EXPECT_EQ(18000.f, wrap_180_cd_float(-18000.f));
    EXPECT_EQ(17990.f, wrap_180_cd_float(-18010.f));
    EXPECT_EQ(9000.f,  wrap_180_cd_float(-27000.f));
    EXPECT_EQ(0.f,     wrap_180_cd_float(-36000.f));
    EXPECT_EQ(0.f,     wrap_180_cd_float(-72000.f));
}

TEST(MathWrapTest, Angle360)
{
    // Full circle test
    for (int32_t i = 0; i <= 36000; i += 100) {
        if (i == 0) {
            // hit pole position
            EXPECT_EQ(i, wrap_360_cd(i));
            EXPECT_EQ(i, wrap_360_cd(-i));
        } else if (i < 36000) {
            // between pole position
            EXPECT_EQ(i, wrap_360_cd(i));
            EXPECT_EQ(36000 - i, wrap_360_cd(-i));
        } else if (i == 36000) {
            // hit pole position
            EXPECT_EQ(0, wrap_360_cd(i));
            EXPECT_EQ(0, wrap_360_cd(-i));
        }
    }

    EXPECT_EQ(4500.f,  wrap_360_cd_float(4500.f));
    EXPECT_EQ(9000.f,  wrap_360_cd_float(9000.f));
    EXPECT_EQ(18000.f, wrap_360_cd_float(18000.f));
    EXPECT_EQ(27000.f, wrap_360_cd_float(27000.f));
    EXPECT_EQ(0.f,     wrap_360_cd_float(36000.f));
    EXPECT_EQ(0.f,     wrap_360_cd_float(72000.f));
    EXPECT_EQ(0.f,     wrap_360_cd_float(360000.f));
    EXPECT_EQ(0.f,     wrap_360_cd_float(720000.f));
    EXPECT_EQ( 0.f,    wrap_360_cd_float(-3600000000.f));

    EXPECT_EQ(31500.f, wrap_360_cd_float(-4500.f));
    EXPECT_EQ(27000.f, wrap_360_cd_float(-9000.f));
    EXPECT_EQ(18000.f, wrap_360_cd_float(-18000.f));
    EXPECT_EQ(9000.f,  wrap_360_cd_float(-27000.f));
    EXPECT_EQ(0.f,     wrap_360_cd_float(-36000.f));
    EXPECT_EQ(0.f,     wrap_360_cd_float(-72000.f));
}

AP_GTEST_MAIN()
