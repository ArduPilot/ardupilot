
// given we are in the Math library, you're epected to know what
// you're doing when directly comparing floats:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

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
    TEST_ROTATION(ROTATION_PITCH_315, 0, 1, SQRT_2);
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_315, 0, -1, SQRT_2);
    TEST_ROTATION(ROTATION_PITCH_7, 1.1144155f, 1, 0.87067682f);

    EXPECT_EQ(ROTATION_MAX, rotation_count) << "All rotations are expect to be tested";
}

TEST(MathTest, IsZero)
{
    EXPECT_FALSE(is_zero(0.1));
    EXPECT_FALSE(is_zero(0.0001));
    EXPECT_TRUE(is_zero(0.f));
    EXPECT_TRUE(is_zero(FLT_MIN));
    EXPECT_TRUE(is_zero(-FLT_MIN));
}

TEST(MathTest, IsEqual)
{
    EXPECT_FALSE(is_equal(1, 0));
    EXPECT_TRUE(is_equal(1, 1));
    EXPECT_FALSE(is_equal(0.1,  0.10001));
    EXPECT_FALSE(is_equal(0.1, -0.1001));
    EXPECT_TRUE(is_equal(0.f,   0.0f));
    EXPECT_FALSE(is_equal(1.f,  1.f + FLT_EPSILON));
    EXPECT_TRUE(is_equal(1.f,  1.f + FLT_EPSILON / 2.f));
    EXPECT_TRUE(is_equal(1.f, (float)(1.f - DBL_EPSILON)));

    // false because the common type is double
    EXPECT_FALSE(is_equal(double(1.), 1 + 2 * std::numeric_limits<double>::epsilon()));

    // true because the common type is float
    EXPECT_TRUE(is_equal(1.f, (float)(1. + std::numeric_limits<double>::epsilon())));
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
    float norm_1 = norm(1, 4.2);
    float norm_2 = norm(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
    float norm_3 = norm(0, 5.3);
    float norm_4 = norm(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    float norm_5 = norm(3,4);
    float norm_6 = norm(4,3,12);

    EXPECT_FLOAT_EQ(norm_1, 4.3174066f);
    EXPECT_EQ(norm_2, 4.f);
    EXPECT_EQ(norm_3, 5.3f);
    EXPECT_EQ(norm_4, 0.f);
    EXPECT_EQ(norm_5, 5.f);
    EXPECT_EQ(norm_6, 13.f);
}


TEST(MathTest, Constrain)
{
    for (int i = 0; i < 1000; i++) {
        if (i < 250) {
            EXPECT_EQ(250, constrain_float(i, 250, 500));
            EXPECT_EQ(250, constrain_int16(i, 250, 500));
            EXPECT_EQ(250, constrain_int32(i, 250, 500));
            EXPECT_EQ(250, constrain_int64(i, 250, 500));
        } else if (i > 500) {
            EXPECT_EQ(500, constrain_float(i, 250, 500));
            EXPECT_EQ(500, constrain_int16(i, 250, 500));
            EXPECT_EQ(500, constrain_int32(i, 250, 500));
            EXPECT_EQ(500, constrain_int64(i, 250, 500));
        } else {
            EXPECT_EQ(i, constrain_float(i, 250, 500));
            EXPECT_EQ(i, constrain_int16(i, 250, 500));
            EXPECT_EQ(i, constrain_int32(i, 250, 500));
            EXPECT_EQ(i, constrain_int64(i, 250, 500));
        }
    }

    for (int i = 0; i <= 1000; i++) {
        int c = i - 1000;
        if (c < -250) {
            EXPECT_EQ(-250, constrain_float(c, -250, -50));
            EXPECT_EQ(-250, constrain_int16(c, -250, -50));
            EXPECT_EQ(-250, constrain_int32(c, -250, -50));
            EXPECT_EQ(-250, constrain_int64(c, -250, -50));
        } else if(c > -50) {
            EXPECT_EQ(-50, constrain_float(c, -250, -50));
            EXPECT_EQ(-50, constrain_int16(c, -250, -50));
            EXPECT_EQ(-50, constrain_int32(c, -250, -50));
            EXPECT_EQ(-50, constrain_int64(c, -250, -50));
        } else {
            EXPECT_EQ(c, constrain_float(c, -250, -50));
            EXPECT_EQ(c, constrain_int16(c, -250, -50));
            EXPECT_EQ(c, constrain_int32(c, -250, -50));
            EXPECT_EQ(c, constrain_int64(c, -250, -50));
        }
    }

    for (int i = 0; i <= 2000; i++) {
        int c = i - 1000;
        if (c < -250) {
            EXPECT_EQ(-250, constrain_float(c, -250, 50));
            EXPECT_EQ(-250, constrain_int16(c, -250, 50));
            EXPECT_EQ(-250, constrain_int32(c, -250, 50));
            EXPECT_EQ(-250, constrain_int64(c, -250, 50));
        } else if(c > 50) {
            EXPECT_EQ(50, constrain_float(c, -250, 50));
            EXPECT_EQ(50, constrain_int16(c, -250, 50));
            EXPECT_EQ(50, constrain_int32(c, -250, 50));
            EXPECT_EQ(50, constrain_int64(c, -250, 50));
        } else {
            EXPECT_EQ(c, constrain_float(c, -250, 50));
            EXPECT_EQ(c, constrain_int16(c, -250, 50));
            EXPECT_EQ(c, constrain_int32(c, -250, 50));
            EXPECT_EQ(c, constrain_int64(c, -250, 50));
        }
    }

    EXPECT_EQ(20.0, constrain_value(20.0, 19.9, 20.1));
    EXPECT_EQ(20.0, constrain_value(20.0f, 19.9f, 20.1f));

    EXPECT_EQ(19.9, constrain_value(19.9, 19.9, 20.1));
    EXPECT_EQ(19.9f, constrain_value(19.9f, 19.9f, 20.1f));

    EXPECT_EQ(19.9, constrain_value(19.8, 19.9, 20.1));
    EXPECT_EQ(19.9f, constrain_value(19.8f, 19.9f, 20.1f));
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

    EXPECT_EQ(4500.f,   wrap_180_cd(4500.f));
    EXPECT_EQ(9000.f,   wrap_180_cd(9000.f));
    EXPECT_EQ(18000.f,  wrap_180_cd(18000.f));
    EXPECT_EQ(-17990.f, wrap_180_cd(18010.f));
    EXPECT_EQ(-9000.f,  wrap_180_cd(27000.f));
    EXPECT_EQ(0.f,      wrap_180_cd(36000.f));
    EXPECT_EQ(0.f,      wrap_180_cd(72000.f));
    EXPECT_EQ(0.f,      wrap_180_cd(360000.f));
    EXPECT_EQ(0.f,      wrap_180_cd(720000.f));
    EXPECT_EQ(0.f,      wrap_180_cd(-3600000000.f));

    EXPECT_EQ(-4500.f, wrap_180_cd(-4500.f));
    EXPECT_EQ(-9000.f, wrap_180_cd(-9000.f));
    EXPECT_EQ(18000.f, wrap_180_cd(-18000.f));
    EXPECT_EQ(17990.f, wrap_180_cd(-18010.f));
    EXPECT_EQ(9000.f,  wrap_180_cd(-27000.f));
    EXPECT_EQ(0.f,     wrap_180_cd(-36000.f));
    EXPECT_EQ(0.f,     wrap_180_cd(-72000.f));

    EXPECT_EQ(45,   wrap_180(int16_t(45)));
    EXPECT_EQ(90,   wrap_180(int16_t(90)));
    EXPECT_EQ(180,  wrap_180(int16_t(180)));
    EXPECT_EQ(-179, wrap_180(int16_t(181)));
    EXPECT_EQ(-90,  wrap_180(int16_t(270)));
    EXPECT_EQ(0,      wrap_180(int16_t(360)));
    EXPECT_EQ(0,      wrap_180(int16_t(720)));
    EXPECT_EQ(0,      wrap_180(int16_t(3600)));
    EXPECT_EQ(0,      wrap_180((int16_t(7200))));
    EXPECT_EQ(0,      wrap_180((int16_t)-3600));
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

    EXPECT_EQ(4500.f,  wrap_360_cd(4500.f));
    EXPECT_EQ(9000.f,  wrap_360_cd(9000.f));
    EXPECT_EQ(18000.f, wrap_360_cd(18000.f));
    EXPECT_EQ(27000.f, wrap_360_cd(27000.f));
    EXPECT_EQ(0.f,     wrap_360_cd(36000.f));
    EXPECT_EQ(5.f,     wrap_360_cd(36005.f));
    EXPECT_EQ(0.f,     wrap_360_cd(72000.f));
    EXPECT_EQ(0.f,     wrap_360_cd(360000.f));
    EXPECT_EQ(0.f,     wrap_360_cd(720000.f));
    EXPECT_EQ( 0.f,    wrap_360_cd(-3600000000.f));

    EXPECT_EQ(31500.f, wrap_360_cd(-4500.f));
    EXPECT_EQ(27000.f, wrap_360_cd(-9000.f));
    EXPECT_EQ(18000.f, wrap_360_cd(-18000.f));
    EXPECT_EQ(9000.f,  wrap_360_cd(-27000.f));
    EXPECT_EQ(0.f,     wrap_360_cd(-36000.f));
    EXPECT_EQ(35995.0f,wrap_360_cd(-36005.f));
    EXPECT_EQ(0.f,     wrap_360_cd(-72000.f));


    EXPECT_EQ(45,  wrap_360((int16_t)45));
    EXPECT_EQ(90,  wrap_360((int16_t)90));
    EXPECT_EQ(180, wrap_360((int16_t)180));
    EXPECT_EQ(270, wrap_360((int16_t)270));
    EXPECT_EQ(0,     wrap_360((int16_t)360));
    EXPECT_EQ(1,     wrap_360((int16_t)361));
    EXPECT_EQ(0,     wrap_360((int16_t)720));
    EXPECT_EQ(0,     wrap_360((int16_t)3600));
    EXPECT_EQ(0,     wrap_360((int16_t)7200));
}

TEST(MathWrapTest, AnglePI)
{
    const float accuracy = 1.0e-5;

    EXPECT_NEAR(M_PI,    wrap_PI(M_PI),      accuracy);
    EXPECT_NEAR(0.f,     wrap_PI(M_2PI),     accuracy);
    EXPECT_NEAR(0,       wrap_PI(M_PI * 10), accuracy);
}

TEST(MathWrapTest, Angle2PI)
{
    const float accuracy = 1.0e-5;

    EXPECT_NEAR(M_PI, wrap_2PI(M_PI), accuracy);
    EXPECT_NEAR(0.f,  wrap_2PI(M_2PI), accuracy);
    EXPECT_NEAR(0.f,  wrap_2PI(M_PI * 10), accuracy);
    EXPECT_NEAR(0.f,  wrap_2PI(0.f), accuracy);
    EXPECT_NEAR(M_PI, wrap_2PI(-M_PI), accuracy);
    EXPECT_NEAR(0,    wrap_2PI(-M_2PI), accuracy);
}

AP_GTEST_MAIN()

#pragma GCC diagnostic pop
