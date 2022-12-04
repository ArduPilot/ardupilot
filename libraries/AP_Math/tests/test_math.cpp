
// given we are in the Math library, you're epected to know what
// you're doing when directly comparing floats:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SQRT_2 1.4142135623730950488016887242097

TEST(VectorTest, Rotations)
{
    unsigned rotation_count = 0;

#define TEST_ROTATION(rotation, _x, _y, _z) { \
    const float accuracy = 1.0e-6; \
    Vector3f v(1, 1, 1); \
    Vector3f v2 = v; \
    v.rotate(rotation); \
    Vector3f expected(_x, _y, _z); \
    EXPECT_NEAR(expected.length(), v.length(), accuracy); \
    EXPECT_FLOAT_EQ(expected.x, v.x); \
    EXPECT_FLOAT_EQ(expected.y, v.y); \
    EXPECT_FLOAT_EQ(expected.z, v.z); \
    Quaternion quat; \
    quat.from_rotation(rotation); \
    quat.earth_to_body(v2); \
    EXPECT_NEAR(expected.length(), v.length(), accuracy); \
    EXPECT_NEAR(expected.x, v2.x, accuracy); \
    EXPECT_NEAR(expected.y, v2.y, accuracy); \
    EXPECT_NEAR(expected.z, v2.z, accuracy); \
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
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_68_YAW_293, -0.40663092252764576617352076937095, -1.5839677018260314156350432313047, -0.57069923113341980425161636958364);
    TEST_ROTATION(ROTATION_PITCH_315, 0, 1, SQRT_2);
    TEST_ROTATION(ROTATION_ROLL_90_PITCH_315, 0, -1, SQRT_2);
    TEST_ROTATION(ROTATION_PITCH_7, 1.1144154950464695286171945554088, 1, 0.87067680823617454866081288855639);
    TEST_ROTATION(ROTATION_ROLL_45, 1, 0, SQRT_2);
    TEST_ROTATION(ROTATION_ROLL_315, 1, SQRT_2, 0);
    EXPECT_EQ(ROTATION_MAX, rotation_count) << "All rotations are expect to be tested";

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    TEST_ROTATION(ROTATION_CUSTOM_OLD, 1, 1, 1);
    TEST_ROTATION(ROTATION_MAX, 1, 1, 1);
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    Vector3F v {1, 1, 1};
    EXPECT_EXIT(v.rotate(ROTATION_CUSTOM_OLD), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::bad_rotation");
    EXPECT_EXIT(v.rotate(ROTATION_MAX), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::bad_rotation");
#endif
}

TEST(MathTest, IsZero)
{
    EXPECT_FALSE(is_zero(0.1f));
    EXPECT_FALSE(is_zero(0.0001f));
    EXPECT_TRUE(is_zero(0.0f));
    EXPECT_TRUE(is_zero(FLT_MIN));
    EXPECT_TRUE(is_zero(-FLT_MIN));
    AP_Float t_float;
    t_float.set(0.1f);
    EXPECT_FALSE(is_zero(t_float));
    t_float.set(0.0f);
    EXPECT_TRUE(is_zero(t_float));
}

TEST(MathTest, IsPositive)
{
    EXPECT_TRUE(is_positive(1.0f));
    EXPECT_TRUE(is_positive(FLT_EPSILON));
    EXPECT_FALSE(is_positive(0.0f));
    EXPECT_FALSE(is_positive(-1.0f));
    AP_Float t_float;
    t_float.set(0.1f);
    EXPECT_TRUE(is_positive(t_float));
    t_float.set(-0.1f);
    EXPECT_FALSE(is_positive(t_float));
}

TEST(MathTest, IsNegative)
{
    EXPECT_TRUE(is_negative(-1.0f * FLT_EPSILON));
    EXPECT_TRUE(is_negative(-1.0f));
    EXPECT_FALSE(is_negative(0.0f));
    EXPECT_FALSE(is_negative(1.0f));
    AP_Float t_float;
    t_float.set(0.1f);
    EXPECT_FALSE(is_negative(t_float));
    t_float.set(-0.1f);
    EXPECT_TRUE(is_negative(t_float));
}

TEST(MathTest, MIN)
{
    const bool testb = true;
    EXPECT_EQ(1, MIN(testb, 2));
    const uint8_t testui8 = 1;
    EXPECT_EQ(1u, MIN(testui8, 2u));
    EXPECT_EQ(1u, MIN(testui8, testui8 + 2u));
    const signed char testcc1 = 1;
    const signed char testcc2 = 2;
    EXPECT_EQ(1, MIN(testcc1, 2));
    EXPECT_EQ(1, MIN(testcc1, testcc2));
    const unsigned char testc = 1;
    const unsigned char testc2 = 2;
    EXPECT_EQ(1u, MIN(testc, 2u));
    EXPECT_EQ(1u, MIN(testc, testc2));
    const unsigned int testuint = 1;
    EXPECT_EQ(1u, MIN(testuint, 2u));
    EXPECT_EQ(1u, MIN(testuint, testuint + 2u));
    const unsigned long testulong = 1;
    EXPECT_EQ(1u, MIN(testulong, 2u));
    EXPECT_EQ(1u, MIN(testulong, testulong + 2u));
    const unsigned short testushort = 1;
    const unsigned short testushort2 = 2;
    EXPECT_EQ(1u, MIN(testushort, 2u));
    EXPECT_EQ(1u, MIN(testushort, testushort2));
    EXPECT_EQ(1u, MIN(testc, testulong));
    EXPECT_EQ(1, MIN(2, testc));
    EXPECT_EQ(1u, MIN(testuint, 2.0f));
    EXPECT_EQ(1u, MIN(testuint, testulong));
    EXPECT_EQ(1u, MIN(testuint, testc));
    EXPECT_EQ(1u, MIN(testulong, testc));
    EXPECT_EQ(1u, MIN(testulong, testuint));
    EXPECT_EQ(1u, MIN(testulong, testushort));
    EXPECT_EQ(1, MIN(testushort, 1));
    EXPECT_EQ(1u, MIN(testushort, testulong));
    const int8_t testi8 = 1;
    EXPECT_EQ(1, MIN(testi8, 2));
    EXPECT_EQ(1, MIN(testui8, testi8 + 2));
    const uint16_t testui16 = 1;
    EXPECT_EQ(1u, MIN(testui16, 2u));
    EXPECT_EQ(1u, MIN(testui16, testui16 + 2u));
    const int16_t testi16 = 1;
    EXPECT_EQ(1, MIN(testi16, 2));
    EXPECT_EQ(1, MIN(testi16, testi16 + 2));
    const uint32_t testui32 = 1;
    EXPECT_EQ(1u, MIN(testui32, 2u));
    EXPECT_EQ(1u, MIN(testui32, testui32 + 2u));
    const int32_t testi32 = 1;
    EXPECT_EQ(1, MIN(testi32, 2));
    EXPECT_EQ(1, MIN(testi32, 2.0f));
    EXPECT_EQ(1.0f, MIN(1.0f, 2.0f));
    EXPECT_EQ(1.0f, MIN(1.0f, 2));
    EXPECT_EQ(1.0f, MIN(2.0f, 1.0f));
}

TEST(MathTest, MAX)
{
    const bool testb = true;
    EXPECT_EQ(2, MAX(testb, 2));
    const unsigned char testc = 1;
    const unsigned char testc2 = 2;
    EXPECT_EQ(2u, MAX(testc, 2u));
    EXPECT_EQ(2u, MAX(testc, testc2));
    const uint8_t testui8 = 1;
    EXPECT_EQ(2u, MAX(testui8, 2u));
    const int8_t testi8 = 1;
    EXPECT_EQ(2, MAX(testi8, 2));
    const uint16_t testui16 = 1;
    EXPECT_EQ(2u, MAX(testui16, 2u));
    const int16_t testi16 = 1;
    EXPECT_EQ(2, MAX(testi16, 2));
    const uint32_t testui32 = 1;
    EXPECT_EQ(2u, MAX(testui32, 2u));
    const int32_t testi32 = 1;
    EXPECT_EQ(2, MAX(testi32, 2));
    const unsigned long testulong = 1;
    EXPECT_EQ(2u, MAX(testulong, 2u));
    EXPECT_EQ(3u, MAX(testulong, testulong + 2u));
    const unsigned short testushort = 1;
    const unsigned short testushort2 = 2;
    EXPECT_EQ(2u, MAX(testushort, 2u));
    EXPECT_EQ(2u, MAX(testushort, testushort2));
    EXPECT_EQ(2.0f, MAX(testushort, 2.0f));
    EXPECT_EQ(2.0f, MAX(testi32, 2.0f));
    AP_Float t_float;
    t_float.set(0.1f);
    EXPECT_EQ(2.0f, MAX(t_float, 2.0f));
    EXPECT_EQ(2.0f, MAX(2.0f, t_float));
    AP_Int8 t_int8;
    t_int8.set(1);
    EXPECT_EQ(2, MAX(t_int8, 2));
    EXPECT_EQ(2, MAX(2, t_int8));
    AP_Int16 t_int16;
    t_int16.set(1);
    EXPECT_EQ(2, MAX(t_int16, 2));
    AP_Int32 t_int32;
    t_int32.set(1);
    EXPECT_EQ(2, MAX(t_int32, 2));
    EXPECT_EQ(2.0f, MAX(1.0f, 2.0f));
    EXPECT_EQ(2.0f, MAX(1.0f, 2));
    EXPECT_EQ(2.0f, MAX(2.0f, 1.0f));
}

TEST(MathTest, Convert)
{
    EXPECT_TRUE(1000000 == hz_to_nsec(1000u));
    EXPECT_TRUE(1000000 == nsec_to_hz(1000u));
    EXPECT_TRUE(1 == nsec_to_usec(1000u));
    EXPECT_TRUE(1000 == hz_to_usec(1000u));
    EXPECT_TRUE(1000 == usec_to_hz(1000u));
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

    EXPECT_TRUE(is_equal(short(1), short(1)));
    EXPECT_TRUE(is_equal(long(1), long(1)));
}

TEST(MathTest, Square)
{
    float sq_0 = sq(0);
    float sq_1 = sq(1);
    float sq_2 = sq(2);

    EXPECT_EQ(0.f, sq_0);
    EXPECT_EQ(1.f, sq_1);
    EXPECT_EQ(4.f, sq_2);
    AP_Float t_sqfloat;
    t_sqfloat.set(sq(2));
    EXPECT_EQ(4.f, t_sqfloat);

    EXPECT_FLOAT_EQ(sq(2.3), 5.289999999999999);  // uses template sq
    EXPECT_FLOAT_EQ(sq(2.3f), 5.29); // uses sq(float v)
    EXPECT_EQ(sq(4294967295), 18446744065119617025U);  // uses template sq
    EXPECT_FLOAT_EQ(sq(4294967295.0), 1.8446744e+19);  // uses template sq
    EXPECT_FLOAT_EQ(sq(pow(2,25)), pow(2,50));
}

TEST(MathTest, Norm)
{
    float norm_1 = norm(1, 4.2);
    float norm_2 = norm(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
    float norm_3 = norm(0, 5.3);
    float norm_4 = norm(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    float norm_5 = norm(3,4);
    float norm_6 = norm(4,3,12);
    AP_Float t_float1, t_float2, t_float3;
    t_float1.set(4.0f);
    t_float2.set(3.0f);
    t_float3.set(12.f);
    float norm_7 = norm(t_float1, t_float2, t_float3);

    EXPECT_FLOAT_EQ(norm_1, 4.3174066f);
    EXPECT_EQ(norm_2, 4.f);
    EXPECT_EQ(norm_3, 5.3f);
    EXPECT_EQ(norm_4, 0.f);
    EXPECT_EQ(norm_5, 5.f);
    EXPECT_EQ(norm_6, 13.f);
    EXPECT_EQ(norm_7, 13.f);
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
    EXPECT_EQ(20.0f, constrain_value(20.0f, 19.9f, 20.1f));
    EXPECT_EQ((long long int)20, constrain_value((long long int)20, (long long int)19, (long long int)20));

    EXPECT_EQ(20.1, constrain_value(21.0, 19.9, 20.1));
    EXPECT_EQ(20.1f, constrain_value(21.0f, 19.9f, 20.1f));

    EXPECT_EQ(19.9, constrain_value(19.9, 19.9, 20.1));
    EXPECT_EQ(19.9f, constrain_value(19.9f, 19.9f, 20.1f));

    EXPECT_EQ(19.9, constrain_value(19.8, 19.9, 20.1));
    EXPECT_EQ(19.9f, constrain_value(19.8f, 19.9f, 20.1f));

    // test that constrain on 32 bit integer works correctly. Note the asymmetry
    EXPECT_EQ(10,    constrain_int32( 0xFFFFFFFFU, 10U, 1200U));
    EXPECT_EQ(1200U, constrain_uint32(0xFFFFFFFFU, 10U, 1200U));

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    EXPECT_EQ(1.0f, constrain_float(nanf("0x4152"), 1.0f, 1.0f));
    EXPECT_EQ(1.0f, constrain_value(nanf("0x4152"), 1.0f, 1.0f));
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    EXPECT_EXIT(constrain_float(nanf("0x4152"), 1.0f, 1.0f), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::cnstring_nan");
    EXPECT_EXIT(constrain_value(nanf("0x4152"), 1.0f, 1.0f), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::cnstring_nan");
#endif
}

TEST(MathWrapTest, Angle180)
{
    // Full circle test
    for (int32_t i = 0; i < 36000; i += 100) {
        if (i < 18000) {
            // smaller pole position
            EXPECT_EQ(i, wrap_180_cd(i));
            EXPECT_EQ(-i, wrap_180_cd(-i));
            EXPECT_EQ(i, wrap_180_cd((long)i));
            EXPECT_EQ(-i, wrap_180_cd((long)-i));
        } else if (i == 18000) {
            // hit pole position -180/+180 degree
            EXPECT_EQ(i, wrap_180_cd(i));
            EXPECT_EQ(i, wrap_180_cd(-i));
            EXPECT_EQ(i, wrap_180_cd((long)i));
            EXPECT_EQ(i, wrap_180_cd((long)-i));
        } else {
            // bigger pole position
            EXPECT_EQ(-(36000 - i), wrap_180_cd(i));
            EXPECT_EQ(36000 - i, wrap_180_cd(-i));
            EXPECT_EQ(-(36000 - i), wrap_180_cd((long)i));
            EXPECT_EQ(36000 - i, wrap_180_cd((long)-i));
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

    EXPECT_EQ(4500, wrap_180_cd((short)4500));
    EXPECT_EQ(-4500, wrap_180_cd((short)-4500));

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
            EXPECT_EQ(i, wrap_360_cd(long(i)));
            EXPECT_EQ(i, wrap_360_cd(long(-i)));
        } else if (i < 36000) {
            // between pole position
            EXPECT_EQ(i, wrap_360_cd(i));
            EXPECT_EQ(36000 - i, wrap_360_cd(-i));
            EXPECT_EQ(i, wrap_360_cd(long(i)));
            EXPECT_EQ(36000 - i, wrap_360_cd(long(-i)));
        } else if (i == 36000) {
            // hit pole position
            EXPECT_EQ(0, wrap_360_cd(i));
            EXPECT_EQ(0, wrap_360_cd(-i));
            EXPECT_EQ(0, wrap_360_cd(long(i)));
            EXPECT_EQ(0, wrap_360_cd(long(-i)));
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


    EXPECT_EQ(45.0f,  wrap_360(45.0f));
    EXPECT_EQ(90.0f,  wrap_360(90.0f));
    EXPECT_EQ(180.0f, wrap_360(180.0f));
    EXPECT_EQ(270.0f, wrap_360(270.0f));
    EXPECT_EQ(0.0f,     wrap_360(360.0f));
    EXPECT_EQ(1.0f,     wrap_360(361.0f));
    EXPECT_EQ(0.0f,     wrap_360(720.0f));
    EXPECT_EQ(0.0f,     wrap_360(3600.0f));
    EXPECT_EQ(0.0f,     wrap_360(7200.0f));
    EXPECT_EQ(260.0f,     wrap_360(-100.0f));

    EXPECT_EQ(45,  wrap_360((int16_t)45));
    EXPECT_EQ(90,  wrap_360((int16_t)90));
    EXPECT_EQ(180, wrap_360((int16_t)180));
    EXPECT_EQ(270, wrap_360((int16_t)270));
    EXPECT_EQ(0,     wrap_360((int16_t)360));
    EXPECT_EQ(1,     wrap_360((int16_t)361));
    EXPECT_EQ(0,     wrap_360((int16_t)720));
    EXPECT_EQ(0,     wrap_360((int16_t)3600));
    EXPECT_EQ(0,     wrap_360((int16_t)7200));
    EXPECT_EQ(260,     wrap_360((int16_t)-100));

}

TEST(MathWrapTest, AnglePI)
{
    const float accuracy = 1.0e-5;

    EXPECT_NEAR(M_PI,    wrap_PI(M_PI),      accuracy);
    EXPECT_NEAR(0.f,     wrap_PI(M_2PI),     accuracy);
    EXPECT_NEAR(0,       wrap_PI(M_PI * 10), accuracy);
    EXPECT_NEAR(-2.1415925025939941f,    wrap_PI(M_PI+1),      accuracy);
    EXPECT_NEAR(1.f,     wrap_PI(1),     accuracy);
    EXPECT_NEAR(1.f,     wrap_PI((short)1),     accuracy);
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
    EXPECT_NEAR(1,    wrap_2PI(1), accuracy);
    EXPECT_NEAR(1,    wrap_2PI((short)1), accuracy);
}

TEST(MathTest, ASin)
{
    const float accuracy = 1.0e-5;
    EXPECT_NEAR(0.0f, safe_asin(0.0f), accuracy);
    EXPECT_NEAR(0.9033391107665127f, safe_asin(M_PI_2 * 0.5f), accuracy);
    EXPECT_NEAR(M_PI_2, safe_asin(M_PI_2 ), accuracy);
    EXPECT_NEAR(M_PI_2, safe_asin(M_PI), accuracy);
    EXPECT_NEAR(M_PI_2, safe_asin(M_2PI), accuracy);
    EXPECT_NEAR(-M_PI_2, safe_asin(-M_PI_2), accuracy);
    EXPECT_NEAR(-M_PI_2, safe_asin(-M_PI), accuracy);
    EXPECT_NEAR(-0.9033391107665127f, safe_asin(-M_PI_2 * 0.5f), accuracy);
    EXPECT_NEAR(0.0f, safe_asin(nanf("0x4152")), accuracy);
    EXPECT_NEAR(0.0f, safe_asin(int(0)), accuracy);
    EXPECT_NEAR(0.0f, safe_asin(short(0)), accuracy);
    EXPECT_NEAR(0.0f, safe_asin(double(0.0)), accuracy);

}

TEST(MathTest, Sqrt)
{
    const float accuracy = 1.0e-5;
    EXPECT_NEAR(0.0f, safe_sqrt(0.0f), accuracy);
    EXPECT_NEAR(32.0f, safe_sqrt(1024.0f), accuracy);
    EXPECT_NEAR(0.0f, safe_sqrt(-1.0f), accuracy);

    EXPECT_NEAR(0.0f, safe_sqrt(nanf("0x4152")), accuracy);
    EXPECT_NEAR(0.0f, safe_sqrt(int(0)), accuracy);
    EXPECT_NEAR(0.0f, safe_sqrt(short(0)), accuracy);
    EXPECT_NEAR(0.0f, safe_sqrt(double(0.0)), accuracy);

}

TEST(MathTest, Interpolation)
{
    const float accuracy = 1.0e-5;
    EXPECT_NEAR(1500.0f, linear_interpolate(1200.0f, 1800.0f, 1500.0f, 1000.0f, 2000.0f), accuracy);
    EXPECT_NEAR(1740.0f, linear_interpolate(1200.0f, 1800.0f, 1900.0f, 1000.0f, 2000.0f), accuracy);
    EXPECT_NEAR(1200.0f, linear_interpolate(1200.0f, 1800.0f, 1000.0f, 1000.0f, 2000.0f), accuracy);
    EXPECT_NEAR(1000.0f, linear_interpolate(1000.0f, 2000.0f, 1100.0f, 1200.0f, 1800.0f), accuracy);
    EXPECT_NEAR(2000.0f, linear_interpolate(1000.0f, 2000.0f, 1900.0f, 1200.0f, 1800.0f), accuracy);

}

TEST(MathTest, ThrottleCurve)
{
    // get hover throttle level [0,1]
    static constexpr float THR_MID = 0.5f;

    const float accuracy = 1.0e-5;
    EXPECT_NEAR(0.0f, throttle_curve(THR_MID, 0.0f, 0.0f), accuracy);
    EXPECT_NEAR(0.25f, throttle_curve(THR_MID, 0.0f, 0.25f), accuracy);
    EXPECT_NEAR(0.5f, throttle_curve(THR_MID, 0.0f, 0.5f), accuracy);
    EXPECT_NEAR(0.75f, throttle_curve(THR_MID, 0.0f, 0.75f), accuracy);
    EXPECT_NEAR(1.0f, throttle_curve(THR_MID, 0.0f, 1.0f), accuracy);
    EXPECT_NEAR(0.0f, throttle_curve(THR_MID, 0.25f, 0.0f), accuracy);
    EXPECT_NEAR(0.296875f, throttle_curve(THR_MID, 0.25f, 0.25f), accuracy);
    EXPECT_NEAR(0.5f, throttle_curve(THR_MID, 0.25f, 0.5f), accuracy);
    EXPECT_NEAR(0.703125f, throttle_curve(THR_MID, 0.25f, 0.75f), accuracy);
    EXPECT_NEAR(1.0f, throttle_curve(THR_MID, 0.25f, 1.0f), accuracy);
    EXPECT_NEAR(0.0f, throttle_curve(THR_MID, 0.5f, 0.0f), accuracy);
    EXPECT_NEAR(0.34375f, throttle_curve(THR_MID, 0.5f, 0.25f), accuracy);
    EXPECT_NEAR(0.5f, throttle_curve(THR_MID, 0.5f, 0.5f), accuracy);
    EXPECT_NEAR(0.65625f, throttle_curve(THR_MID, 0.5f, 0.75f), accuracy);
    EXPECT_NEAR(1.0f, throttle_curve(THR_MID, 0.5f, 1.0f), accuracy);
    EXPECT_NEAR(0.0f, throttle_curve(THR_MID, 0.75f, 0.0f), accuracy);
    EXPECT_NEAR(0.390625f, throttle_curve(THR_MID, 0.75f, 0.25f), accuracy);
    EXPECT_NEAR(0.5f, throttle_curve(THR_MID, 0.75f, 0.5f), accuracy);
    EXPECT_NEAR(0.609375f, throttle_curve(THR_MID, 0.75f, 0.75f), accuracy);
    EXPECT_NEAR(1.0f, throttle_curve(THR_MID, 0.75f, 1.0f), accuracy);
    EXPECT_NEAR(0.0f, throttle_curve(THR_MID, 1.0f, 0.0f), accuracy);
    EXPECT_NEAR(0.4375f, throttle_curve(THR_MID, 1.0f, 0.25f), accuracy);
    EXPECT_NEAR(0.5f, throttle_curve(THR_MID, 1.0f, 0.5f), accuracy);
    EXPECT_NEAR(0.5625f, throttle_curve(THR_MID, 1.0f, 0.75f), accuracy);
    EXPECT_NEAR(1.0f, throttle_curve(THR_MID, 1.0f, 1.0f), accuracy);
}

TEST(MathTest, RotationEqual)
{
    EXPECT_TRUE(rotation_equal(ROTATION_ROLL_180_YAW_90, ROTATION_ROLL_180_YAW_90));
    EXPECT_TRUE(rotation_equal(ROTATION_ROLL_180_YAW_90, ROTATION_PITCH_180_YAW_270));
    EXPECT_FALSE(rotation_equal(ROTATION_NONE, ROTATION_PITCH_180_YAW_270));
}

TEST(MathTest, FIXED2FLOAT)
{
    static constexpr uint16_t test_value = 1024;
    EXPECT_EQ(512.0f, fixed2float(test_value, 1));
    EXPECT_EQ(256.0f, fixed2float(test_value, 2));
}

TEST(MathTest, FLOAT2FIXED)
{
    static constexpr float test_value = 1024.0f;
    EXPECT_EQ(2048, float2fixed(test_value, 1));
    EXPECT_EQ(4096, float2fixed(test_value, 2));
}

TEST(MathTest, RANDOM16)
{
    static const uint16_t random_value = get_random16();
    EXPECT_NE(random_value, get_random16());
}

TEST(MathTest, VELCORRECTION)
{
    static constexpr Vector3F pos{1.0f, 1.0f, 0.0f};
    static constexpr Matrix3F rot(0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    static constexpr Vector3F rate{-1.0f, -1.0f, -1.0f};
    EXPECT_TRUE(Vector3F(1.0f, 1.0f, 0.0f) == get_vel_correction_for_sensor_offset(pos, rot, rate));
    EXPECT_TRUE(Vector3F() == get_vel_correction_for_sensor_offset(Vector3F(), rot, rate));
}

TEST(MathTest, LOWPASSALPHA)
{
    const float accuracy = 1.0e-5f;
    EXPECT_EQ(0.0f, calc_lowpass_alpha_dt(0.0f, 2.0f));
    EXPECT_EQ(1.0f, calc_lowpass_alpha_dt(1.0f, 0.0f));

    EXPECT_NEAR(0.926288f, calc_lowpass_alpha_dt(1.0f, 2.0f), accuracy);
}

TEST(MathTest, FIXEDWINGTURNRATE)
{
    const float accuracy = 1.0e-5f;

    EXPECT_NEAR(-318.65771484375f, fixedwing_turn_rate(-90, 10.0f), accuracy);
    EXPECT_NEAR(318.65771484375f, fixedwing_turn_rate(90, 10.0f), accuracy);
    EXPECT_NEAR(56.187965393066406f, fixedwing_turn_rate(45, 10.0f), accuracy);
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()

#pragma GCC diagnostic pop
