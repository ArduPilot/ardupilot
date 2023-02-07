
// given we are in the Math library, you're epected to know what
// you're doing when directly comparing floats:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#define ALLOW_DOUBLE_MATH_FUNCTIONS
#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


TEST(MathTest, IsZeroDouble)
{
    EXPECT_FALSE(is_zero(0.1));
    EXPECT_FALSE(is_zero(0.0001));
    EXPECT_TRUE(is_zero(0.0));
    EXPECT_TRUE(is_zero(DBL_MIN));
    EXPECT_TRUE(is_zero(-DBL_MIN));
}

TEST(MathTest, MAXDouble)
{
    AP_Float t_float;
    t_float.set(0.1f);
    EXPECT_EQ(2.0, MAX(t_float, 2.0));
}
TEST(MathTest, IsEqualDouble)
{
    EXPECT_FALSE(is_equal(1.0, 0.0));
    EXPECT_TRUE(is_equal(1.0, 1.0));
    EXPECT_FALSE(is_equal(0.1,  0.10001));
    EXPECT_FALSE(is_equal(0.1, -0.1001));
    EXPECT_TRUE(is_equal(0.0,   0.0));
    EXPECT_FALSE(is_equal(1.0,  1.0 + DBL_EPSILON));
    EXPECT_TRUE(is_equal(1.0,  1.0 + DBL_EPSILON / 2.0));
    EXPECT_FALSE(is_equal((double)1.0, (double)(1.0 - DBL_EPSILON)));

    // false because the common type is double
    EXPECT_FALSE(is_equal(double(1.0), 1 + 2 * std::numeric_limits<double>::epsilon()));

    // true because the common type is float
    EXPECT_FALSE(is_equal((double)1.0, (double)(1.0 + std::numeric_limits<double>::epsilon())));
}

TEST(MathTest, ConstrainDouble)
{
    for (int i = 0; i < 1000; i++) {
        if (i < 250) {
            EXPECT_EQ(250.0, constrain_value((double)i, 250.0, 500.0));
        } else if (i > 500) {
            EXPECT_EQ(500.0, constrain_value((double)i, 250.0, 500.0));
        } else {
            EXPECT_EQ(i, constrain_value((double)i, 250.0, 500.0));
        }
    }

    for (int i = 0; i <= 1000; i++) {
        int c = i - 1000;
        if (c < -250) {
            EXPECT_EQ(-250.0, constrain_value((double)c, -250.0, -50.0));
        } else if(c > -50) {
            EXPECT_EQ(-50.0, constrain_value((double)c, -250.0, -50.0));
        } else {
            EXPECT_EQ(c, constrain_value((double)c, -250.0, -50.0));
        }
    }

    for (int i = 0; i <= 2000; i++) {
        int c = i - 1000;
        if (c < -250) {
            EXPECT_EQ(-250.0, constrain_value((double)c, -250.0, 50.0));
        } else if(c > 50) {
            EXPECT_EQ(50.0, constrain_value((double)c, -250.0, 50.0));
        } else {
            EXPECT_EQ(c, constrain_value((double)c, -250.0, 50.0));
        }
    }

    EXPECT_EQ(20.0, constrain_value(20.0, 19.9, 20.1));

    EXPECT_EQ(20.1, constrain_value(21.0, 19.9, 20.1));

    EXPECT_EQ(19.9, constrain_value(19.9, 19.9, 20.1));

    EXPECT_EQ(19.9, constrain_value(19.8, 19.9, 20.1));

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    EXPECT_EQ(1.0, constrain_value(nan("0x4152"), 1.0, 1.0));
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    EXPECT_EXIT(constrain_value(nan("0x4152"), 1.0, 1.0), testing::KilledBySignal(SIGABRT), "AP_InternalError::error_t::cnstring_nan");
#endif
}

TEST(MathWrapTest, Angle180Double)
{
    EXPECT_EQ(4500.0,   wrap_180_cd(4500.0));
    EXPECT_EQ(9000.0,   wrap_180_cd(9000.0));
    EXPECT_EQ(18000.0,  wrap_180_cd(18000.0));
    EXPECT_EQ(-17990.0, wrap_180_cd(18010.0));
    EXPECT_EQ(-9000.0,  wrap_180_cd(27000.0));
    EXPECT_EQ(0.0,      wrap_180_cd(36000.0));
    EXPECT_EQ(0.0,      wrap_180_cd(72000.0));
    EXPECT_EQ(0.0,      wrap_180_cd(360000.0));
    EXPECT_EQ(0.0,      wrap_180_cd(720000.0));
    EXPECT_EQ(0.0,      wrap_180_cd(-3600000000.0));

    EXPECT_EQ(-4500.0, wrap_180_cd(-4500.0));
    EXPECT_EQ(-9000.0, wrap_180_cd(-9000.0));
    EXPECT_EQ(18000.0, wrap_180_cd(-18000.0));
    EXPECT_EQ(17990.0, wrap_180_cd(-18010.0));
    EXPECT_EQ(9000.0,  wrap_180_cd(-27000.0));
    EXPECT_EQ(0.0,     wrap_180_cd(-36000.0));
    EXPECT_EQ(0.0,     wrap_180_cd(-72000.0));

    EXPECT_EQ(-45.0, wrap_180(-45.0));
    EXPECT_EQ(-90.0, wrap_180(-90.0));
    EXPECT_EQ(180.0, wrap_180(-180.0));
    EXPECT_EQ(90.0,  wrap_180(-270.0));
    EXPECT_EQ(0.0,     wrap_180(-360.0));
    EXPECT_EQ(0.0,     wrap_180(-720.0));
}

TEST(MathWrapTest, Angle360Double)
{
// Full circle test
    for (int32_t i = 0; i <= 36000; i += 100) {
        if (i == 0) {
            // hit pole position
            EXPECT_EQ(i, wrap_360_cd(static_cast<double>(i)));
            EXPECT_EQ(i, wrap_360_cd(static_cast<double>(-i)));
        } else if (i < 36000) {
            // between pole position
            EXPECT_EQ(i, wrap_360_cd(static_cast<double>(i)));
            EXPECT_EQ(36000.0 - i, wrap_360_cd(static_cast<double>(-i)));
        } else if (i == 36000) {
            // hit pole position
            EXPECT_EQ(0.0, wrap_360_cd(static_cast<double>(i)));
            EXPECT_EQ(0.0, wrap_360_cd(static_cast<double>(-i)));
        }
    }

    EXPECT_EQ(4500.0,  wrap_360_cd(static_cast<double>(4500.0)));
    EXPECT_EQ(9000.0,  wrap_360_cd(static_cast<double>(9000.0)));
    EXPECT_EQ(18000.0, wrap_360_cd(static_cast<double>(18000.0)));
    EXPECT_EQ(27000.0, wrap_360_cd(static_cast<double>(27000.0)));
    EXPECT_EQ(0.0,     wrap_360_cd(static_cast<double>(36000.0)));
    EXPECT_EQ(5.0,     wrap_360_cd(static_cast<double>(36005.0)));
    EXPECT_EQ(0.0,     wrap_360_cd(static_cast<double>(72000.0)));
    EXPECT_EQ(0.0,     wrap_360_cd(static_cast<double>(360000.0)));
    EXPECT_EQ(0.0,     wrap_360_cd(static_cast<double>(720000.0)));
    EXPECT_EQ( 0.0,    wrap_360_cd(static_cast<double>(-3600000000.0)));

    EXPECT_EQ(31500.0, wrap_360_cd(static_cast<double>(-4500.0)));
    EXPECT_EQ(27000.0, wrap_360_cd(static_cast<double>(-9000.0)));
    EXPECT_EQ(18000.0, wrap_360_cd(static_cast<double>(-18000.0)));
    EXPECT_EQ(9000.0,  wrap_360_cd(static_cast<double>(-27000.0)));
    EXPECT_EQ(0.0,     wrap_360_cd(static_cast<double>(-36000.0)));
    EXPECT_EQ(35995.0,wrap_360_cd(static_cast<double>(-36005.0)));
    EXPECT_EQ(0.0,     wrap_360_cd(static_cast<double>(-72000.0)));

    EXPECT_EQ(45.0,  wrap_360(static_cast<double>(45.0)));
    EXPECT_EQ(90.0,  wrap_360(static_cast<double>(90.0)));
    EXPECT_EQ(180.0, wrap_360(static_cast<double>(180.0)));
    EXPECT_EQ(270.0, wrap_360(static_cast<double>(270.0)));
    EXPECT_EQ(0.0,     wrap_360(static_cast<double>(360.0)));
    EXPECT_EQ(1.0,     wrap_360(static_cast<double>(361.0)));
    EXPECT_EQ(0.0,     wrap_360(static_cast<double>(720.0)));
    EXPECT_EQ(0.0,     wrap_360(static_cast<double>(3600.0)));
    EXPECT_EQ(0.0,     wrap_360(static_cast<double>(7200.0)));
    EXPECT_EQ(260.0,     wrap_360(static_cast<double>(-100.0)));
}

TEST(MathWrapTest, AnglePIDouble)
{
    const double accuracy = 1.0e-5;

    EXPECT_NEAR(M_PI,    wrap_PI((double)M_PI),      accuracy);
    EXPECT_NEAR(0.0,     wrap_PI((double)M_2PI),     accuracy);
    EXPECT_NEAR(0,       wrap_PI((double)M_PI * 10.0), accuracy);
    EXPECT_NEAR(-2.1415925025939941,    wrap_PI((double)M_PI+1.0),      accuracy);
}

TEST(MathWrapTest, Angle2PIDouble)
{
    const double accuracy = 1.0e-5;

    EXPECT_NEAR(M_PI, wrap_2PI((double)M_PI), accuracy);
    EXPECT_NEAR(0.0,  wrap_2PI((double)M_2PI), accuracy);
    EXPECT_NEAR(0.0,  wrap_2PI((double)M_PI * 10.0), accuracy);
    EXPECT_NEAR(0.0,  wrap_2PI(0.0), accuracy);
    EXPECT_NEAR(M_PI, wrap_2PI((double)-M_PI), accuracy);
    EXPECT_NEAR(0,    wrap_2PI((double)-M_2PI), accuracy);
}

TEST(MathTest, ASinDouble)
{
    const double accuracy = 1.0e-5;
    EXPECT_NEAR(0.0, safe_asin(double(0.0)), accuracy);

}

TEST(MathTest, SqrtDouble)
{
    const double accuracy = 1.0e-5;
    EXPECT_NEAR(0.0, safe_sqrt(double(0.0)), accuracy);

}

TEST(MathTest, SquareDouble)
{
    double sq_0 = sq(0);
    double sq_1 = sq(1);
    double sq_2 = sq(2);

    EXPECT_DOUBLE_EQ(0.0, sq_0);
    EXPECT_DOUBLE_EQ(1.0, sq_1);
    EXPECT_DOUBLE_EQ(4.0, sq_2);
}

TEST(MathTest, NormDouble)
{
    double norm_1 = norm(1.0, 4.2);
    double norm_2 = norm(1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1);
    double norm_3 = norm(0, 5.3);
    double norm_4 = norm(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
    double norm_5 = norm(3,4);
    double norm_6 = norm(4.0,3.0,12.0);

    EXPECT_DOUBLE_EQ(norm_1, 4.3174066289845809);
    EXPECT_DOUBLE_EQ(norm_2, 4.0);
    EXPECT_DOUBLE_EQ(norm_3, 5.2999999999999998);
    EXPECT_DOUBLE_EQ(norm_4, 0.0);
    EXPECT_DOUBLE_EQ(norm_5, 5.0);
    EXPECT_DOUBLE_EQ(norm_6, 13.0);
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()

#pragma GCC diagnostic pop
