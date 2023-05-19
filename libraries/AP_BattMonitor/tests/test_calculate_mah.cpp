#include <AP_gtest.h>

#include <AP_BattMonitor/AP_BattMonitor_Backend.h>

static float calculate_mah_with_double_cast(float amps, float dt)
{
    return (float) ((double) amps * (double) dt * (double) 0.0000002778f);
}

static float calculate_mah(float amps, float dt)
{
    return AP_BattMonitor_Backend::calculate_mah(amps, dt);
}

TEST(AP_BATTMONITOR_MAH, test_calculate_mah)
{
    /* Basic unit tests to check for regressions */
    EXPECT_FLOAT_EQ(0.0002778, calculate_mah(1000, 1));
    EXPECT_FLOAT_EQ(2.778e-06, calculate_mah(1, 10));
    EXPECT_FLOAT_EQ(0.34296274, calculate_mah(1234567, 1));

    /* Test negative amperes */
    EXPECT_FLOAT_EQ(-0.02778, calculate_mah(-100, 1000));
    EXPECT_FLOAT_EQ(-34296.3, calculate_mah(-12345678, 10000));
    EXPECT_FLOAT_EQ(-2.778e-10, calculate_mah(-0.000001, 1000));
    EXPECT_FLOAT_EQ(-2.778e-07, calculate_mah(-1, 1));
}

TEST(AP_BATTMONITOR_ACCURACY, test_float_accuracy)
{
    /* Test for loss of accuracy */
    EXPECT_FLOAT_EQ(calculate_mah(100, 1), calculate_mah_with_double_cast(100, 1));
    EXPECT_FLOAT_EQ(calculate_mah(-1, 1), calculate_mah_with_double_cast(-1, 1));
    EXPECT_FLOAT_EQ(calculate_mah(0.0000000001f, 1), calculate_mah_with_double_cast(0.0000000001f, 1)); 
    EXPECT_FLOAT_EQ(calculate_mah(1234.123456789, 1), calculate_mah_with_double_cast(1234.123456789, 1));
    EXPECT_FLOAT_EQ(calculate_mah(-1234.123456789, 1), calculate_mah_with_double_cast(-1234.123456789, 1));
}

AP_GTEST_MAIN()
