#define AP_PARAM_VEHICLE_NAME testvehicle

#include <AP_gtest.h>
#include <AP_Param/AP_Param.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(ParamProtection, MatchesConfiguredProtectedNames)
{
#if AP_PARAM_PROTECTION_ENABLED
    EXPECT_TRUE(AP_Param::is_protected("LOG_BITMASK"));
    EXPECT_TRUE(AP_Param::is_protected("PDE_SECRET_GAIN"));
    EXPECT_TRUE(AP_Param::is_protected("PDE_TUNE_ROLL"));
    EXPECT_TRUE(AP_Param::is_protected("PDE_TUNE_PITCH"));
    EXPECT_FALSE(AP_Param::is_protected("PDE_PUBLIC_GAIN"));
    EXPECT_FALSE(AP_Param::is_protected(""));
    EXPECT_FALSE(AP_Param::is_protected(nullptr));
#else
    EXPECT_FALSE(AP_Param::is_protected("LOG_BITMASK"));
    EXPECT_FALSE(AP_Param::is_protected("PDE_SECRET_GAIN"));
    EXPECT_FALSE(AP_Param::is_protected("PDE_TUNE_ROLL"));
    EXPECT_FALSE(AP_Param::is_protected("PDE_TUNE_PITCH"));
#endif
}

AP_GTEST_MAIN()
