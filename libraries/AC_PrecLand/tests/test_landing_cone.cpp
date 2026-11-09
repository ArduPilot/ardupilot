#include <AP_gtest.h>
#include <AC_PrecLand/AC_PrecLand.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AC_PrecLand precland;

class LandingCone : public ::testing::Test {
protected:
    void SetUp() override
    {
        AP_Param::setup_object_defaults(&precland, AC_PrecLand::var_info);
    }

    void set_parameter(const char *name, float value)
    {
        ASSERT_TRUE(AP_Param::set_object_value(&precland, AC_PrecLand::var_info, name, value));
    }
};

TEST_F(LandingCone, DisabledByDefault)
{
    EXPECT_TRUE(precland.is_within_landing_cone(100.0f, 0.0f));
}

TEST_F(LandingCone, DefaultAngle)
{
    set_parameter("CONE_ENABLE", 1);
    EXPECT_TRUE(precland.is_within_landing_cone(0.0f, 10.0f));
    EXPECT_TRUE(precland.is_within_landing_cone(2.67f, 10.0f));
    EXPECT_FALSE(precland.is_within_landing_cone(2.69f, 10.0f));
    EXPECT_TRUE(precland.is_within_landing_cone(0.267f, 1.0f));
    EXPECT_FALSE(precland.is_within_landing_cone(0.269f, 1.0f));
}

TEST_F(LandingCone, ConfigurableAngleAndBoundary)
{
    set_parameter("CONE_ENABLE", 1);
    set_parameter("CONE_ANGLE", 45);
    const float boundary_m = 2.0f * tanf(radians(45.0f));
    EXPECT_TRUE(precland.is_within_landing_cone(boundary_m, 2.0f));
    EXPECT_FALSE(precland.is_within_landing_cone(boundary_m + 0.01f, 2.0f));
    set_parameter("CONE_ANGLE", 15);
    EXPECT_FALSE(precland.is_within_landing_cone(1.0f, 2.0f));
}

TEST_F(LandingCone, ActiveToTouchdown)
{
    set_parameter("CONE_ENABLE", 1);
    EXPECT_TRUE(precland.is_within_landing_cone(0.0f, 0.0f));
    EXPECT_FALSE(precland.is_within_landing_cone(0.001f, 0.0f));
    EXPECT_FALSE(precland.is_within_landing_cone(0.001f, -1.0f));
    EXPECT_TRUE(precland.is_within_landing_cone(0.02f, 0.1f));
    EXPECT_FALSE(precland.is_within_landing_cone(0.03f, 0.1f));
}

TEST_F(LandingCone, AngleIsConstrained)
{
    set_parameter("CONE_ENABLE", 1);
    set_parameter("CONE_ANGLE", 0);
    EXPECT_TRUE(precland.is_within_landing_cone(0.017f, 1.0f));
    EXPECT_FALSE(precland.is_within_landing_cone(0.018f, 1.0f));
    set_parameter("CONE_ANGLE", 90);
    EXPECT_TRUE(precland.is_within_landing_cone(11.4f, 1.0f));
    EXPECT_FALSE(precland.is_within_landing_cone(11.5f, 1.0f));
}

AP_GTEST_MAIN()
