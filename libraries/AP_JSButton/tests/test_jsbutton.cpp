#include <AP_gtest.h>
#include <AP_JSButton/AP_JSButton.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(JSButtonTest, Test)
{
    JSButton jsbutton;
    EXPECT_EQ(jsbutton.function(), JSButton::k_none);
    EXPECT_EQ(jsbutton.function(true), JSButton::k_none);
    jsbutton.set_default(JSButton::k_shift, JSButton::k_arm_toggle);
    EXPECT_EQ(jsbutton.function(), JSButton::k_shift);
    EXPECT_EQ(jsbutton.function(true), JSButton::k_arm_toggle);
}

AP_GTEST_MAIN()
