#include <AP_gtest.h>
#include <AP_Common/ExpandingString.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(ExpandingString, Tests)
{
    ExpandingString *test_string = new ExpandingString();
    test_string->printf("Test\n");
    EXPECT_STREQ("Test\n", test_string->get_string());
    EXPECT_STREQ("Test\n", test_string->get_writeable_string());
    EXPECT_EQ(5u, test_string->get_length());
    EXPECT_FALSE(test_string->has_failed_allocation());
    EXPECT_TRUE(test_string->append("Test2\n", 6));
}

AP_GTEST_MAIN()
