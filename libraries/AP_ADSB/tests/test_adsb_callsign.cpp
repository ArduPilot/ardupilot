#include <AP_gtest.h>

#include <AP_ADSB/AP_ADSB.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(IsValidCallsign, Valid)
{
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(7777));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(777));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(77));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(7));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(0));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(1111));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(111));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(11));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(1));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(0));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(7654));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(321));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(23));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(5));
    EXPECT_TRUE(AP_ADSB::is_valid_callsign(5));
}

TEST(IsValidCallsign, Invalid)
{
    EXPECT_FALSE(AP_ADSB::is_valid_callsign(17777));
    EXPECT_FALSE(AP_ADSB::is_valid_callsign(8888));
    EXPECT_FALSE(AP_ADSB::is_valid_callsign(888));
    EXPECT_FALSE(AP_ADSB::is_valid_callsign(88));
    EXPECT_FALSE(AP_ADSB::is_valid_callsign(8));
    EXPECT_FALSE(AP_ADSB::is_valid_callsign(9));

    EXPECT_FALSE(AP_ADSB::is_valid_callsign(7778));
    EXPECT_FALSE(AP_ADSB::is_valid_callsign(7788));
    EXPECT_FALSE(AP_ADSB::is_valid_callsign(7888));
    EXPECT_FALSE(AP_ADSB::is_valid_callsign(8888));
}

AP_GTEST_MAIN()

#pragma GCC diagnostic pop
