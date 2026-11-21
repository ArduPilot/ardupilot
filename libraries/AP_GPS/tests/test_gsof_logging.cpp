#include <AP_gtest.h>

#include <AP_GPS/AP_GPS_GSOF.h>

#if AP_GPS_GSOF_ENABLED

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// packets given in ArduPilot issue 25517
TEST(AP_GPS_GSOF, logging_request_start)
{
    uint8_t buffer[AP_GPS_GSOF::LOGGING_REQUEST_LEN];
    AP_GPS_GSOF::build_logging_request(buffer, true);
    const uint8_t expected[] = {0x02, 0x00, 0x4C, 0x09, 0x08, 0x44, 0x45, 0x46,
                                0x41, 0x55, 0x4C, 0x54, 0x00, 0x62, 0x03};
    EXPECT_EQ(0, memcmp(buffer, expected, sizeof(expected)));
}

TEST(AP_GPS_GSOF, logging_request_stop)
{
    uint8_t buffer[AP_GPS_GSOF::LOGGING_REQUEST_LEN];
    AP_GPS_GSOF::build_logging_request(buffer, false);
    const uint8_t expected[] = {0x02, 0x00, 0x4C, 0x09, 0x09, 0x44, 0x45, 0x46,
                                0x41, 0x55, 0x4C, 0x54, 0x00, 0x63, 0x03};
    EXPECT_EQ(0, memcmp(buffer, expected, sizeof(expected)));
}

#endif  // AP_GPS_GSOF_ENABLED

AP_GTEST_MAIN()
