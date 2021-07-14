#include <AP_gtest.h>
#include <AP_Common/AP_FWVersion.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


TEST(AP_FWVersion, FWVersion)
{
    EXPECT_EQ(AP::fwversion().header, 0x61706677766572fbu);
    EXPECT_EQ(AP::fwversion().header_version, 0x0100U);
    EXPECT_EQ(AP::fwversion().pointer_size, static_cast<uint8_t>(sizeof(void*)));
    EXPECT_EQ(AP::fwversion().reserved, 0);
    EXPECT_EQ(AP::fwversion().vehicle_type, static_cast<uint8_t>(APM_BUILD_DIRECTORY));
    EXPECT_EQ(AP::fwversion().board_type, static_cast<uint8_t>(CONFIG_HAL_BOARD));
    EXPECT_EQ(AP::fwversion().board_subtype, static_cast<uint16_t>(CONFIG_HAL_BOARD_SUBTYPE));
    EXPECT_EQ(AP::fwversion().major, FW_MAJOR);
    EXPECT_EQ(AP::fwversion().minor, FW_MINOR);
    EXPECT_EQ(AP::fwversion().patch, FW_PATCH);
    EXPECT_EQ(AP::fwversion().fw_type, FW_TYPE);
    EXPECT_EQ(AP::fwversion().os_sw_version, 0u);
    EXPECT_STREQ(AP::fwversion().fw_string, THISFIRMWARE);
    EXPECT_STREQ(AP::fwversion().fw_hash_str, "");
    EXPECT_STREQ(AP::fwversion().fw_short_string, THISFIRMWARE);
    EXPECT_EQ(AP::fwversion().middleware_name, nullptr);
    EXPECT_EQ(AP::fwversion().middleware_hash_str, nullptr);
    EXPECT_EQ(AP::fwversion().os_name, nullptr);
    EXPECT_EQ(AP::fwversion().os_hash_str, nullptr);
}

AP_GTEST_MAIN()
