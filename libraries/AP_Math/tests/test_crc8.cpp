#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(CRC8, crc8_dvb_buffer)
{
    const uint8_t buffer[2] { 0xBE, 0xEF };
    EXPECT_EQ(0x92, crc8_generic(buffer, 2, 0x31, 0xff));
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
