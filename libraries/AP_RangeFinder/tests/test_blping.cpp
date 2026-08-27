#include <AP_gtest.h>

#include <AP_RangeFinder/AP_RangeFinder_BLPing.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();


TEST(BLPing, completion_is_reported_once)
{
    uint8_t frame[] = {
        0x42, 0x52, 0x05, 0x00, 0xBB, 0x04, 0x01, 0x00,
        0xD0, 0x07, 0x00, 0x00, 100,
        0x00, 0x00,
    };
    uint16_t checksum = 0;
    for (uint8_t i = 0; i < sizeof(frame) - 2; i++) {
        checksum += frame[i];
    }
    frame[sizeof(frame) - 2] = checksum & 0xFF;
    frame[sizeof(frame) - 1] = checksum >> 8;

    PingProtocol protocol {};
    for (uint8_t i = 0; i < sizeof(frame) - 1; i++) {
        EXPECT_EQ(protocol.parse_byte(frame[i]), PingProtocol::MessageId::INVALID);
    }
    EXPECT_EQ(protocol.parse_byte(frame[sizeof(frame) - 1]),
              PingProtocol::MessageId::DISTANCE_SIMPLE);
    EXPECT_EQ(protocol.get_distance_mm(), 2000U);
    EXPECT_EQ(protocol.get_confidence(), 100U);

    EXPECT_EQ(protocol.parse_byte(0x00), PingProtocol::MessageId::INVALID);
    EXPECT_EQ(protocol.parse_byte(0xFF), PingProtocol::MessageId::INVALID);
}

AP_GTEST_MAIN()
