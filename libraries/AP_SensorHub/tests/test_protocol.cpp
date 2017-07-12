#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_SensorHub/Protocol.h>

#if HAL_SENSORHUB_ENABLED
using namespace SensorHub;
const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class ProtocolTest : public testing::Test {
protected:
  uint8_t buf[BaroMessage::PACKET_LENGTH];
  BaroMessage msg;

  Packet::hdr_t e_hdr{Packet::MAGIC, Packet::VERSION,
          sizeof(BaroMessage::data_t), BaroMessage::ID, 0};
    BaroMessage::data_t e_data{BaroMessage::temperatureScaleToPacket(48.888), BaroMessage::pressureScaleToPacket(101325), 0};
  Packet::crc_t e_crc;

  uint8_t e_raw_offset_garbage[sizeof(e_hdr) + sizeof(e_data) + sizeof(e_crc) + 10]{};

  virtual void SetUp() {
    msg.setInstance(0);
    msg.setPressure(101325);
    msg.setTemperature(48.888);
    createPacket();
  }

  virtual void calcCrc() {
      e_crc = crc16_ccitt(reinterpret_cast<uint8_t *>(&e_hdr), sizeof(e_hdr), 0);
      e_crc = crc16_ccitt(reinterpret_cast<uint8_t *>(&e_data), sizeof(e_data), e_crc);
  }

  virtual void createPacket() {
      calcCrc();
      // Mimics receiver buffer with packet inside: HDR | DATA | CRC
      e_raw_offset_garbage[1] = Packet::MAGIC;
      memcpy(&e_raw_offset_garbage[3], &e_hdr, sizeof(e_hdr));
      memcpy(&e_raw_offset_garbage[sizeof(e_hdr) + 3], &e_data, sizeof(e_data));
      memcpy(&e_raw_offset_garbage[sizeof(e_hdr) + sizeof(e_data) + 3], &e_crc,
             sizeof(e_crc));
    }
};

/*
 * Test Protocol Encoding functionality.
 */
TEST_F(ProtocolTest, encode) {
    auto packet = msg.encode();

    EXPECT_TRUE(0 == memcmp(&packet->hdr, &e_hdr, sizeof(e_hdr)));
    EXPECT_TRUE(0 == memcmp(packet->data, reinterpret_cast<uint8_t *>(&e_data),
                            sizeof(e_data)));
    EXPECT_EQ(e_crc, packet->crc);
}

/*
 * Test Protocol Decoding functionality.
 */
TEST_F(ProtocolTest, decode) {

    Packet::packet_t a_packet;

    // Moves past garbage.
    auto decoded = static_cast<decode_t>(Packet::decode(&a_packet, &e_raw_offset_garbage[0],
                                                        sizeof(e_raw_offset_garbage)));

    EXPECT_TRUE(decoded == decode_t::SUCCESS);

    // Verify hdr & crc are expected.
    EXPECT_TRUE(0 == memcmp(&e_hdr, &a_packet.hdr, sizeof(e_hdr)));
    EXPECT_TRUE(0 == memcmp(&e_crc, &a_packet.crc, sizeof(e_crc)));

    auto isValid = Message::verify<BaroMessage>(&a_packet);
    EXPECT_TRUE(isValid);

    BaroMessage::data_t data {};
    Message::decode<BaroMessage>(&a_packet, &data);
    EXPECT_TRUE(0 == memcmp(&e_data, &data, sizeof(e_data)));
}

TEST_F(ProtocolTest, decode_no_magic) {
    uint8_t no_magic[BaroMessage::PACKET_LENGTH];
    memset(&no_magic[0], 0, sizeof(no_magic));

    Packet::packet_t a_packet;
    auto decoded = static_cast<decode_t>(Packet::decode(&a_packet, &no_magic[0], sizeof(no_magic)));
    EXPECT_TRUE(decoded == decode_t::FAIL_ADV);
}

TEST_F(ProtocolTest, decode_two_magic) {
    uint8_t two_magic[BaroMessage::PACKET_LENGTH];
    memset(&two_magic[0], 0, sizeof(two_magic));
    two_magic[1] = Packet::MAGIC;
    two_magic[5] = Packet::MAGIC;

    Packet::packet_t a_packet;
    auto decoded = static_cast<decode_t>(Packet::decode(&a_packet, &two_magic[0], sizeof(two_magic)));
    EXPECT_TRUE(decoded == decode_t::FAIL_ADV);
}

TEST_F(ProtocolTest, decode_no_remaining_bytes) {
    uint8_t no_remain[BaroMessage::PACKET_LENGTH];
    memset(&no_remain[0], 0, sizeof(no_remain));
    memcpy(&no_remain[BaroMessage::PACKET_LENGTH - Packet::EMPTY_PACKET_LEN - 2], &e_hdr, sizeof(e_hdr));
    Packet::packet_t a_packet;
    auto decoded = static_cast<decode_t>(Packet::decode(&a_packet, &no_remain[0], Packet::EMPTY_PACKET_LEN + 2));
    EXPECT_TRUE(decoded == decode_t::FAIL_CON);
}

TEST_F(ProtocolTest, decode_magic_end_buffer) {
    uint8_t magic_end[BaroMessage::PACKET_LENGTH];
    memset(&magic_end[0], 0, sizeof(magic_end) - 1);
    magic_end[sizeof(magic_end) - 1] = Packet::MAGIC;

    Packet::packet_t a_packet;
    auto decoded = static_cast<decode_t>(Packet::decode(&a_packet, &magic_end[0],
                                 sizeof(magic_end)));
    EXPECT_TRUE(decoded == decode_t::FAIL_CON);
}

TEST_F(ProtocolTest, decode_incomplete_packet) {
    uint8_t incomplete_packet[BaroMessage::PACKET_LENGTH];
    Packet::hdr_t inc_hdr{Packet::MAGIC, Packet::VERSION,
            sizeof(BaroMessage::data_t) + 3, BaroMessage::ID, 0};
    memcpy(&incomplete_packet[0], &inc_hdr, sizeof(inc_hdr));

    Packet::packet_t a_packet;
    auto decoded = static_cast<decode_t>(Packet::decode(&a_packet, &incomplete_packet[0],
                                                        sizeof(incomplete_packet)));

    EXPECT_TRUE(decoded == decode_t::FAIL_CON);
}
#endif

AP_GTEST_MAIN()