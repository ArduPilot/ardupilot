#include <AP_gtest.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS_Xsens_Parser.h>
#include <AP_HAL/AP_HAL.h>

#include <cmath>
#include <cstring>
#include <vector>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {
void add_field(std::vector<uint8_t> &payload, uint16_t id, const float *values, uint8_t count)
{
    payload.push_back(id >> 8);
    payload.push_back(id);
    payload.push_back(count * 4);
    for (uint8_t i = 0; i < count; i++) {
        uint32_t bits;
        memcpy(&bits, &values[i], sizeof(bits));
        payload.push_back(bits >> 24);
        payload.push_back(bits >> 16);
        payload.push_back(bits >> 8);
        payload.push_back(bits);
    }
}

std::vector<uint8_t> make_packet(uint16_t quaternion_id = 0x2014, bool add_gyro = true)
{
    std::vector<uint8_t> payload;
    const float quat[] = {1, 0, 0, 0};
    const float accel[] = {0.1f, -0.2f, 9.8f};
    const float gyro[] = {0.01f, 0.02f, -0.03f};
    add_field(payload, quaternion_id, quat, 4);
    add_field(payload, 0x4020, accel, 3);
    if (add_gyro) {
        add_field(payload, 0x8020, gyro, 3);
    }
    payload.insert(payload.end(), {0x10, 0x20, 0x02, 0x12, 0x34});
    std::vector<uint8_t> packet {0xFA, 0xFF, 0x36, uint8_t(payload.size())};
    for (uint8_t value : payload) {
        packet.push_back(value);
    }
    uint8_t sum = 0;
    for (size_t i = 1; i < packet.size(); i++) {
        sum += packet[i];
    }
    packet.push_back(uint8_t(-sum));
    return packet;
}

bool feed_packet(AP_ExternalAHRS_Xsens_Parser &parser, const std::vector<uint8_t> &packet,
                 AP_ExternalAHRS_Xsens_Parser::Sample &sample)
{
    bool received = false;
    for (uint8_t value : packet) {
        received |= parser.feed(value, sample);
    }
    return received;
}
}

TEST(XsensParser, CompleteNedSample)
{
    AP_ExternalAHRS_Xsens_Parser parser;
    AP_ExternalAHRS_Xsens_Parser::Sample sample {};
    EXPECT_TRUE(feed_packet(parser, make_packet(), sample));
    EXPECT_EQ(sample.counter, 0x1234);
    EXPECT_FLOAT_EQ(sample.accel_mps2[2], 9.8f);
    EXPECT_FLOAT_EQ(sample.gyro_rad_s[2], -0.03f);
    EXPECT_FLOAT_EQ(sample.quaternion_wxyz[0], 1.0f);
}

TEST(XsensParser, RejectsEnuMissingFieldAndChecksum)
{
    AP_ExternalAHRS_Xsens_Parser parser;
    AP_ExternalAHRS_Xsens_Parser::Sample sample {};
    EXPECT_FALSE(feed_packet(parser, make_packet(0x2010), sample));
    EXPECT_FALSE(feed_packet(parser, make_packet(0x2014, false), sample));
    auto corrupt = make_packet();
    corrupt[6] ^= 0x01;
    EXPECT_FALSE(feed_packet(parser, corrupt, sample));
    EXPECT_TRUE(feed_packet(parser, make_packet(), sample));
}

TEST(XsensParser, HandlesFragmentedStreamAndNoise)
{
    AP_ExternalAHRS_Xsens_Parser parser;
    AP_ExternalAHRS_Xsens_Parser::Sample sample {};
    EXPECT_FALSE(parser.feed(0x00, sample));
    EXPECT_FALSE(parser.feed(0xFA, sample));
    EXPECT_FALSE(parser.feed(0x00, sample));
    const auto packet = make_packet();
    for (size_t i = 0; i < packet.size() - 1; i++) {
        EXPECT_FALSE(parser.feed(packet[i], sample));
    }
    EXPECT_TRUE(parser.feed(packet.back(), sample));
}

TEST(XsensParser, DecodesOptionalMagnetometerField)
{
    AP_ExternalAHRS_Xsens_Parser parser;
    AP_ExternalAHRS_Xsens_Parser::Sample sample {};
    std::vector<uint8_t> payload;
    const float quat[] = {1, 0, 0, 0};
    const float accel[] = {0.0f, 0.0f, -9.81f};
    const float gyro[] = {0.0f, 0.0f, 0.0f};
    const float mag[] = {0.4f, -0.1f, 0.8f};
    add_field(payload, 0x2014, quat, 4);
    add_field(payload, 0x4020, accel, 3);
    add_field(payload, 0x8020, gyro, 3);
    add_field(payload, 0xC020, mag, 3);
    payload.insert(payload.end(), {0x10, 0x20, 0x02, 0x00, 0x01});
    std::vector<uint8_t> pkt {0xFA, 0xFF, 0x36, uint8_t(payload.size())};
    pkt.insert(pkt.end(), payload.begin(), payload.end());
    uint8_t sum = 0;
    for (size_t i = 1; i < pkt.size(); i++) {
        sum += pkt[i];
    }
    pkt.push_back(uint8_t(-sum));
    EXPECT_TRUE(feed_packet(parser, pkt, sample));
    EXPECT_TRUE(sample.have_mag);
    EXPECT_FLOAT_EQ(sample.mag_au[0], 0.4f);
    EXPECT_FLOAT_EQ(sample.mag_au[1], -0.1f);
    EXPECT_FLOAT_EQ(sample.mag_au[2], 0.8f);
}

AP_GTEST_MAIN()
