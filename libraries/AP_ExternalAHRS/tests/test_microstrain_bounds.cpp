#include <AP_gtest.h>
#include <AP_ExternalAHRS/MicroStrain_common.h>

#if AP_MICROSTRAIN_ENABLED

namespace
{

class TestMicroStrain : public AP_MicroStrain
{
public:
    using AP_MicroStrain::MicroStrain_Packet;
    using AP_MicroStrain::handle_gnss;

    int32_t lat() const { return gnss_data[0].lat; }
};

// 1.0 encoded as a big-endian IEEE-754 double.
constexpr uint8_t DOUBLE_ONE[8] = {0x3F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void set_llh_lat(TestMicroStrain::MicroStrain_Packet &packet, const uint8_t field_offset)
{
    for (uint8_t i = 0; i < 8; i++) {
        packet.payload[field_offset + 2 + i] = DOUBLE_ONE[i];
    }
}

} // namespace

TEST(MicroStrainBounds, TruncatedLLHFieldIgnored)
{
    TestMicroStrain::MicroStrain_Packet packet{};
    packet.header[0] = 0x75;
    packet.header[1] = 0x65;
    packet.header[2] = 0x81; // GNSSData -> GNSS instance 0
    packet.header[3] = 18;   // payload_length
    packet.payload[0] = 16;  // field length: too short for LLH
    packet.payload[1] = 0x03; // LLH_POSITION
    packet.payload[16] = 2;  // terminator field, so the old loop still advances
    set_llh_lat(packet, 0);

    TestMicroStrain dut{};
    dut.handle_gnss(packet);

    EXPECT_EQ(0, dut.lat());
}

TEST(MicroStrainBounds, ValidLLHFieldParsed)
{
    TestMicroStrain::MicroStrain_Packet packet{};
    packet.header[0] = 0x75;
    packet.header[1] = 0x65;
    packet.header[2] = 0x81;
    packet.header[3] = 42;   // LLH field length
    packet.payload[0] = 42;
    packet.payload[1] = 0x03; // LLH_POSITION
    set_llh_lat(packet, 0);

    TestMicroStrain dut{};
    dut.handle_gnss(packet);

    EXPECT_EQ(10000000, dut.lat());
}

#endif // AP_MICROSTRAIN_ENABLED

AP_GTEST_MAIN()
