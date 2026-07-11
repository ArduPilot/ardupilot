// Tests for the GSOF parser.
// * ./waf tests
// * ./build/sitl/tests/test_gsof
// Or, with GDB
// * gdb ./build/sitl/tests/test_gsof

#include <AP_gtest.h>

#include <AP_GSOF/AP_GSOF.h>

#include <cstdio>
#include <cstdlib>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();


TEST(AP_GSOF, incomplete_packet)
{
    AP_GSOF gsof;
    AP_GSOF::MsgTypes expected;
    EXPECT_FALSE(gsof.parse(0, expected));
}

TEST(AP_GSOF, packet1)
{
    GTEST_SKIP() << "There is not yet a convention for loading in a data file in a cross-platform way in AP for unit tests";
    FILE* fp = std::fopen("libraries/AP_GSOF/tests/gsof_gps.dat", "rb");
    EXPECT_NE(fp, nullptr);
    AP_GSOF gsof;
    char c = 0;
    bool parsed = false;

    AP_GSOF::MsgTypes expected;
    expected.set(1);
    expected.set(2);
    expected.set(8);
    expected.set(9);
    expected.set(12);

    while (c != EOF) {
        c = fgetc (fp);
        parsed |= gsof.parse((uint8_t)c, expected);
    }
    
    EXPECT_TRUE(parsed);

    fclose(fp);

}

// Build a minimal DCOL/GSOF packet wrapping a single output record and feed it
// byte-by-byte to AP_GSOF::parse().  Returns true iff the parser signals a
// completed packet.
static bool feed_packet(AP_GSOF &gsof, AP_GSOF::MsgTypes &parsed,
                         const uint8_t *pkt, size_t len)
{
    bool got = false;
    for (size_t i = 0; i < len; i++) {
        if (gsof.parse(pkt[i], parsed) == AP_GSOF::PARSED_GSOF_DATA) {
            got = true;
        }
    }
    return got;
}

// Packet layout (DCOL framing):
//   STX | STATUS | PACKETTYPE(0x40) | LENGTH | DATA[LENGTH] | CHECKSUM | ENDTX
// DATA for a GSOF GENOUT packet:
//   [0..2]  transmission_number, page_index, max_page_index  (loop starts at 3)
//   [3]     output_type
//   [4]     output_length
//   [5..]   record data
// CHECKSUM = (STATUS + PACKETTYPE + LENGTH + sum(DATA)) & 0xFF

// VEL record (type 8) = 13 bytes:
//   velocity_flags (1) | horizontal_velocity BE float (4) |
//   heading BE float (4) | vertical_velocity BE float (4)

TEST(AP_GSOF, vel_flags_parsed)
{
    // velocity_flags = 0x05 (VELOCITY_VALID | HEADING_VALID)
    // horizontal_velocity = 1.5 m/s  -> IEEE-754 BE: 3F C0 00 00
    // heading            = 0.0 rad   -> IEEE-754 BE: 00 00 00 00
    // vertical_velocity  = -0.5 m/s  -> IEEE-754 BE: BF 00 00 00
    //
    // DATA (18 bytes):  01 01 01  08 0D  05  3F C0 00 00  00 00 00 00  BF 00 00 00
    // CHECKSUM = (0x00+0x40+0x12 + sum(DATA)) & 0xFF
    //          = (0x52 + 0x1DB) & 0xFF = 0x2D
    const uint8_t packet[] = {
        0x02,                                     // STX
        0x00,                                     // STATUS
        0x40,                                     // PACKETTYPE (GSOF)
        0x12,                                     // LENGTH = 18
        0x01, 0x01, 0x01,                         // GENOUT header
        0x08,                                     // output_type = VEL
        0x0D,                                     // output_length = 13
        0x05,                                     // velocity_flags
        0x3F, 0xC0, 0x00, 0x00,                   // horizontal_velocity = 1.5f
        0x00, 0x00, 0x00, 0x00,                   // heading = 0.0f
        0xBF, 0x00, 0x00, 0x00,                   // vertical_velocity = -0.5f
        0x2D,                                     // CHECKSUM
        0x03,                                     // ENDTX
    };

    AP_GSOF gsof;
    AP_GSOF::MsgTypes parsed;
    EXPECT_TRUE(feed_packet(gsof, parsed, packet, sizeof(packet)));
    EXPECT_TRUE(parsed.get(AP_GSOF::VEL));
    EXPECT_EQ(gsof.vel.velocity_flags, 0x05U);
    EXPECT_FLOAT_EQ(gsof.vel.horizontal_velocity, 1.5f);
    EXPECT_FLOAT_EQ(gsof.vel.heading, 0.0f);
    EXPECT_FLOAT_EQ(gsof.vel.vertical_velocity, -0.5f);
}

TEST(AP_GSOF, vel_flags_not_valid)
{
    // Same packet but velocity_flags = 0x00 (VELOCITY_VALID bit absent).
    // parse_vel() must not populate horizontal/vertical_velocity in that case.
    //
    // DATA same as above except byte [9] = 0x00 instead of 0x05.
    // CHECKSUM = (0x52 + 0x1D6) & 0xFF = 0x28
    const uint8_t packet[] = {
        0x02,
        0x00,
        0x40,
        0x12,
        0x01, 0x01, 0x01,
        0x08,
        0x0D,
        0x00,                                     // velocity_flags = 0 (not valid)
        0x3F, 0xC0, 0x00, 0x00,                   // velocity bytes present but ignored
        0x00, 0x00, 0x00, 0x00,
        0xBF, 0x00, 0x00, 0x00,
        0x28,                                     // CHECKSUM
        0x03,
    };

    AP_GSOF gsof;
    // Pre-load sentinel values so we can detect whether parse_vel() writes them.
    gsof.vel.horizontal_velocity = 42.0f;
    gsof.vel.vertical_velocity   = 42.0f;

    AP_GSOF::MsgTypes parsed;
    EXPECT_TRUE(feed_packet(gsof, parsed, packet, sizeof(packet)));
    EXPECT_TRUE(parsed.get(AP_GSOF::VEL));
    EXPECT_EQ(gsof.vel.velocity_flags, 0x00U);
    // parse_vel() must not overwrite velocity fields when the valid flag is absent.
    EXPECT_FLOAT_EQ(gsof.vel.horizontal_velocity, 42.0f);
    EXPECT_FLOAT_EQ(gsof.vel.vertical_velocity,   42.0f);
}

AP_GTEST_MAIN()
