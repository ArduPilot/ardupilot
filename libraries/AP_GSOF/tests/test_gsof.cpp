// Tests for the GSOF parser.
// * ./waf tests
// * ./build/sitl/tests/test_gsof
// Or, with GDB
// * gdb ./build/sitl/tests/test_gsof

#include <AP_gtest.h>

#include <AP_GSOF/AP_GSOF.h>
#include <AP_Logger/AP_Logger.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if HAL_LOGGING_ENABLED
static AP_Logger logger;
#endif

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

TEST(AP_GSOF, position_time_fields)
{
    // GSOF1: TOW=0x01020304, week=2434, 15 satellites and valid RTK flags.
    const uint8_t packet[] = {
        0x02, 0x00, 0x40, 0x0F,
        0x01, 0x00, 0x00,
        0x01, 0x0A,
        0x01, 0x02, 0x03, 0x04,
        0x09, 0x82,
        0x0F, 0xAF, 0x07, 0x11,
        0xC6, 0x03,
    };

    AP_GSOF gsof;
    AP_GSOF::MsgTypes parsed;
    EXPECT_TRUE(feed_packet(gsof, parsed, packet, sizeof(packet)));
    EXPECT_TRUE(parsed.get(AP_GSOF::POS_TIME));
    EXPECT_EQ(gsof.pos_time.time_week_ms, 0x01020304U);
    EXPECT_EQ(gsof.pos_time.time_week, 2434U);
    EXPECT_EQ(gsof.pos_time.num_sats, 15U);
    EXPECT_EQ(gsof.pos_time.pos_flags1, 0xAFU);
    EXPECT_EQ(gsof.pos_time.pos_flags2, 0x07U);
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

// Build a DCOL/GSOF packet around the given DATA bytes, computing the checksum.
static size_t build_packet(uint8_t *out, const uint8_t *data, uint8_t data_len)
{
    size_t n = 0;
    out[n++] = 0x02;             // STX
    out[n++] = 0x00;             // STATUS
    out[n++] = 0x40;             // PACKETTYPE (GSOF)
    out[n++] = data_len;         // LENGTH
    uint8_t ck = 0x00 + 0x40 + data_len;
    for (uint8_t i = 0; i < data_len; i++) {
        out[n++] = data[i];
        ck += data[i];
    }
    out[n++] = ck;               // CHECKSUM
    out[n++] = 0x03;             // ENDTX
    return n;
}

// A record whose declared output_length runs past the end of DATA must be
// rejected rather than parsed. The record is placed at the tail of a
// maximum-size (255 byte) DATA area so that, without the bounds check,
// parse_pos() decodes 24 bytes starting at data[250]: it runs past the
// received bytes into the parser's own bookkeeping fields and, because the
// be64todouble_ptr(p, offset) helper takes a uint8_t offset, the altitude
// field is silently read from data[(250+16) & 0xFF] = data[10] instead.
TEST(AP_GSOF, record_longer_than_packet_rejected)
{
    uint8_t data[255];
    memset(data, 0x5A, sizeof(data));
    data[0] = data[1] = data[2] = 0x01;
    data[3] = 60;                // skipped record type
    data[4] = 243;               // filler occupies data[5..247]
    data[248] = 2;               // output_type = POS
    data[249] = 24;              // output_length claims 24 bytes...
    // ...but only data[250..254] (5 bytes) remain
    uint8_t packet[300];
    const size_t len = build_packet(packet, data, sizeof(data));

    AP_GSOF gsof;
    gsof.position.altitude = 1234.0;
    AP_GSOF::MsgTypes parsed;
    EXPECT_FALSE(feed_packet(gsof, parsed, packet, len));
    EXPECT_FALSE(parsed.get(AP_GSOF::POS));
    EXPECT_DOUBLE_EQ(gsof.position.altitude, 1234.0);
}

// A record that fits in DATA but is shorter than the fixed layout the parser
// reads must also be rejected (parse_pos reads 24 bytes).
TEST(AP_GSOF, record_shorter_than_layout_rejected)
{
    const uint8_t data[] = {
        0x01, 0x01, 0x01,
        2,                       // output_type = POS
        4,                       // output_length = 4 (layout needs 24)
        0x11, 0x22, 0x33, 0x44,
    };
    uint8_t packet[64];
    const size_t len = build_packet(packet, data, sizeof(data));

    AP_GSOF gsof;
    AP_GSOF::MsgTypes parsed;
    EXPECT_FALSE(feed_packet(gsof, parsed, packet, len));
    EXPECT_FALSE(parsed.get(AP_GSOF::POS));
}

// output_type as the very last DATA byte (no output_length byte at all).
TEST(AP_GSOF, trailing_output_type_without_length_rejected)
{
    const uint8_t data[] = {
        0x01, 0x01, 0x01,
        8,                       // output_type = VEL, then DATA ends
    };
    uint8_t packet[64];
    const size_t len = build_packet(packet, data, sizeof(data));

    AP_GSOF gsof;
    AP_GSOF::MsgTypes parsed;
    EXPECT_FALSE(feed_packet(gsof, parsed, packet, len));
    EXPECT_FALSE(parsed.get(AP_GSOF::VEL));
}

// Maximum-size DATA (255 bytes) with a well-formed VEL record still parses:
// the bounds check must not reject legitimate full-length packets.
TEST(AP_GSOF, full_length_packet_still_parses)
{
    uint8_t data[255];
    memset(data, 0, sizeof(data));
    data[0] = data[1] = data[2] = 0x01;
    data[3] = 8;                 // VEL
    data[4] = 13;
    data[5] = 0x05;              // velocity_flags: velocity + heading valid
    // remaining 12 bytes are zero floats; then pad the rest with a skipped
    // record type 60 whose output_length consumes the remainder exactly.
    data[18] = 60;
    data[19] = 255 - 20;         // 235 bytes of filler
    uint8_t packet[300];
    const size_t len = build_packet(packet, data, 255);

    AP_GSOF gsof;
    AP_GSOF::MsgTypes parsed;
    EXPECT_TRUE(feed_packet(gsof, parsed, packet, len));
    EXPECT_TRUE(parsed.get(AP_GSOF::VEL));
    EXPECT_EQ(gsof.vel.velocity_flags, 0x05U);
}

AP_GTEST_MAIN()
