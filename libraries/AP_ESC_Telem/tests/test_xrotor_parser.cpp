/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  tests for the HobbyWing XRotor / T-Motor Alpha telemetry frame parser
 */

#include <AP_gtest.h>

#include <AP_ESC_Telem/AP_ESC_Telem_XRotor_Parser.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_ESC_TELEM_XROTOR_PARSER_ENABLED

// frames recorded from a T-Motor Alpha ESC during a bench motor test

// motor stopped
static const uint8_t idle_frame[] = {
    0x9b, 0x16, 0x01, 0x02, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xe7, 0x00, 0x00, 0x00, 0x00, 0xd8, 0xd9, 0x00, 0x00, 0x5f, 0x03,
};

// 30% throttle
static const uint8_t running_frame[] = {
    0x9b, 0x16, 0x01, 0x02, 0x01, 0xb2, 0x01, 0x59, 0x01, 0x59, 0x08, 0xa2,
    0x00, 0xe7, 0x00, 0x02, 0x00, 0x08, 0xd6, 0xd8, 0x00, 0x00, 0x64, 0x05,
};

// sent 549 ms after a lone 0x9b byte, as the motor started
static const uint8_t frame_after_lone_header[] = {
    0x9b, 0x16, 0x01, 0x02, 0x00, 0x85, 0x00, 0x9e, 0x00, 0x9e, 0x00, 0x00,
    0x00, 0xe7, 0x00, 0x00, 0x00, 0x11, 0xd7, 0xd9, 0x00, 0x00, 0x1d, 0x05,
};

// received with one bit flipped (0x00 to 0x10 in byte 8)
static const uint8_t corrupted_frame[] = {
    0x9b, 0x16, 0x01, 0x02, 0x01, 0x2c, 0x00, 0xfb, 0x10, 0xfb, 0x06, 0x2d,
    0x00, 0xe7, 0x00, 0x02, 0x00, 0x0e, 0xd6, 0xd8, 0x00, 0x00, 0xaf, 0x05,
};

// constructed: running_frame with header and length bytes inside its data
// (voltage 15.5 V, current 0x1602) and its checksum recomputed
static const uint8_t frame_with_header_in_data[] = {
    0x9b, 0x16, 0x01, 0x02, 0x01, 0xb2, 0x01, 0x59, 0x01, 0x59, 0x08, 0xa2,
    0x00, 0x9b, 0x16, 0x02, 0x00, 0x08, 0xd6, 0xd8, 0x00, 0x00, 0x2e, 0x05,
};

// running_frame with byte 7 lost
static const uint8_t frame_missing_a_byte[] = {
    0x9b, 0x16, 0x01, 0x02, 0x01, 0xb2, 0x01, 0x01, 0x59, 0x08, 0xa2, 0x00,
    0xe7, 0x00, 0x02, 0x00, 0x08, 0xd6, 0xd8, 0x00, 0x00, 0x64, 0x05,
};

// feed bytes to the parser, returning the number of good frames they completed
static uint8_t feed(AP_ESC_Telem_XRotor_Parser &parser, const uint8_t *data, uint8_t len)
{
    uint8_t frames = 0;
    for (uint8_t i = 0; i < len; i++) {
        if (parser.parse_byte(data[i])) {
            frames++;
        }
    }
    return frames;
}

TEST(XRotorParser, IdleFrame)
{
    AP_ESC_Telem_XRotor_Parser parser {};
    EXPECT_EQ(1, feed(parser, idle_frame, sizeof(idle_frame)));
    const AP_ESC_Telem_XRotor_Parser::Frame &f = parser.get_frame();
    EXPECT_EQ(0x01020013U, f.counter);
    EXPECT_EQ(0, f.throttle_req);
    EXPECT_EQ(0, f.rpm);
    EXPECT_EQ(231, f.voltage);
    EXPECT_EQ(0, f.current);
    EXPECT_EQ(22, AP_ESC_Telem_XRotor_Parser::temperature_decode(f.mos_temperature));
    EXPECT_EQ(21, AP_ESC_Telem_XRotor_Parser::temperature_decode(f.cap_temperature));
    EXPECT_EQ(0, f.status);
}

TEST(XRotorParser, RunningFrame)
{
    AP_ESC_Telem_XRotor_Parser parser {};
    EXPECT_EQ(1, feed(parser, running_frame, sizeof(running_frame)));
    const AP_ESC_Telem_XRotor_Parser::Frame &f = parser.get_frame();
    EXPECT_EQ(0x010201b2U, f.counter);
    EXPECT_EQ(345, f.throttle_req);
    EXPECT_EQ(345, f.throttle);
    EXPECT_EQ(2210, f.rpm);
    EXPECT_EQ(231, f.voltage);
    EXPECT_EQ(2, f.current);
    EXPECT_EQ(8, f.phase_current);
    EXPECT_EQ(23, AP_ESC_Telem_XRotor_Parser::temperature_decode(f.mos_temperature));
    EXPECT_EQ(22, AP_ESC_Telem_XRotor_Parser::temperature_decode(f.cap_temperature));
}

TEST(XRotorParser, CorruptedFrameRejected)
{
    AP_ESC_Telem_XRotor_Parser parser {};
    EXPECT_EQ(0, feed(parser, corrupted_frame, sizeof(corrupted_frame)));
    // the next good frame is still found
    EXPECT_EQ(1, feed(parser, idle_frame, sizeof(idle_frame)));
    EXPECT_EQ(0x01020013U, parser.get_frame().counter);
}

TEST(XRotorParser, LoneHeaderByte)
{
    AP_ESC_Telem_XRotor_Parser parser {};
    const uint8_t lone_header = 0x9b;
    EXPECT_EQ(0, feed(parser, &lone_header, 1));
    EXPECT_EQ(1, feed(parser, frame_after_lone_header, sizeof(frame_after_lone_header)));
    EXPECT_EQ(0x01020085U, parser.get_frame().counter);
}

TEST(XRotorParser, HeaderInsideData)
{
    AP_ESC_Telem_XRotor_Parser parser {};
    EXPECT_EQ(1, feed(parser, frame_with_header_in_data, sizeof(frame_with_header_in_data)));
    EXPECT_EQ(155, parser.get_frame().voltage);
    EXPECT_EQ(0x1602, parser.get_frame().current);
}

TEST(XRotorParser, LostByteCostsOneFrame)
{
    AP_ESC_Telem_XRotor_Parser parser {};
    EXPECT_EQ(0, feed(parser, frame_missing_a_byte, sizeof(frame_missing_a_byte)));
    EXPECT_EQ(1, feed(parser, idle_frame, sizeof(idle_frame)));
    EXPECT_EQ(0x01020013U, parser.get_frame().counter);
}

TEST(XRotorParser, BackToBackFrames)
{
    AP_ESC_Telem_XRotor_Parser parser {};
    EXPECT_EQ(1, feed(parser, idle_frame, sizeof(idle_frame)));
    EXPECT_EQ(1, feed(parser, running_frame, sizeof(running_frame)));
    EXPECT_EQ(2210, parser.get_frame().rpm);
}

TEST(XRotorParser, NoiseBeforeFrame)
{
    AP_ESC_Telem_XRotor_Parser parser {};
    // includes a header byte followed by the wrong length byte
    const uint8_t noise[] = { 0x00, 0xff, 0x9b, 0x17, 0x16, 0x55 };
    EXPECT_EQ(0, feed(parser, noise, sizeof(noise)));
    EXPECT_EQ(1, feed(parser, idle_frame, sizeof(idle_frame)));
}

TEST(XRotorParser, TemperatureDecode)
{
    EXPECT_EQ(0, AP_ESC_Telem_XRotor_Parser::temperature_decode(255));
    EXPECT_EQ(0, AP_ESC_Telem_XRotor_Parser::temperature_decode(241));
    EXPECT_EQ(25, AP_ESC_Telem_XRotor_Parser::temperature_decode(211));
    EXPECT_EQ(129, AP_ESC_Telem_XRotor_Parser::temperature_decode(33));
    EXPECT_EQ(130, AP_ESC_Telem_XRotor_Parser::temperature_decode(32));
}

#endif  // AP_ESC_TELEM_XROTOR_PARSER_ENABLED

AP_GTEST_MAIN()
