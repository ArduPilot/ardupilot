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
#include <AP_gtest.h>

#include <string.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/dsm.h>

/*
  dsm_decode keeps static state for its 10/11-bit format guessing; a
  gap of more than one second between frames resets that state.  Each
  test uses advance_time() to get a fresh decoder state.
 */
static uint64_t frame_time_us;

static void advance_time(void)
{
    frame_time_us += 2000000;
}

// number of consecutive frames the format-guesser must see before it
// locks on (5 samples, then decoding starts on the next frame)
static const uint8_t GUESS_FRAMES = 6;

// build a frame of 7 channels in 11-bit format; channel numbers
// 0..6, all with the given raw value
static void build_11bit_frame(uint8_t frame[16], const uint16_t raw_value)
{
    frame[0] = 0;
    frame[1] = 0;
    for (uint8_t ch=0; ch<7; ch++) {
        const uint16_t raw = (ch << 11) | (raw_value & 0x7ff);
        frame[2 + 2*ch] = raw >> 8;
        frame[2 + 2*ch + 1] = raw & 0xff;
    }
}

// build a frame of 7 channels in 10-bit format
static void build_10bit_frame(uint8_t frame[16], const uint16_t raw_value)
{
    frame[0] = 0;
    frame[1] = 0;
    for (uint8_t ch=0; ch<7; ch++) {
        const uint16_t raw = (ch << 10) | (raw_value & 0x3ff);
        frame[2 + 2*ch] = raw >> 8;
        frame[2 + 2*ch + 1] = raw & 0xff;
    }
}

// feed the same frame until the format guesser locks on, then decode
static bool train_and_decode(const uint8_t frame[16],
                             uint16_t *values,
                             uint16_t *num_values,
                             uint16_t max_values)
{
    bool ret = false;
    for (uint8_t i=0; i<GUESS_FRAMES+1; i++) {
        frame_time_us += 11000;
        ret = dsm_decode(frame_time_us, frame, values, num_values, max_values);
    }
    return ret;
}

// the DSM value scaling used by dsm_decode for 11-bit data
static uint16_t expected_value_11bit(uint16_t raw)
{
    return ((((int)raw - 1024) * 1000) / 1700) + 1500;
}

TEST(DSMTest, Decode11Bit)
{
    advance_time();

    uint8_t frame[16];
    const uint16_t raw = 1024;  // mid-point
    build_11bit_frame(frame, raw);

    uint16_t values[16] {};
    uint16_t num_values = 0;

    EXPECT_TRUE(train_and_decode(frame, values, &num_values, ARRAY_SIZE(values)));
    EXPECT_EQ(num_values, 7);

    // 1024 scales to exactly the centered mid-point
    for (uint8_t ch=0; ch<7; ch++) {
        EXPECT_EQ(values[ch], 1500) << "channel " << (unsigned)ch;
    }
}

TEST(DSMTest, GuessingReturnsFalse)
{
    advance_time();

    uint8_t frame[16];
    build_11bit_frame(frame, 1024);

    uint16_t values[16] {};
    uint16_t num_values = 0;

    // while the format guesser has not locked on, no frame decodes
    for (uint8_t i=0; i<GUESS_FRAMES; i++) {
        frame_time_us += 11000;
        EXPECT_FALSE(dsm_decode(frame_time_us, frame, values, &num_values, ARRAY_SIZE(values)));
    }
    // ... and the next frame does
    frame_time_us += 11000;
    EXPECT_TRUE(dsm_decode(frame_time_us, frame, values, &num_values, ARRAY_SIZE(values)));
}

TEST(DSMTest, ChannelMapping)
{
    advance_time();

    // give each channel a distinct value so the AETR reordering is visible
    uint8_t frame[16];
    frame[0] = 0;
    frame[1] = 0;
    const uint16_t raws[7] { 1024, 1124, 1224, 1324, 1424, 1524, 1624 };
    for (uint8_t ch=0; ch<7; ch++) {
        const uint16_t raw = (ch << 11) | raws[ch];
        frame[2 + 2*ch] = raw >> 8;
        frame[2 + 2*ch + 1] = raw & 0xff;
    }

    uint16_t values[16] {};
    uint16_t num_values = 0;
    EXPECT_TRUE(train_and_decode(frame, values, &num_values, ARRAY_SIZE(values)));
    EXPECT_EQ(num_values, 7);

    // DSM channel order is TAER; ArduPilot buffer order is AETR:
    // DSM channel 0 (throttle) lands in slot 2, 1 in slot 0, 2 in slot 1
    EXPECT_EQ(values[2], expected_value_11bit(raws[0]));
    EXPECT_EQ(values[0], expected_value_11bit(raws[1]));
    EXPECT_EQ(values[1], expected_value_11bit(raws[2]));
    // channels 3 and up are not remapped
    for (uint8_t ch=3; ch<7; ch++) {
        EXPECT_EQ(values[ch], expected_value_11bit(raws[ch])) << "channel " << (unsigned)ch;
    }
}

TEST(DSMTest, Decode10Bit)
{
    advance_time();

    uint8_t frame[16];
    const uint16_t raw = 512;  // 10-bit mid-point
    build_10bit_frame(frame, raw);

    uint16_t values[16] {};
    uint16_t num_values = 0;

    EXPECT_TRUE(train_and_decode(frame, values, &num_values, ARRAY_SIZE(values)));
    EXPECT_EQ(num_values, 7);

    // 10-bit values are doubled before scaling, so 512 -> 1024 -> 1500
    for (uint8_t ch=0; ch<7; ch++) {
        EXPECT_EQ(values[ch], 1500) << "channel " << (unsigned)ch;
    }
}

TEST(DSMTest, ChannelsBeyondMaxValuesIgnored)
{
    advance_time();

    uint8_t frame[16];
    build_11bit_frame(frame, 1024);

    // only 4 slots available; channels 4..6 must be dropped rather
    // than written out-of-bounds
    uint16_t values[4] {};
    uint16_t num_values = 0;

    EXPECT_TRUE(train_and_decode(frame, values, &num_values, ARRAY_SIZE(values)));
    EXPECT_EQ(num_values, 4);
    for (uint8_t ch=0; ch<4; ch++) {
        EXPECT_EQ(values[ch], 1500) << "channel " << (unsigned)ch;
    }
}

TEST(DSMTest, AllOnesChannelsSkipped)
{
    advance_time();

    // 0xffff raw channel data must not decode
    uint8_t frame[16];
    memset(frame, 0xff, sizeof(frame));

    uint16_t values[16] {};
    uint16_t num_values = 0;

    // the format guesser can never lock onto all-0xffff frames, so
    // decoding never succeeds
    for (uint8_t i=0; i<GUESS_FRAMES*3; i++) {
        frame_time_us += 11000;
        EXPECT_FALSE(dsm_decode(frame_time_us, frame, values, &num_values, ARRAY_SIZE(values)));
    }
    EXPECT_EQ(num_values, 0);
}

TEST(DSMTest, SignalLossResetsFormatGuess)
{
    advance_time();

    uint8_t frame[16];
    build_11bit_frame(frame, 1024);

    uint16_t values[16] {};
    uint16_t num_values = 0;
    EXPECT_TRUE(train_and_decode(frame, values, &num_values, ARRAY_SIZE(values)));

    // more than 1s of signal loss must reset the format guess, so
    // the next frame goes back to training
    advance_time();
    frame_time_us += 11000;
    EXPECT_FALSE(dsm_decode(frame_time_us, frame, values, &num_values, ARRAY_SIZE(values)));
}

AP_GTEST_MAIN()
