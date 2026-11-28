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
  test that CRSF parsed a block at a time decodes the same frames, in the
  same block, as CRSF parsed a byte at a time, through junk and frames
  split across blocks
 */
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// minimal vehicle singletons the decoder needs, as in RCProtocolTest
class RC_Channel_Test : public RC_Channel {};

class RC_Channels_Test : public RC_Channels
{
public:
    RC_Channel_Test obj_channels[NUM_RC_CHANNELS];

    const RC_Channel_Test *channel(const uint8_t chan) const override
    {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }
    RC_Channel_Test *channel(const uint8_t chan) override
    {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:
    int8_t flight_mode_channel_number() const override
    {
        return 5;
    }
};

#define RC_CHANNELS_SUBCLASS RC_Channels_Test
#define RC_CHANNEL_SUBCLASS RC_Channel_Test

#include <RC_Channel/RC_Channels_VarInfo.h>

static RC_Channels_Test rchannels;
static AP_SerialManager serial_manager;

// ArduPilot objects rely on zeroed allocation, so create them with new
static AP_RCProtocol &frontend()
{
    static AP_RCProtocol *rc = new AP_RCProtocol();
    return *rc;
}

static constexpr uint16_t STREAMS = 500;
static constexpr uint16_t STREAM_LEN_MAX = 2048;
static constexpr uint32_t BLOCK_INTERVAL_US = 1000;
// longer than CRSF's 50ms frame timeout, so a frame cut by it is dropped
static constexpr uint32_t LONG_GAP_US = 60000;
static constexpr uint8_t NUM_CHANNELS = 16;
// 98.0% of frames decode with this seed; the rest are cut by long gaps or
// swallowed by a false header. A resync that loses more than 1% more fails
static constexpr uint32_t MIN_DECODED_PERMILLE = 970;

// deterministic pseudo-random numbers, so a failure repeats
static uint32_t rng_state;
static uint32_t rng(uint32_t n)
{
    rng_state = rng_state * 1664525U + 1013904223U;
    return (rng_state >> 8) % n;
}

static void append(uint8_t *buf, uint16_t &len, uint8_t b)
{
    if (len < STREAM_LEN_MAX) {
        buf[len++] = b;
    }
}

// an RC frame with every channel at value, so the last frame decoded can be told apart
static void append_rc_frame(uint8_t *buf, uint16_t &len, uint16_t value)
{
    uint8_t frame[26] {};
    frame[0] = AP_CRSF_Protocol::CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame[1] = sizeof(frame) - 2;  // type, channels and CRC
    frame[2] = AP_CRSF_Protocol::CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        for (uint8_t bit = 0; bit < 11; bit++) {
            if (value & (1U << bit)) {
                const uint16_t pos = ch * 11 + bit;
                frame[3 + pos / 8] |= 1U << (pos % 8);
            }
        }
    }
    frame[sizeof(frame) - 1] = crc8_dvb_s2_update(0, &frame[2], sizeof(frame) - 3);
    for (uint8_t b : frame) {
        append(buf, len, b);
    }
}

// noise, stray sync bytes, impossible lengths and false headers
static void append_junk(uint8_t *buf, uint16_t &len)
{
    const uint8_t sync = AP_CRSF_Protocol::CRSF_ADDRESS_FLIGHT_CONTROLLER;
    switch (rng(5)) {
    case 0:
        for (uint32_t i = rng(8); i > 0; i--) {
            append(buf, len, rng(256));
        }
        break;
    case 1:
        append(buf, len, sync);
        break;
    case 2:
        append(buf, len, sync);
        append(buf, len, rng(2) ? 0xFF : 0x01);
        break;
    case 3:
        append(buf, len, sync);
        append(buf, len, sync);
        break;
    case 4:
        append(buf, len, sync);
        append(buf, len, 2 + rng(61));
        for (uint32_t i = rng(12); i > 0; i--) {
            append(buf, len, rng(4) ? rng(256) : sync);
        }
        break;
    }
}

struct Decoded {
    uint32_t frames;
    uint16_t channels[NUM_CHANNELS];
};

static Decoded decoded(AP_RCProtocol_CRSF &crsf)
{
    Decoded d {};
    d.frames = crsf.get_rc_frame_count();
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        d.channels[ch] = crsf.read(ch);
    }
    return d;
}

/*
  a byte is a block of one, so the parser fed a byte at a time takes the
  steps the byte-at-a-time parser took and is the reference here. Frames
  pass a CRC to decode, so the same count and the same channels after
  every block means the same frames were decoded in the same block
 */
TEST(CRSF, block_read_decodes_as_byte_read)
{
    static uint8_t stream[STREAM_LEN_MAX];
    uint32_t frames_sent = 0;
    uint32_t frames_decoded = 0;
    rng_state = 1;
    uint64_t now_us = 1000000;

    for (uint16_t s = 0; s < STREAMS; s++) {
        uint16_t len = 0;
        while (len < STREAM_LEN_MAX - 200) {
            if (rng(3) != 0) {
                append_junk(stream, len);
            }
            append_rc_frame(stream, len, 172 + rng(1640));
            frames_sent++;
        }

        // the same blocks, read a byte at a time and a block at a time
        AP_RCProtocol_CRSF &by_byte = *new AP_RCProtocol_CRSF(frontend());
        AP_RCProtocol_CRSF &by_block = *new AP_RCProtocol_CRSF(frontend());
        uint16_t ofs = 0;
        while (ofs < len) {
            const uint16_t n = MIN(uint16_t(1 + rng(64)), uint16_t(len - ofs));
            hal.scheduler->stop_clock(now_us);
            now_us += rng(50) == 0 ? LONG_GAP_US : BLOCK_INTERVAL_US;
            for (uint16_t i = 0; i < n; i++) {
                by_byte.process_byte(stream[ofs + i], CRSF_BAUDRATE);
            }
            by_block.process_bytes(&stream[ofs], n, CRSF_BAUDRATE);
            const Decoded byte_read = decoded(by_byte);
            const Decoded block_read = decoded(by_block);
            ASSERT_EQ(byte_read.frames, block_read.frames) << "stream " << s << " offset " << ofs;
            for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
                ASSERT_EQ(byte_read.channels[ch], block_read.channels[ch]) << "stream " << s << " offset " << ofs << " channel " << unsigned(ch);
            }
            ofs += n;
        }
        frames_decoded += decoded(by_block).frames;
        delete &by_byte;
        delete &by_block;
    }
    EXPECT_GE(uint64_t(frames_decoded) * 1000, uint64_t(frames_sent) * MIN_DECODED_PERMILLE);
}

AP_GTEST_MAIN()
