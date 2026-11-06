/*
  test that the GHST parser resynchronises after a byte is lost or
  added in a continuous stream of frames
 */
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <AP_RCProtocol/AP_RCProtocol_GHST.h>
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

// build a 14-byte GHST RC frame whose channel values change with n
static void make_frame(uint8_t frame[14], uint32_t n)
{
    frame[0] = AP_RCProtocol_GHST::GHST_ADDRESS_FLIGHT_CONTROLLER;
    frame[1] = 12;  // type + payload + crc
    frame[2] = AP_RCProtocol_GHST::GHST_UL_RC_CHANS_HS4_12_5TO8;
    const uint64_t ch = 1024 + (n * 37) % 2048;
    const uint64_t packed = ch | (ch << 12) | (ch << 24) | (ch << 36);
    for (uint8_t i = 0; i < 6; i++) {
        frame[3 + i] = (packed >> (8 * i)) & 0xFF;
    }
    for (uint8_t i = 9; i < 13; i++) {
        frame[i] = uint8_t(100 + (n * 7 + i) % 100);
    }
    uint8_t crc = crc8_dvb_s2(0, frame[2]);
    for (uint8_t i = 3; i < 13; i++) {
        crc = crc8_dvb_s2(crc, frame[i]);
    }
    frame[13] = crc;
}

enum class Glitch {
    DROP_BYTE,
    EXTRA_BYTE,
};

// stream frames every period_us, with a lost or added byte at
// glitch_offset in one frame, and return how many frames were
// decoded after the damaged one
static uint32_t frames_decoded_after_glitch(uint32_t period_us, Glitch glitch, uint8_t glitch_offset)
{
    AP_RCProtocol_GHST &ghst = *new AP_RCProtocol_GHST(frontend());
    const uint32_t byte_us = 24;  // 10 bits at 420 kbaud
    const uint32_t frames_before = 50;
    const uint32_t frames_after = 100;
    // start in a later 32-bit micros() period so the clock only moves
    // forward (the Linux HAL will not move it backwards)
    uint64_t t = (((AP_HAL::micros64() >> 32) + 2) << 32) + 1000000;
    uint32_t count_after_glitch = 0;
    for (uint32_t n = 0; n < frames_before + frames_after; n++) {
        uint8_t frame[14];
        make_frame(frame, n);
        uint64_t byte_t = t;
        for (uint8_t i = 0; i < sizeof(frame); i++) {
            if (n == frames_before && i == glitch_offset) {
                if (glitch == Glitch::DROP_BYTE) {
                    continue;
                }
                hal.scheduler->stop_clock(byte_t);
                ghst.process_byte(0x55, GHST_BAUDRATE);
                byte_t += byte_us;
            }
            hal.scheduler->stop_clock(byte_t);
            ghst.process_byte(frame[i], GHST_BAUDRATE);
            byte_t += byte_us;
        }
        if (n == frames_before) {
            count_after_glitch = ghst.get_rc_input_count();
        }
        t += period_us;
    }
    const uint32_t decoded = ghst.get_rc_input_count() - count_after_glitch;
    delete &ghst;
    return decoded;
}

// at link rates of 160Hz and above the gap between frames is shorter
// than the frame timeout, so the parser must find the next frame
// header itself rather than wait for a gap
TEST(GHST, resync_after_lost_or_added_byte)
{
    const uint32_t periods_us[] { 6250, 4000, 2000 };  // 160Hz, 250Hz, 500Hz
    const Glitch glitches[] { Glitch::DROP_BYTE, Glitch::EXTRA_BYTE };
    for (const uint32_t period_us : periods_us) {
        for (const Glitch glitch : glitches) {
            for (uint8_t ofs = 0; ofs < 14; ofs++) {
                // every frame after the damaged one should decode
                EXPECT_EQ(frames_decoded_after_glitch(period_us, glitch, ofs), 99U)
                        << "period " << period_us << "us, "
                        << (glitch == Glitch::DROP_BYTE ? "lost" : "added")
                        << " byte at offset " << unsigned(ofs);
            }
        }
    }
}

// a damaged frame can hold a byte sequence that looks like a short frame
// with a valid CRC over its type byte alone. Resynchronising on it must not
// publish channels from bytes that CRC does not cover
TEST(GHST, short_rc_frame_rejected)
{
    AP_RCProtocol_GHST &ghst = *new AP_RCProtocol_GHST(frontend());
    const uint32_t byte_us = 24;  // 10 bits at 420 kbaud
    uint64_t t = (((AP_HAL::micros64() >> 32) + 2) << 32) + 1000000;

    // frame A with "82 02 30 crc" in its low-res channels, then its own CRC
    // byte lost, then a good frame B
    uint8_t a[14];
    make_frame(a, 1);
    a[9] = AP_RCProtocol_GHST::GHST_ADDRESS_FLIGHT_CONTROLLER;
    a[10] = 2;  // type + crc
    a[11] = AP_RCProtocol_GHST::GHST_UL_RC_CHANS_HS4_12_5TO8;
    a[12] = crc8_dvb_s2(0, a[11]);
    uint8_t b[14];
    make_frame(b, 2);

    uint8_t stream[13 + 14];
    memcpy(stream, a, 13);
    memcpy(&stream[13], b, 14);
    for (uint8_t i = 0; i < sizeof(stream); i++) {
        hal.scheduler->stop_clock(t);
        ghst.process_byte(stream[i], GHST_BAUDRATE);
        t += byte_us;
    }

    // only frame B is decoded
    EXPECT_EQ(ghst.get_rc_input_count(), 1U);
    uint16_t pwm[4];
    ghst.read(pwm, 4);
    AP_RCProtocol_GHST &ref = *new AP_RCProtocol_GHST(frontend());
    for (uint8_t i = 0; i < sizeof(b); i++) {
        hal.scheduler->stop_clock(t);
        ref.process_byte(b[i], GHST_BAUDRATE);
        t += byte_us;
    }
    uint16_t ref_pwm[4];
    ref.read(ref_pwm, 4);
    for (uint8_t i = 0; i < 4; i++) {
        EXPECT_EQ(pwm[i], ref_pwm[i]) << "channel " << unsigned(i + 1);
    }
    delete &ref;
    delete &ghst;
}

AP_GTEST_MAIN()
