/*
  test that GHST rx/tx activity detection survives the 32-bit micros() wrap
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

// feed one valid GHST_UL_RC_CHANS_HS4_5TO8 frame into the decoder
static void feed_rc_frame(AP_RCProtocol_GHST &ghst)
{
    // address, length, type, 10 bytes payload, crc
    uint8_t frame[14] {};
    frame[0] = AP_RCProtocol_GHST::GHST_ADDRESS_FLIGHT_CONTROLLER;
    frame[1] = 12;  // type + payload + crc
    frame[2] = AP_RCProtocol_GHST::GHST_UL_RC_CHANS_HS4_5TO8;
    // payload: four 12-bit channels at centre plus four low-res channels
    frame[3] = 0x00; frame[4] = 0x08; frame[5] = 0x80; frame[6] = 0x00; frame[7] = 0x08; frame[8] = 0x80;
    frame[9] = 128; frame[10] = 128; frame[11] = 128; frame[12] = 128;
    uint8_t crc = crc8_dvb_s2(0, frame[2]);
    for (uint8_t i = 3; i < 13; i++) {
        crc = crc8_dvb_s2(crc, frame[i]);
    }
    frame[13] = crc;
    for (uint8_t i = 0; i < sizeof(frame); i++) {
        ghst.process_byte(frame[i], GHST_BAUDRATE);
    }
}

// start of a 32-bit micros() period at least one full period ahead,
// so each scenario only moves the clock forward (the Linux HAL will
// not move it backwards) and does not depend on test order
static uint64_t next_micros_wrap()
{
    return ((AP_HAL::micros64() >> 32) + 2) << 32;
}

TEST(GHST, inactive_before_first_frame)
{
    AP_RCProtocol_GHST &ghst = *new AP_RCProtocol_GHST(frontend());

    // no frame has ever been received: must not report active, even
    // while micros() is still below the timeouts
    hal.scheduler->stop_clock(next_micros_wrap() + 1000);
    EXPECT_FALSE(ghst.is_rx_active());
    EXPECT_FALSE(ghst.is_tx_active());
    delete &ghst;
}

TEST(GHST, active_at_normal_time)
{
    AP_RCProtocol_GHST &ghst = *new AP_RCProtocol_GHST(frontend());

    const uint64_t t0 = next_micros_wrap() + 1000000;  // well clear of a wrap
    hal.scheduler->stop_clock(t0);
    feed_rc_frame(ghst);
    EXPECT_TRUE(ghst.is_rx_active());
    EXPECT_TRUE(ghst.is_tx_active());

    hal.scheduler->stop_clock(t0 + GHST_RX_TIMEOUT - 1000);
    EXPECT_TRUE(ghst.is_rx_active());

    hal.scheduler->stop_clock(t0 + GHST_RX_TIMEOUT + 1000);
    EXPECT_FALSE(ghst.is_rx_active());
    EXPECT_TRUE(ghst.is_tx_active());

    hal.scheduler->stop_clock(t0 + GHST_TX_TIMEOUT + 1000);
    EXPECT_FALSE(ghst.is_tx_active());
    delete &ghst;
}

TEST(GHST, active_across_micros_wrap)
{
    AP_RCProtocol_GHST &ghst = *new AP_RCProtocol_GHST(frontend());

    // last frame arrives 50ms before the 32-bit micros() counter wraps
    const uint64_t wrap = next_micros_wrap();
    const uint64_t t0 = wrap - 50000;
    hal.scheduler->stop_clock(t0);
    feed_rc_frame(ghst);
    EXPECT_TRUE(ghst.is_rx_active());
    EXPECT_TRUE(ghst.is_tx_active());

    // 20ms later, still before the wrap, well inside both timeouts
    hal.scheduler->stop_clock(t0 + 20000);
    EXPECT_TRUE(ghst.is_rx_active());
    EXPECT_TRUE(ghst.is_tx_active());

    // 20ms after the wrap, 70ms since the frame, still inside both timeouts
    hal.scheduler->stop_clock(wrap + 20000);
    EXPECT_TRUE(ghst.is_rx_active());
    EXPECT_TRUE(ghst.is_tx_active());

    // rx timeout expired, tx not yet
    hal.scheduler->stop_clock(t0 + GHST_RX_TIMEOUT + 1000);
    EXPECT_FALSE(ghst.is_rx_active());
    EXPECT_TRUE(ghst.is_tx_active());

    // both expired
    hal.scheduler->stop_clock(t0 + GHST_TX_TIMEOUT + 1000);
    EXPECT_FALSE(ghst.is_rx_active());
    EXPECT_FALSE(ghst.is_tx_active());
    delete &ghst;
}

AP_GTEST_MAIN()
