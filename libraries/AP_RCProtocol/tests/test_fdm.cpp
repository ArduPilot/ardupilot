/*
  Test RC input supplied by an external flight dynamics model.
 */

#include <AP_gtest.h>

#include <AP_RCProtocol/AP_RCProtocol_FDM.h>
#include <RC_Channel/RC_Channel.h>
#include <SITL/SITL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class RC_Channel_FDMTest : public RC_Channel {};

class RC_Channels_FDMTest : public RC_Channels {
public:
    RC_Channel_FDMTest obj_channels[NUM_RC_CHANNELS];

    const RC_Channel_FDMTest *channel(const uint8_t chan) const override
    {
        return chan < NUM_RC_CHANNELS ? &obj_channels[chan] : nullptr;
    }

    RC_Channel_FDMTest *channel(const uint8_t chan) override
    {
        return chan < NUM_RC_CHANNELS ? &obj_channels[chan] : nullptr;
    }

protected:
    int8_t flight_mode_channel_number() const override { return 5; }
};

#define RC_CHANNELS_SUBCLASS RC_Channels_FDMTest
#define RC_CHANNEL_SUBCLASS RC_Channel_FDMTest

#include <RC_Channel/RC_Channels_VarInfo.h>

static RC_Channels_FDMTest rchannels;

static void update_after_input_interval(AP_RCProtocol_FDM &backend)
{
    hal.scheduler->delay(21);
    backend.update();
}

TEST(AP_RCProtocolFDM, AppliesSITLRCFailureModes)
{
    static SITL::SIM sitl;
    static AP_RCProtocol frontend;
    static AP_RCProtocol_FDM backend(frontend);

    auto &fdm = sitl.state;
    fdm.rcin_chan_count = 4;
    fdm.rcin[0] = 0.25f;
    fdm.rcin[1] = 0.50f;
    fdm.rcin[2] = 0.75f;
    fdm.rcin[3] = 1.00f;

    sitl.rc_fail.set(SITL::SIM::SITL_RCFail_None);
    frontend.set_failsafe_active(false);
    update_after_input_interval(backend);
    EXPECT_EQ(backend.read(0), 1250);
    EXPECT_EQ(backend.read(1), 1500);
    EXPECT_EQ(backend.read(2), 1750);
    EXPECT_EQ(backend.read(3), 2000);
    EXPECT_FALSE(frontend.failsafe_active());

    const uint32_t frame_count = backend.get_rc_frame_count();
    sitl.rc_fail.set(SITL::SIM::SITL_RCFail_NoPulses);
    update_after_input_interval(backend);
    EXPECT_EQ(backend.get_rc_frame_count(), frame_count);

    sitl.rc_fail.set(SITL::SIM::SITL_RCFail_Throttle950);
    update_after_input_interval(backend);
    EXPECT_EQ(backend.get_rc_frame_count(), frame_count + 1);
    EXPECT_EQ(backend.read(0), 1500);
    EXPECT_EQ(backend.read(1), 1500);
    EXPECT_EQ(backend.read(2), 950);
    EXPECT_EQ(backend.read(3), 1500);
    EXPECT_FALSE(frontend.failsafe_active());

    sitl.rc_fail.set(SITL::SIM::SITL_RCFail_Protocol_Fail_Bit_Set);
    update_after_input_interval(backend);
    EXPECT_EQ(backend.get_rc_frame_count(), frame_count + 2);
    EXPECT_EQ(backend.read(0), 1456);
    EXPECT_EQ(backend.read(1), 1456);
    EXPECT_EQ(backend.read(2), 1456);
    EXPECT_EQ(backend.read(3), 1456);
    EXPECT_TRUE(frontend.failsafe_active());

    sitl.rc_fail.set(SITL::SIM::SITL_RCFail_None);
    update_after_input_interval(backend);
    EXPECT_EQ(backend.get_rc_frame_count(), frame_count + 3);
    EXPECT_EQ(backend.read(0), 1250);
    EXPECT_EQ(backend.read(1), 1500);
    EXPECT_EQ(backend.read(2), 1750);
    EXPECT_EQ(backend.read(3), 2000);
    EXPECT_FALSE(frontend.failsafe_active());
}

AP_GTEST_MAIN()
