/*
  test that SBUS failsafe frames do not overwrite the last accepted
  channel values
 */
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SBusOut/AP_SBusOut.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <AP_RCProtocol/AP_RCProtocol_SBUS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// minimal vehicle singletons the decoder needs, as in GHST.inactive_before_first_frame
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

// these mirror the private definitions in AP_RCProtocol_SBUS.cpp
static constexpr uint32_t SBUS_BAUD = 100000;
static constexpr uint32_t SBUS_FRAME_GAP_US = 2000;
static constexpr uint8_t SBUS_FLAGS_BYTE = 23;
static constexpr uint8_t SBUS_FRAMELOST_BIT = 2;
static constexpr uint8_t SBUS_FAILSAFE_BIT = 3;

static constexpr uint8_t NUM_CHANNELS = 16;

class SBUSFailsafe : public ::testing::Test
{
protected:
    void SetUp() override
    {
        set_ignore_failsafe(false);
        sbus = new AP_RCProtocol_SBUS(frontend(), false, SBUS_BAUD);
        // each test moves the clock forward from where the last stopped
        now_us = AP_HAL::micros64() + 1000000;
        hal.scheduler->stop_clock(now_us);
    }

    void TearDown() override
    {
        delete sbus;
        set_ignore_failsafe(false);
    }

    static void set_ignore_failsafe(bool ignore)
    {
        const uint32_t bits = ignore ? uint32_t(RC_Channels::Option::IGNORE_FAILSAFE) : 0;
        ASSERT_TRUE(AP_Param::set_object_value(&rchannels, RC_Channels_Test::var_info, "_OPTIONS", float(bits)));
    }

    // feed one SBUS frame with the given channel values (and flags)
    // to the decoder, a frame gap after the previous one
    void feed_frame(uint16_t pwm, uint8_t flags = 0)
    {
        uint16_t values[NUM_CHANNELS];
        for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
            values[i] = pwm + i;
        }
        feed_frame(values, flags);
    }

    void feed_frame(uint16_t *values, uint8_t flags)
    {
        uint8_t frame[25];
        AP_SBusOut::sbus_format_frame(values, NUM_CHANNELS, frame);
        frame[SBUS_FLAGS_BYTE] = flags;

        now_us += 10 * SBUS_FRAME_GAP_US;
        hal.scheduler->stop_clock(now_us);
        for (uint8_t i = 0; i < sizeof(frame); i++) {
            sbus->process_byte(frame[i], SBUS_BAUD);
        }
    }

    // the frame a receiver sends when it has lost the link: every
    // channel at the SBUS minimum (875us), failsafe bit set
    void feed_failsafe_frame()
    {
        feed_frame(875, 1U << SBUS_FAILSAFE_BIT);
    }

    // channels as the autopilot would read them
    void expect_channels(uint16_t pwm)
    {
        for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
            EXPECT_EQ(pwm + i, sbus->read(i)) << "channel " << unsigned(i);
        }
    }

    AP_RCProtocol_SBUS *sbus;
    uint64_t now_us;
};

TEST_F(SBUSFailsafe, valid_frame_updates_channels)
{
    feed_frame(1100);
    expect_channels(1100);
    EXPECT_EQ(NUM_CHANNELS, sbus->num_channels());
    EXPECT_FALSE(frontend().failsafe_active());
    EXPECT_EQ(1U, sbus->get_rc_input_count());
    EXPECT_TRUE(sbus->new_input());

    feed_frame(1300);
    expect_channels(1300);
    EXPECT_EQ(2U, sbus->get_rc_input_count());
}

TEST_F(SBUSFailsafe, lost_frame_flag_is_not_a_failsafe)
{
    feed_frame(1100);
    feed_frame(1300, 1U << SBUS_FRAMELOST_BIT);
    expect_channels(1300);
    EXPECT_FALSE(frontend().failsafe_active());
    EXPECT_EQ(2U, sbus->get_rc_input_count());
}

TEST_F(SBUSFailsafe, failsafe_frame_keeps_channels)
{
    feed_frame(1100);
    ASSERT_TRUE(sbus->new_input());

    feed_failsafe_frame();

    // the last accepted values and channel count survive
    expect_channels(1100);
    EXPECT_EQ(NUM_CHANNELS, sbus->num_channels());
    // the failsafe is still reported ...
    EXPECT_TRUE(frontend().failsafe_active());
    // ... and the frame is not counted as input, so the link loss
    // timeout still sees no new input
    EXPECT_EQ(1U, sbus->get_rc_input_count());
    EXPECT_FALSE(sbus->new_input());
    // but it is still a frame, which is what protocol detection uses
    EXPECT_EQ(2U, sbus->get_rc_frame_count());

    // a stream of failsafe frames never produces new input
    for (uint8_t i = 0; i < 10; i++) {
        feed_failsafe_frame();
        expect_channels(1100);
        EXPECT_FALSE(sbus->new_input());
    }
}

TEST_F(SBUSFailsafe, failsafe_bit_with_plausible_values_keeps_channels)
{
    // the receiver set the failsafe bit but the values look fine
    feed_frame(1100);
    feed_frame(1400, 1U << SBUS_FAILSAFE_BIT);
    expect_channels(1100);
    EXPECT_TRUE(frontend().failsafe_active());
    EXPECT_EQ(1U, sbus->get_rc_input_count());
}

TEST_F(SBUSFailsafe, invalid_data_frame_keeps_channels)
{
    // no failsafe bit, but the first channels are at the SBUS minimum,
    // which the decoder treats as a failsafe
    feed_frame(1100);
    feed_frame(875);
    expect_channels(1100);
    EXPECT_TRUE(frontend().failsafe_active());
    EXPECT_EQ(1U, sbus->get_rc_input_count());
}

TEST_F(SBUSFailsafe, first_frame_in_failsafe_accepts_nothing)
{
    feed_failsafe_frame();
    EXPECT_EQ(0U, sbus->num_channels());
    EXPECT_EQ(0U, sbus->read(0));
    EXPECT_TRUE(frontend().failsafe_active());
    EXPECT_EQ(0U, sbus->get_rc_input_count());
    EXPECT_FALSE(sbus->new_input());
}

TEST_F(SBUSFailsafe, recovers_on_next_valid_frame)
{
    feed_frame(1100);
    feed_failsafe_frame();
    feed_failsafe_frame();
    expect_channels(1100);

    feed_frame(1600);
    expect_channels(1600);
    EXPECT_EQ(NUM_CHANNELS, sbus->num_channels());
    EXPECT_FALSE(frontend().failsafe_active());
    EXPECT_EQ(2U, sbus->get_rc_input_count());
    EXPECT_TRUE(sbus->new_input());
}

TEST_F(SBUSFailsafe, ignore_failsafe_does_not_apply_failsafe_frame)
{
    set_ignore_failsafe(true);

    feed_frame(1100);
    ASSERT_TRUE(sbus->new_input());

    feed_failsafe_frame();

    // IGNORE_FAILSAFE only stops the failsafe action: the rejected
    // frame must still not reach the channel values
    expect_channels(1100);
    EXPECT_EQ(NUM_CHANNELS, sbus->num_channels());
    // the receiver's failsafe is still reported, and the option makes
    // the frame count as input as before
    EXPECT_TRUE(frontend().failsafe_active());
    EXPECT_EQ(2U, sbus->get_rc_input_count());
    EXPECT_TRUE(sbus->new_input());

    feed_frame(1600);
    expect_channels(1600);
    EXPECT_FALSE(frontend().failsafe_active());
}

AP_GTEST_MAIN()
