/*
  test that SBUS frames rejected as invalid data never reach the RC
  input, with or without RC_OPTIONS IGNORE_FAILSAFE, while frames with
  the receiver's failsafe bit keep their existing handling
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

#define SBUS_BAUD 100000U
#define SBUS_FRAME_SIZE 25
#define SBUS_FLAGS_BYTE 23
#define SBUS_FRAMELOST_BIT 2
#define SBUS_FAILSAFE_BIT 3
#define SBUS_NUM_CHANNELS 16

// lowest value SBUS can carry; the decoder rejects a frame with any
// of the first 4 channels at or below it
#define SBUS_MIN_PWM 875

static void set_rc_options(uint32_t options)
{
    ASSERT_TRUE(AP_Param::set_object_value(&rchannels, RC_Channels::var_info, "_OPTIONS", options));
}

static void set_ignore_failsafe(bool ignore)
{
    set_rc_options(ignore ? uint32_t(RC_Channels::Option::IGNORE_FAILSAFE) : 0);
}

static void format_frame(uint8_t frame[SBUS_FRAME_SIZE], const uint16_t pwm[SBUS_NUM_CHANNELS], uint8_t flags)
{
    uint16_t values[SBUS_NUM_CHANNELS];
    memcpy(values, pwm, sizeof(values));
    AP_SBusOut::sbus_format_frame(values, SBUS_NUM_CHANNELS, frame);
    frame[SBUS_FLAGS_BYTE] = flags;
}

// every channel bit zero; the encoder can't produce this, as it
// sends the minimum value as 1
static void format_zeroed_frame(uint8_t frame[SBUS_FRAME_SIZE], uint8_t flags)
{
    memset(frame, 0, SBUS_FRAME_SIZE);
    frame[0] = 0x0F;
    frame[SBUS_FLAGS_BYTE] = flags;
}

// feed one frame to an SBUS backend or the frontend after a frame
// gap; the clock only moves forward, whatever order the tests run in
template <typename T>
static void feed(T &decoder, const uint8_t frame[SBUS_FRAME_SIZE])
{
    static uint64_t now_us;
    now_us = MAX(now_us, AP_HAL::micros64()) + 10000;
    hal.scheduler->stop_clock(now_us);
    for (uint8_t i=0; i<SBUS_FRAME_SIZE; i++) {
        decoder.process_byte(frame[i], SBUS_BAUD);
    }
}

// valid values, different on each channel
static void fill(uint16_t pwm[SBUS_NUM_CHANNELS], uint16_t base)
{
    for (uint8_t i=0; i<SBUS_NUM_CHANNELS; i++) {
        pwm[i] = base + 10*i;
    }
}

class SBUSInvalidData : public ::testing::Test
{
protected:
    void SetUp() override
    {
        set_rc_options(0);
        sbus = NEW_NOTHROW AP_RCProtocol_SBUS(frontend(), false, SBUS_BAUD);
        ASSERT_NE(sbus, nullptr);
        fill(good, 1100);
        fill(other, 1300);
    }

    void TearDown() override
    {
        delete sbus;
        set_rc_options(0);
    }

    void feed_frame(const uint16_t pwm[SBUS_NUM_CHANNELS], uint8_t flags=0)
    {
        uint8_t frame[SBUS_FRAME_SIZE];
        format_frame(frame, pwm, flags);
        feed(*sbus, frame);
    }

    void feed_zeroed_frame(uint8_t flags=0)
    {
        uint8_t frame[SBUS_FRAME_SIZE];
        format_zeroed_frame(frame, flags);
        feed(*sbus, frame);
    }

    void expect_channels(const uint16_t pwm[SBUS_NUM_CHANNELS])
    {
        EXPECT_EQ(SBUS_NUM_CHANNELS, sbus->num_channels());
        for (uint8_t i=0; i<SBUS_NUM_CHANNELS; i++) {
            EXPECT_EQ(pwm[i], sbus->read(i)) << "channel " << unsigned(i+1);
        }
    }

    // a valid frame is used and counted as input
    void expect_accepted(const uint16_t pwm[SBUS_NUM_CHANNELS])
    {
        expect_channels(pwm);
        EXPECT_TRUE(sbus->new_input());
        EXPECT_FALSE(frontend().failsafe_active());
    }

    // send one frame with feed_rejected(), and check it changed
    // nothing but the failsafe flag and the frame count
    template <typename F>
    void expect_rejected(const uint16_t held[SBUS_NUM_CHANNELS], F feed_rejected)
    {
        const uint32_t input_count = sbus->get_rc_input_count();
        const uint32_t frame_count = sbus->get_rc_frame_count();
        feed_rejected();
        expect_channels(held);
        EXPECT_FALSE(sbus->new_input());
        EXPECT_EQ(input_count, sbus->get_rc_input_count());
        // still a frame, which is what protocol detection counts
        EXPECT_EQ(frame_count+1, sbus->get_rc_frame_count());
        EXPECT_TRUE(frontend().failsafe_active());
    }

    AP_RCProtocol_SBUS *sbus;
    uint16_t good[SBUS_NUM_CHANNELS];
    uint16_t other[SBUS_NUM_CHANNELS];
};

TEST_F(SBUSInvalidData, valid_frames_are_input)
{
    feed_frame(good);
    expect_accepted(good);
    EXPECT_EQ(1U, sbus->get_rc_input_count());

    feed_frame(other);
    expect_accepted(other);
    EXPECT_EQ(2U, sbus->get_rc_input_count());
}

TEST_F(SBUSInvalidData, each_low_channel_rejects_frame)
{
    for (const bool ignore : { false, true }) {
        set_ignore_failsafe(ignore);
        for (uint8_t chan=0; chan<4; chan++) {
            SCOPED_TRACE(testing::Message() << "IGNORE_FAILSAFE " << ignore << " channel " << unsigned(chan+1));
            feed_frame(good);
            expect_accepted(good);

            // the reported problem: with IGNORE_FAILSAFE this frame
            // was copied into the channels and counted as input
            uint16_t bad[SBUS_NUM_CHANNELS];
            memcpy(bad, other, sizeof(bad));
            bad[chan] = SBUS_MIN_PWM;
            expect_rejected(good, [&] { feed_frame(bad); });

            // the next valid frame is accepted
            feed_frame(other);
            expect_accepted(other);
        }
    }
}

TEST_F(SBUSInvalidData, zeroed_frame_rejected)
{
    for (const bool ignore : { false, true }) {
        set_ignore_failsafe(ignore);
        SCOPED_TRACE(testing::Message() << "IGNORE_FAILSAFE " << ignore);
        feed_frame(good);
        expect_accepted(good);

        expect_rejected(good, [&] { feed_zeroed_frame(); });
        // the receiver's failsafe bit doesn't make invalid data usable
        expect_rejected(good, [&] { feed_zeroed_frame(1U << SBUS_FAILSAFE_BIT); });
        // nor does a lost frame flag
        expect_rejected(good, [&] { feed_zeroed_frame(1U << SBUS_FRAMELOST_BIT); });
    }
}

TEST_F(SBUSInvalidData, only_first_four_channels_at_minimum_are_invalid)
{
    set_ignore_failsafe(true);

    // just above the minimum is valid
    uint16_t low[SBUS_NUM_CHANNELS];
    memcpy(low, good, sizeof(low));
    for (uint8_t i=0; i<4; i++) {
        low[i] = SBUS_MIN_PWM + 1;
    }
    feed_frame(low);
    expect_accepted(low);

    // channels after the first 4 may be at the minimum
    uint16_t aux_low[SBUS_NUM_CHANNELS];
    memcpy(aux_low, other, sizeof(aux_low));
    for (uint8_t i=4; i<SBUS_NUM_CHANNELS; i++) {
        aux_low[i] = SBUS_MIN_PWM;
    }
    feed_frame(aux_low);
    expect_accepted(aux_low);
}

TEST_F(SBUSInvalidData, repeated_invalid_frames_give_no_input)
{
    for (const bool ignore : { false, true }) {
        set_ignore_failsafe(ignore);
        SCOPED_TRACE(testing::Message() << "IGNORE_FAILSAFE " << ignore);
        feed_frame(good);
        expect_accepted(good);

        // a receiver which keeps sending invalid frames must look like
        // no input at all, so the vehicle's RC timeout can still trigger
        for (uint8_t i=0; i<50; i++) {
            expect_rejected(good, [&] { feed_zeroed_frame(); });
        }

        feed_frame(other);
        expect_accepted(other);
    }
}

TEST_F(SBUSInvalidData, lost_frame_flag_is_not_a_failsafe)
{
    feed_frame(good);
    expect_accepted(good);

    feed_frame(other, 1U << SBUS_FRAMELOST_BIT);
    expect_accepted(other);
}

TEST_F(SBUSInvalidData, failsafe_bit_frame_not_input)
{
    feed_frame(good);
    expect_accepted(good);
    const uint32_t input_count = sbus->get_rc_input_count();

    // a receiver failsafe preset with low throttle, still valid data
    uint16_t preset[SBUS_NUM_CHANNELS];
    memcpy(preset, good, sizeof(preset));
    preset[2] = 950;
    feed_frame(preset, 1U << SBUS_FAILSAFE_BIT);
    EXPECT_FALSE(sbus->new_input());
    EXPECT_EQ(input_count, sbus->get_rc_input_count());
    EXPECT_TRUE(frontend().failsafe_active());

    feed_frame(other);
    expect_accepted(other);
}

TEST_F(SBUSInvalidData, failsafe_bit_frame_is_input_with_ignore_failsafe)
{
    // IGNORE_FAILSAFE exists so that the values a receiver sends in
    // failsafe (e.g. HOLD, or a preset with low throttle) are used;
    // that must not change
    set_ignore_failsafe(true);

    feed_frame(good);
    expect_accepted(good);

    uint16_t preset[SBUS_NUM_CHANNELS];
    memcpy(preset, good, sizeof(preset));
    preset[2] = 950;
    feed_frame(preset, 1U << SBUS_FAILSAFE_BIT);
    expect_channels(preset);
    EXPECT_TRUE(sbus->new_input());
    EXPECT_TRUE(frontend().failsafe_active());
}

TEST_F(SBUSInvalidData, frontend_detects_sbus_from_invalid_frames)
{
    // a frontend of our own, so that nothing has been detected yet
    AP_RCProtocol *rcp = NEW_NOTHROW AP_RCProtocol();
    ASSERT_NE(rcp, nullptr);
    AP_RCProtocol &rc = *rcp;
    rc.init();
    ASSERT_EQ(AP_RCProtocol::NONE, rc.protocol_detected());
    set_ignore_failsafe(true);

    uint8_t zeroed[SBUS_FRAME_SIZE];
    format_zeroed_frame(zeroed, 0);
    uint8_t good_frame[SBUS_FRAME_SIZE];
    format_frame(good_frame, good, 0);

    // invalid frames are still SBUS frames, so they lock on to the
    // protocol as frames with the failsafe bit do, without giving
    // any channel values
    for (uint8_t i=0; i<3; i++) {
        feed(rc, zeroed);
    }
    EXPECT_EQ(AP_RCProtocol::SBUS, rc.protocol_detected());
    EXPECT_EQ(0U, rc.num_channels());
    EXPECT_TRUE(rc.failsafe_active());

    feed(rc, good_frame);
    EXPECT_EQ(SBUS_NUM_CHANNELS, rc.num_channels());
    EXPECT_EQ(good[0], rc.read(0));
    EXPECT_FALSE(rc.failsafe_active());

    for (uint8_t i=0; i<10; i++) {
        feed(rc, zeroed);
    }
    EXPECT_EQ(AP_RCProtocol::SBUS, rc.protocol_detected());
    for (uint8_t i=0; i<SBUS_NUM_CHANNELS; i++) {
        EXPECT_EQ(good[i], rc.read(i)) << "channel " << unsigned(i+1);
    }
    EXPECT_TRUE(rc.failsafe_active());

    delete rcp;
}

AP_GTEST_MAIN()
