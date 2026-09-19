#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class RC_Channel_Test : public RC_Channel
{
public:
    // each test channel carries a single RCx_OPTION, so recording the
    // most recent initialisation is enough to describe it
    uint8_t init_count;
    AUX_FUNC init_func;
    AuxSwitchPos init_position;

    RC_Channel_Test() :
        init_count(0),
        init_func(AUX_FUNC::DO_NOTHING),
        init_position(AuxSwitchPos::LOW)
    {
    }

protected:
    // a vehicle subclass classifying one of its own functions as
    // early, in the way a vehicle would: by delegating to the base
    // list rather than re-enumerating it.  no vehicle needs this
    // today, which is why it is exercised here
    bool init_aux_function_early(const AUX_FUNC func) const override
    {
        if (func == AUX_FUNC::MISSION_RESET) {
            return true;
        }
        return RC_Channel::init_aux_function_early(func);
    }

    void init_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag) override
    {
        init_count++;
        init_func = ch_option;
        init_position = ch_flag;
    }
};

class RC_Channels_Test : public RC_Channels
{
public:
    RC_Channel_Test obj_channels[NUM_RC_CHANNELS];

    RC_Channel_Test *channel(const uint8_t chan) override
    {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    const RC_Channel_Test *channel(const uint8_t chan) const override
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

// RC_Channels is a singleton (it panics if a second instance is
// constructed while one already exists, and nothing clears the
// singleton pointer on destruction), so all scenarios needing an
// RC_Channels_Test instance share this single test body rather than
// each getting their own local instance.
TEST(RCChannel, AuxFunctionInitialisationPhases)
{
    RC_Channels_Test rc_channels;

    // channel index 4 is reserved by
    // RC_Channels_Test::flight_mode_channel_number() (returning the
    // 1-indexed channel number 5) via channel(num - 1); the remaining
    // indices used here are arbitrary.

    // RC_Channel does not zero radio_in and rc_channels here is
    // automatic storage, so seed every channel rather than leaving the
    // ones this test does not configure to be read indeterminate: both
    // initialisation phases sweep all NUM_RC_CHANNELS channels, and
    // reset_mode_switch() reads the flight-mode channel.  0 is below
    // RC_MIN_LIMIT_PWM, so it makes those reads fail deterministically.
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        rc_channels.channel(i)->set_radio_in(0);
    }

    // MOTOR_ESTOP gates the outputs from boot and has no backend
    // dependency, so it is initialised early, from the live switch.
    RC_Channel_Test *motor_estop_channel = rc_channels.channel(6);
    motor_estop_channel->option.set(int16_t(RC_Channel::AUX_FUNC::MOTOR_ESTOP));
    motor_estop_channel->set_radio_in(1900);

    // ARM_EMERGENCY_STOP is initialised early too, so that
    // emergency-stop is engaged for the whole of startup.
    RC_Channel_Test *arm_estop_channel = rc_channels.channel(3);
    arm_estop_channel->option.set(int16_t(RC_Channel::AUX_FUNC::ARM_EMERGENCY_STOP));
    arm_estop_channel->set_radio_in(1900);  // switch physically HIGH at boot

    // RC_OVERRIDE_ENABLE gates GCS overrides from boot.
    RC_Channel_Test *rc_override_channel = rc_channels.channel(7);
    rc_override_channel->option.set(int16_t(RC_Channel::AUX_FUNC::RC_OVERRIDE_ENABLE));
    rc_override_channel->set_radio_in(1900);

    // FENCE acts on a backend created during vehicle initialisation,
    // so it must not be initialised until that backend exists.
    RC_Channel_Test *fence_channel = rc_channels.channel(8);
    fence_channel->option.set(int16_t(RC_Channel::AUX_FUNC::FENCE));
    fence_channel->set_radio_in(1900);

    // a second late function, but with RC not readable, which is the
    // usual state at RC_Channels::init(); its switch position is
    // defaulted rather than read
    RC_Channel_Test *unreadable_channel = rc_channels.channel(10);
    unreadable_channel->option.set(int16_t(RC_Channel::AUX_FUNC::FENCE));

    // a function the subclass has classified as early, which the base
    // class classifies as late
    RC_Channel_Test *subclass_early_channel = rc_channels.channel(11);
    subclass_early_channel->option.set(int16_t(RC_Channel::AUX_FUNC::MISSION_RESET));
    subclass_early_channel->set_radio_in(1900);

    rc_channels.init();

    // the subclass's own early function is initialised by the early
    // phase, without the subclass having had to re-enumerate the base
    // list to get there
    EXPECT_EQ(1, subclass_early_channel->init_count);
    EXPECT_EQ(RC_Channel::AUX_FUNC::MISSION_RESET, subclass_early_channel->init_func);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::HIGH, subclass_early_channel->init_position);

    // MOTOR_ESTOP is safe to initialise against the live switch position.
    EXPECT_EQ(1, motor_estop_channel->init_count);
    EXPECT_EQ(RC_Channel::AUX_FUNC::MOTOR_ESTOP, motor_estop_channel->init_func);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::HIGH, motor_estop_channel->init_position);

    // ARM_EMERGENCY_STOP must never be initialised against a live
    // switch position: a transmitter left with the switch physically
    // HIGH at boot must not clear emergency-stop / request arming
    // during init. It is still initialised (to the safe LOW /
    // e-stop-engaged position); the pilot's actual switch position is
    // picked up later via the debounced, first-read-guarded read_aux()
    // path.
    EXPECT_EQ(1, arm_estop_channel->init_count);
    EXPECT_EQ(RC_Channel::AUX_FUNC::ARM_EMERGENCY_STOP, arm_estop_channel->init_func);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::LOW, arm_estop_channel->init_position);

    EXPECT_EQ(1, rc_override_channel->init_count);
    EXPECT_EQ(RC_Channel::AUX_FUNC::RC_OVERRIDE_ENABLE, rc_override_channel->init_func);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::HIGH, rc_override_channel->init_position);

    // the backend-dependent function has not been initialised yet
    EXPECT_EQ(0, fence_channel->init_count);

    rc_channels.init_aux();

    // the backend-dependent function is initialised now that the
    // backends exist
    EXPECT_EQ(1, fence_channel->init_count);
    EXPECT_EQ(RC_Channel::AUX_FUNC::FENCE, fence_channel->init_func);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::HIGH, fence_channel->init_position);

    // a late function whose switch could not be read is still
    // initialised, to the defaulted LOW position; the pilot's real
    // position is picked up by read_aux() once RC becomes readable
    EXPECT_EQ(1, unreadable_channel->init_count);
    EXPECT_EQ(RC_Channel::AUX_FUNC::FENCE, unreadable_channel->init_func);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::LOW, unreadable_channel->init_position);

    // and not a second time by the late phase
    EXPECT_EQ(1, subclass_early_channel->init_count);

    // the early functions are not initialised a second time; in
    // particular ARM_EMERGENCY_STOP must not be re-read here, where
    // RC is likely to be live
    EXPECT_EQ(1, motor_estop_channel->init_count);
    EXPECT_EQ(1, arm_estop_channel->init_count);
    EXPECT_EQ(1, rc_override_channel->init_count);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::LOW, arm_estop_channel->init_position);

    // the function is latched by the first phase, so a mid-boot
    // PARAM_SET arriving between the two phases cannot slip through the
    // partition and leave a channel initialised not at all.  MAVLink is
    // serviced between them by AP_Vehicle::scheduler_delay_callback().
    // this channel carried no RCx_OPTION above, so the sweeps already
    // run have initialised it as DO_NOTHING; count from there
    RC_Channel_Test *late_to_early_channel = rc_channels.channel(9);
    const uint8_t init_count_before = late_to_early_channel->init_count;

    late_to_early_channel->option.set(int16_t(RC_Channel::AUX_FUNC::FENCE));
    late_to_early_channel->set_radio_in(1900);
    late_to_early_channel->init_aux_early();

    // a late function is not initialised by the early phase
    EXPECT_EQ(init_count_before, late_to_early_channel->init_count);

    // the GCS changes the option from a late function to an early one
    // after the early phase has run
    late_to_early_channel->option.set(int16_t(RC_Channel::AUX_FUNC::MOTOR_ESTOP));
    late_to_early_channel->init_aux();

    // the late phase still initialises the function the early phase
    // latched, rather than skipping the channel entirely
    EXPECT_EQ(init_count_before + 1, late_to_early_channel->init_count);
    EXPECT_EQ(RC_Channel::AUX_FUNC::FENCE, late_to_early_channel->init_func);
}

AP_GTEST_MAIN()
