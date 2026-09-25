// covers what RC_Channel does with the switch position an auxiliary
// function is initialised from.  unlike test_aux_initialisation.cpp,
// which mocks init_aux_function() to observe the phase split, this
// exercises the real RC_Channel::init_aux_function() and the real
// debounce path, because what is under test is precisely the
// interaction between the two: a function applied at boot must not be
// applied a second time by the first debounced read, and a function
// which was *not* applied at boot must still be applied by it.

#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Logger/AP_Logger.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class RC_Channel_Test : public RC_Channel
{
public:
    // run_aux_function() dispatches through do_aux_function(), so
    // counting there counts every application of the function,
    // whether it came from initialisation or from a debounced read
    uint8_t dispatches;
    AuxSwitchPos last_pos;
    AUX_FUNC last_func;
    AuxFuncTrigger::Source last_source;
    // what do_aux_function() reports back; a handler which could not
    // apply the function returns false
    bool dispatch_succeeds;

    RC_Channel_Test() :
        dispatches(0),
        last_pos(AuxSwitchPos::LOW),
        last_func(AUX_FUNC::DO_NOTHING),
        last_source(AuxFuncTrigger::Source::INIT),
        dispatch_succeeds(true)
    {
    }

protected:
    bool do_aux_function(const AuxFuncTrigger &trigger) override
    {
        dispatches++;
        last_pos = trigger.pos;
        last_func = trigger.func;
        last_source = trigger.source;
        return dispatch_succeeds;
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

// run_aux_function() logs the invocation.  file scope rather than
// automatic storage: AP_Logger::Write() walks backends[0.._next_backend),
// and an AP_Logger on the stack has an indeterminate _next_backend.
// zero-initialised in BSS it has no backends and the write is a no-op.
static AP_Logger logger;

// RC_Channels is a singleton and panics if a second instance is
// constructed, so the tests in this file share one file-scope
// instance rather than each declaring their own
static RC_Channels_Test rc_channels;

// step past the switch debounce window, calling read_aux() as the
// scheduled RC_Channels::read_aux() would
static void read_aux_through_debounce(RC_Channel_Test *c)
{
    for (uint8_t i=0; i<5; i++) {
        c->read_aux();
        hal.scheduler->delay(60);
    }
}

TEST(RCChannel, AuxFunctionBootPosition)
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        rc_channels.channel(i)->set_radio_in(0);
    }

    // AHRS_TYPE is one of the functions RC_Channel::init_aux_function()
    // applies explicitly.  the switch is readable here, which is what
    // the deferred late phase can now find on Plane, where
    // failsafe_check() reads RC while the main loop is stalled
    // through setup().  the functions used in this file are ones with
    // no entry in RC_Channel's aux-function name table, so that
    // read_aux() does not announce them: there is no GCS singleton
    // here to announce them to
    RC_Channel_Test *applied = rc_channels.channel(8);
    applied->option.set(int16_t(RC_Channel::AUX_FUNC::AHRS_TYPE));
    applied->set_radio_in(1900);

    // EKF_SOURCE_SET is in the group init_aux_function() deliberately
    // does not apply, precisely so that the first debounced read_aux()
    // applies it
    RC_Channel_Test *not_applied = rc_channels.channel(9);
    not_applied->option.set(int16_t(RC_Channel::AUX_FUNC::EKF_SOURCE_SET));
    not_applied->set_radio_in(1900);

    // a third switch is moved between initialisation and the first
    // read, which must still be picked up
    RC_Channel_Test *moved = rc_channels.channel(10);
    moved->option.set(int16_t(RC_Channel::AUX_FUNC::AHRS_TYPE));
    moved->set_radio_in(1900);

    rc_channels.init();
    rc_channels.init_aux();

    EXPECT_EQ(1, applied->dispatches);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::HIGH, applied->last_pos);
    EXPECT_EQ(RC_Channel::AuxFuncTrigger::Source::INIT, applied->last_source);

    EXPECT_EQ(0, not_applied->dispatches);

    // a function applied from a live boot position is not applied a
    // second time by the first debounced read of a switch which has
    // not moved
    read_aux_through_debounce(applied);
    EXPECT_EQ(1, applied->dispatches);

    // whereas a function which initialisation did not apply is still
    // applied by that read
    read_aux_through_debounce(not_applied);
    EXPECT_EQ(1, not_applied->dispatches);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::HIGH, not_applied->last_pos);
    EXPECT_EQ(RC_Channel::AuxFuncTrigger::Source::RC, not_applied->last_source);

    // and recording the boot position does not swallow a genuine
    // change: this switch was HIGH at initialisation and is LOW by the
    // time it is read
    EXPECT_EQ(1, moved->dispatches);
    moved->set_radio_in(1000);
    read_aux_through_debounce(moved);
    EXPECT_EQ(2, moved->dispatches);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::LOW, moved->last_pos);
    EXPECT_EQ(RC_Channel::AuxFuncTrigger::Source::RC, moved->last_source);
}

// the same two functions, but with the switch unreadable at
// initialisation - the usual case, and the only one on vehicles whose
// timer failsafe does not read RC.  the boot position is defaulted
// rather than read, so it must not be recorded: the pilot's real
// position has to be acted upon once RC comes up.
TEST(RCChannel, AuxFunctionDefaultedBootPosition)
{
    RC_Channel_Test *applied = rc_channels.channel(11);
    applied->option.set(int16_t(RC_Channel::AUX_FUNC::AHRS_TYPE));
    applied->set_radio_in(0);  // unreadable

    applied->init_aux_early();
    applied->init_aux();

    // initialised, from the defaulted LOW position
    EXPECT_EQ(1, applied->dispatches);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::LOW, applied->last_pos);

    // RC comes up with the switch HIGH; the defaulted position was not
    // recorded, so this is seen as a change and applied
    applied->set_radio_in(1900);
    read_aux_through_debounce(applied);
    EXPECT_EQ(2, applied->dispatches);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::HIGH, applied->last_pos);
    EXPECT_EQ(RC_Channel::AuxFuncTrigger::Source::RC, applied->last_source);

    // and the case that distinguishes a defaulted position from a
    // read one: RC comes up with the switch in the same LOW position
    // that initialisation defaulted to.  the function is applied
    // again, because that first application was against a position
    // nobody had actually read.  recording defaulted positions would
    // swallow this
    RC_Channel_Test *unread_then_low = rc_channels.channel(12);
    unread_then_low->option.set(int16_t(RC_Channel::AUX_FUNC::AHRS_TYPE));
    unread_then_low->set_radio_in(0);

    unread_then_low->init_aux_early();
    unread_then_low->init_aux();
    EXPECT_EQ(1, unread_then_low->dispatches);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::LOW, unread_then_low->last_pos);

    unread_then_low->set_radio_in(1000);
    read_aux_through_debounce(unread_then_low);
    EXPECT_EQ(2, unread_then_low->dispatches);
    EXPECT_EQ(RC_Channel::AuxSwitchPos::LOW, unread_then_low->last_pos);
    EXPECT_EQ(RC_Channel::AuxFuncTrigger::Source::RC, unread_then_low->last_source);
}

// three cases where the first debounced read is still owed an
// application, so the boot position must not be recorded even though
// it was read live and the function was one initialisation applies
TEST(RCChannel, AuxFunctionBootPositionNotRecorded)
{
    // the channel's function is changed by a GCS between the two
    // initialisation phases.  init_aux_early() latches the function
    // for both phases deliberately, so the late phase applies the
    // latched one - but switch_state is per-channel, not
    // per-function, so recording its position would suppress the read
    // which is the only thing that will ever apply the replacement
    RC_Channel_Test *option_changed = rc_channels.channel(13);
    option_changed->option.set(int16_t(RC_Channel::AUX_FUNC::AHRS_TYPE));
    option_changed->set_radio_in(1900);

    option_changed->init_aux_early();
    // MAVLink is serviced between the phases by
    // AP_Vehicle::scheduler_delay_callback(), so a PARAM_SET can land
    // here
    option_changed->option.set(int16_t(RC_Channel::AUX_FUNC::EKF_SOURCE_SET));
    option_changed->init_aux();

    // the latched function was applied, as the latch intends
    EXPECT_EQ(1, option_changed->dispatches);
    EXPECT_EQ(RC_Channel::AUX_FUNC::AHRS_TYPE, option_changed->last_func);

    // and the channel's new function is still applied by the first
    // debounced read
    read_aux_through_debounce(option_changed);
    EXPECT_EQ(2, option_changed->dispatches);
    EXPECT_EQ(RC_Channel::AUX_FUNC::EKF_SOURCE_SET, option_changed->last_func);
    EXPECT_EQ(RC_Channel::AuxFuncTrigger::Source::RC, option_changed->last_source);

    // a handler which could not apply the function at boot - e.g. a
    // MAVLink camera which has not finished discovery - is retried by
    // the first debounced read rather than left unapplied
    RC_Channel_Test *dispatch_failed = rc_channels.channel(14);
    dispatch_failed->option.set(int16_t(RC_Channel::AUX_FUNC::AHRS_TYPE));
    dispatch_failed->set_radio_in(1900);
    dispatch_failed->dispatch_succeeds = false;

    dispatch_failed->init_aux_early();
    dispatch_failed->init_aux();
    EXPECT_EQ(1, dispatch_failed->dispatches);

    dispatch_failed->dispatch_succeeds = true;
    read_aux_through_debounce(dispatch_failed);
    EXPECT_EQ(2, dispatch_failed->dispatches);
    EXPECT_EQ(RC_Channel::AuxFuncTrigger::Source::RC, dispatch_failed->last_source);
}

AP_GTEST_MAIN()
