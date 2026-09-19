// pins the property the early-initialisation allowlist exists for:
// that RC_Channels::init() alone - before any backend exists, and
// before the deferred RC_Channels::init_aux() runs - leaves the
// vehicle gated.  Nothing here is mocked: the other two test files
// override init_aux_function() and do_aux_function() respectively to
// observe dispatch, which means neither of them would notice a
// function being moved out of init_aux_function_early() and the gate
// silently not being established.

#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class RC_Channel_Test : public RC_Channel
{
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

// run_aux_function() logs the invocation; file scope so that it is
// zero-initialised and Write() finds no backends to walk
static AP_Logger logger;

static RC_Channels_Test rc_channels;

TEST(RCChannel, AuxFunctionEarlyGating)
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        rc_channels.channel(i)->set_radio_in(0);
    }

    // a transmitter left with the arm switch physically HIGH at boot
    RC_Channel_Test *arm_estop = rc_channels.channel(3);
    arm_estop->option.set(int16_t(RC_Channel::AUX_FUNC::ARM_EMERGENCY_STOP));
    arm_estop->set_radio_in(1900);

    // RC is not readable this early on any vehicle, which is the
    // position this function's gate is established from
    RC_Channel_Test *rc_override = rc_channels.channel(7);
    rc_override->option.set(int16_t(RC_Channel::AUX_FUNC::RC_OVERRIDE_ENABLE));

    // SRV_Channels::emergency_stop is a zero-init static and
    // _gcs_overrides_enabled defaults true, so neither is gated until
    // the aux function establishes it
    EXPECT_FALSE(SRV_Channels::get_emergency_stop());
    EXPECT_TRUE(rc_channels.gcs_overrides_enabled());

    // RC_Channels::init() only.  RC_Channels::init_aux() is
    // deliberately not called: these functions must be gated by the
    // end of this call, not by the end of AP_Vehicle::setup()
    rc_channels.init();

    // engaged despite the switch being physically HIGH: the pilot's
    // real position is picked up later, by the debounced,
    // first-read-guarded read_aux() path
    EXPECT_TRUE(SRV_Channels::get_emergency_stop());

    // GCS RC overrides are gated for a user who put them behind a switch
    EXPECT_FALSE(rc_channels.gcs_overrides_enabled());
}

AP_GTEST_MAIN()
