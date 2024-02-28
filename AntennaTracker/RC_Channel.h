#pragma once

#include <RC_Channel/RC_Channel.h>

class RC_Channel_Tracker : public RC_Channel
{

public:

protected:

private:

};

class RC_Channels_Tracker : public RC_Channels
{
public:

    RC_Channel_Tracker obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Tracker *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    // we always trust our RC inputs ATM:
    bool in_rc_failsafe() const override { return false; }

protected:

    // note that these callbacks are not presently used on Tracker:
    int8_t flight_mode_channel_number() const override;

};
