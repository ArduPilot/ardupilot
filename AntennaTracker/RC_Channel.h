#pragma once

#include <RC_Channel/RC_Channel.h>

class RC_Channel_Tracker : public RC_Channel
{

public:

    void do_aux_function(aux_func_t ch_option, AuxSwitchPos) override;

protected:

private:

    void do_aux_function_quick_tune(const AuxSwitchPos ch_flag);

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

protected:

    // note that these callbacks are not presently used on Tracker:
    int8_t flight_mode_channel_number() const override;

};
