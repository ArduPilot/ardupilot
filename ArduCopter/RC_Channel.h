#pragma once

#include <RC_Channel/RC_Channel.h>

class RC_Channel_Copter : public RC_Channel
{

public:

protected:

private:

    // called when the mode switch changes position:
    void mode_switch_changed(modeswitch_pos_t new_pos) override;

};

class RC_Channels_Copter : public RC_Channels
{
public:

    bool has_valid_input() const override;

    RC_Channel *get_arming_channel(void) const override;

    RC_Channel_Copter obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Copter *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    int8_t flight_mode_channel_number() const override;

};
