#pragma once

#include <RC_Channel/RC_Channel.h>

class RC_Channel_Sub : public RC_Channel
{

public:

protected:
#if RC_ENABLED
    __INITFUNC__ void init_aux_function(AUX_FUNC ch_option, AuxSwitchPos) override;
    bool do_aux_function(const AuxFuncTrigger &trigger) override;
#endif
    
private:
#if RC_ENABLED
    // called when the mode switch changes position:
    void mode_switch_changed(modeswitch_pos_t new_pos) override;
#endif
};

class RC_Channels_Sub : public RC_Channels
{
public:
#if RC_ENABLED
    bool has_valid_input() const override;
    bool in_rc_failsafe() const override;
#endif
    RC_Channel_Sub obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Sub *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    // note that these callbacks are not presently used on Plane:
    int8_t flight_mode_channel_number() const override;

};
