#pragma once

#include <RC_Channel/RC_Channel.h>
#include "Copter.h"

class RC_Channel_Copter : public RC_Channel
{

public:

protected:

    void init_aux_function(aux_func_t ch_option, aux_switch_pos_t) override;
    void do_aux_function(aux_func_t ch_option, aux_switch_pos_t) override;

private:

    void do_aux_function_change_mode(const control_mode_t mode,
                                     const aux_switch_pos_t ch_flag);

    // called when the mode switch changes position:
    void mode_switch_changed(modeswitch_pos_t new_pos) override;

};

class RC_Channels_Copter : public RC_Channels
{
public:

    // this must be implemented for the AP_Scheduler functor to work:
    void read_aux_all() override {
        RC_Channels::read_aux_all();
    }

    bool has_valid_input() const override;

    RC_Channel_Copter obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Copter *channel(const uint8_t chan) override {
        if (chan > NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    int8_t flight_mode_channel_number() const override;

};
