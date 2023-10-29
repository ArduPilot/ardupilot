#pragma once

#include <RC_Channel/RC_Channel.h>
#include "Rover.h"
#include "mode.h"

class RC_Channel_Rover : public RC_Channel
{

public:

protected:

    void init_aux_function(aux_func_t ch_option, AuxSwitchPos) override;
    bool do_aux_function(aux_func_t ch_option, AuxSwitchPos) override;

    // called when the mode switch changes position:
    void mode_switch_changed(modeswitch_pos_t new_pos) override;

private:

    void do_aux_function_change_mode(Mode &mode,
                                     const AuxSwitchPos ch_flag);

    void add_waypoint_for_current_loc();

    void do_aux_function_sailboat_motor_3pos(const AuxSwitchPos ch_flag);
};

class RC_Channels_Rover : public RC_Channels
{

public:

    bool in_rc_failsafe() const override;
    bool has_valid_input() const override;

    RC_Channel *get_arming_channel(void) const override;

    RC_Channel_Rover obj_channels[NUM_RC_CHANNELS];

    RC_Channel_Rover *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

private:

    int8_t flight_mode_channel_number() const override;
};
