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

    void init() override;
    void set_control_channels(void);

    bool has_valid_input() const override;

    RC_Channel *get_arming_channel(void) const override;

    RC_Channel_Rover obj_channels[NUM_RC_CHANNELS];

    RC_Channel_Rover *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    // primary control channels
    RC_Channel *channel_throttle;
    RC_Channel *channel_steer;
    RC_Channel *channel_lateral;

protected:

    bool k_param_rcmap_for_conversion(uint8_t &k_param_rcmap) const override;

    // get_option_defaults provides a map from user-visible channel
    // numbers to the default _OPTION value for that channel number.
    // For example, typically users have STEERING input on channel 1.
    void get_option_defaults(const struct RC_Channels::OptionDefault *&defaults,
                             uint8_t &num_defaults) override;

private:

    int8_t flight_mode_channel_number() const override;

    static const RC_Channels::OptionDefault option_defaults[];

};
