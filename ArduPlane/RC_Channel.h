#pragma once

#include <RC_Channel/RC_Channel.h>

class RC_Channel_Plane : public RC_Channel
{

public:

protected:

    void init_aux_function(AUX_FUNC ch_option,
                           AuxSwitchPos ch_flag) override;
    bool do_aux_function(const AuxFuncTrigger &trigger) override;

    // called when the mode switch changes position:
    void mode_switch_changed(modeswitch_pos_t new_pos) override;

private:

    void do_aux_function_change_mode(Mode::Number number,
                                     AuxSwitchPos ch_flag);

#if HAL_QUADPLANE_ENABLED
    void do_aux_function_q_assist_state(AuxSwitchPos ch_flag);
#endif

    void do_aux_function_crow_mode(AuxSwitchPos ch_flag);

    void do_aux_function_soaring_3pos(AuxSwitchPos ch_flag);

    void do_aux_function_flare(AuxSwitchPos ch_flag);
};

class RC_Channels_Plane : public RC_Channels
{
public:

    RC_Channel_Plane obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Plane *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    bool in_rc_failsafe() const override;
    bool has_valid_input() const override;

    RC_Channel *get_arming_channel(void) const override;

    void read_mode_switch() override;

protected:

    // note that these callbacks are not presently used on Plane:
    int8_t flight_mode_channel_number() const override;

};
