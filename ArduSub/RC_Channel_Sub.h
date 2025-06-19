#pragma once

#include <RC_Channel/RC_Channel.h>
#include "config.h"

#if AP_SUB_RC_ENABLED
class RC_Channel_Sub : public RC_Channel
{

public:

protected:

    __INITFUNC__ void init_aux_function(AUX_FUNC ch_option, AuxSwitchPos) override;
    bool do_aux_function(const AuxFuncTrigger &trigger) override;
    
private:
    // called when the mode switch changes position:
    void mode_switch_changed(modeswitch_pos_t new_pos) override;
};

class RC_Channels_Sub : public RC_Channels
{
public:
    bool has_valid_input() const override;
    bool in_rc_failsafe() const override;
    // returns true if throttle arming checks should be run
    bool arming_check_throttle() const override;
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

#else

class RC_Channel_Sub : public RC_Channel
{

public:

protected:

private:

};

class RC_Channels_Sub : public RC_Channels
{
public:

    RC_Channel_Sub obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Sub *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    // tell the gimbal code all is good with RC input:
    bool in_rc_failsafe() const override { return false; };
    bool arming_check_throttle() const override;

protected:

    // note that these callbacks are not presently used on Plane:
    int8_t flight_mode_channel_number() const override;

};
#endif


