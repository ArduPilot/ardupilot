#pragma once

#include <RC_Channel/RC_Channel.h>
#include <AP_Motors/AP_Motors.h>
#include "mode.h"

class RC_Channel_Copter : public RC_Channel
{

public:

protected:

    __INITFUNC__ void init_aux_function(AUX_FUNC ch_option, AuxSwitchPos) override;
    bool do_aux_function(const AuxFuncTrigger &trigger) override;

private:

    void do_aux_function_change_mode(const Mode::Number mode,
                                     const AuxSwitchPos ch_flag);
    void do_aux_function_change_air_mode(const AuxSwitchPos ch_flag);
    void do_aux_function_change_force_flying(const AuxSwitchPos ch_flag);

    // called when the mode switch changes position:
    void mode_switch_changed(modeswitch_pos_t new_pos) override;

};

class RC_Channels_Copter : public RC_Channels
{
public:

    bool has_valid_input() const override;
    bool in_rc_failsafe() const override;

    RC_Channel *get_arming_channel(void) const override;

    RC_Channel_Copter obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Copter *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    // returns true if throttle arming checks should be run
    bool arming_check_throttle() const override;

    // support for trimming AHRS using RC stick inputs, enabled via an
    // aux function
#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
    void do_aux_function_ahrs_auto_trim(const RC_Channel::AuxSwitchPos ch_flag);
    struct {
        bool running;
    } auto_trim;
    void auto_trim_run();
    void auto_trim_cancel();
#endif  // AP_COPTER_AHRS_AUTO_TRIM_ENABLED
    void save_trim();

protected:

    int8_t flight_mode_channel_number() const override;

};
