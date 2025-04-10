#include "Sub.h"

#include "RC_Channel_Sub.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Sub
#define RC_CHANNEL_SUBCLASS RC_Channel_Sub

#include <RC_Channel/RC_Channels_VarInfo.h>


#if RC_ENABLED
int8_t RC_Channels_Sub::flight_mode_channel_number() const
{
    return sub.g.flight_mode_chan.get();
}

void RC_Channel_Sub::mode_switch_changed(modeswitch_pos_t new_pos)
{
    if (new_pos < 0 || new_pos > 6) {
        // should not have been called
        return;
    }

    if (!sub.set_mode((Mode::Number)sub.flight_modes[new_pos].get(), ModeReason::RC_COMMAND)) {
        return;
    }
}

// init_aux_switch_function - initialize aux functions
void RC_Channel_Sub::init_aux_function(const AUX_FUNC ch_option, const AuxSwitchPos ch_flag)
{
RC_Channel::init_aux_function(ch_option, ch_flag);
}

bool RC_Channels_Sub::in_rc_failsafe() const
{
    return sub.failsafe.radio;
}

bool RC_Channels_Sub::has_valid_input() const
{
    if (in_rc_failsafe()) {
        return false;
    }
    if (sub.failsafe.radio_counter != 0) {
        return false;
    }
    return true;
}


// do_aux_function - implement the function invoked by auxiliary switches
bool RC_Channel_Sub::do_aux_function(const AuxFuncTrigger &trigger)
{
   return RC_Channel::do_aux_function(trigger);
}
#endif
