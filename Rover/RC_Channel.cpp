#include "Rover.h"

#include "RC_Channel.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Rover
#define RC_CHANNEL_SUBCLASS RC_Channel_Rover

#include <RC_Channel/RC_Channels_VarInfo.h>

int8_t RC_Channels_Rover::flight_mode_channel_number() const
{
    return rover.g.mode_channel;
}

void RC_Channel_Rover::mode_switch_changed(modeswitch_pos_t new_pos)
{
    if (new_pos < 0 || (uint8_t)new_pos > rover.num_modes) {
        // should not have been called
        return;
    }
    Mode *new_mode = rover.mode_from_mode_num((Mode::Number)rover.modes[new_pos].get());
    if (new_mode != nullptr) {
        rover.set_mode(*new_mode, ModeReason::RC_COMMAND);
    }
}
bool RC_Channels_Rover::has_valid_input() const
{
    if (rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
        return false;
    }
    return true;
}

RC_Channel * RC_Channels_Rover::get_arming_channel(void) const
{
    return rover.channel_steer;
}
