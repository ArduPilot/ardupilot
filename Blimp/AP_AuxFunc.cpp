#include "Blimp.h"

#include "AP_AuxFunc.h"

// init_function_switch_function - initialize aux functions
bool AP_AuxFunc_Blimp::init_function(const Function function, const SwitchPos pos)
{
    // init channel options
    switch (function) {
    // the following functions do not need to be initialised:
    case Function::MANUAL:
    case Function::LOITER:
        return true;
    default:
        return AP_AuxFunc::init_function(function, pos);
    }
}

// do_function_change_mode - change mode based on an aux switch
// being moved
void AP_AuxFunc_Blimp::do_function_change_mode(const Mode::Number mode,
        const SwitchPos pos)
{
    switch (pos) {
    case SwitchPos::HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        const bool success = blimp.set_mode(mode, ModeReason::RC_COMMAND);
        if (blimp.ap.initialised) {
            if (success) {
                AP_Notify::events.user_mode_change = 1;
            } else {
                AP_Notify::events.user_mode_change_failed = 1;
            }
        }
        break;
    }
    default:
        // return to flight mode switch's flight mode if we are currently
        // in this mode
        if (blimp.control_mode == mode) {
            rc().reset_mode_switch();
        }
    }
}

// do_function - implement the function invoked by auxiliary switches
bool AP_AuxFunc_Blimp::do_function(const Function function, const SwitchPos pos)
{
    switch (function) {

    case Function::SAVE_TRIM:
        if ((pos == SwitchPos::HIGH) &&
            (blimp.control_mode <= Mode::Number::MANUAL) &&
            (blimp.channel_down->get_control_in() == 0)) {
            blimp.save_trim();
        }
        break;

    case Function::MANUAL:
        do_function_change_mode(Mode::Number::MANUAL, pos);
        break;

    case Function::LOITER:
        do_function_change_mode(Mode::Number::LOITER, pos);
        break;

    default:
        return AP_AuxFunc::do_function(function, pos);
    }

    return true;
}
