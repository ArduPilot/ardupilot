#include "Copter.h"

#if MODE_GUIDED_ENABLED && AP_SCRIPTING_ENABLED
// constructor registers custom number and names
ModeGuidedCustom::ModeGuidedCustom(const Number _number, const char* _full_name, const char* _short_name):
    number(_number),
    full_name(_full_name),
    short_name(_short_name)
{
}

bool ModeGuidedCustom::init(bool ignore_checks)
{
    // Stript can block entry
    if (!state.allow_entry) {
        if (state.entry_denied_reason != nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s", state.entry_denied_reason);
        }
        return false;
    }

    if (state.angle_only) {
        ModeGuided::angle_control_start();
        return true;
    } else {
        // Guided entry checks must also pass
        return ModeGuided::init(ignore_checks);
    }
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void ModeGuidedCustom::run()
{
    if (state.angle_only) {
        // run angle controller
        ModeGuided::angle_control_run();
    } else {
        ModeGuided::run();
    }
}

#endif
