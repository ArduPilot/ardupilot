#include "Tracker.h"

#include "RC_Channel.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Tracker
#define RC_CHANNEL_SUBCLASS RC_Channel_Tracker

#include <RC_Channel/RC_Channels_VarInfo.h>

// note that this callback is not presently used on Plane:
int8_t RC_Channels_Tracker::flight_mode_channel_number() const
{
    return 1; // tracker does not have a flight mode channel
}

// do_aux_function - implement the function invoked by auxiliary switches
void RC_Channel_Tracker::do_aux_function(const aux_func_t ch_option, const AuxSwitchPos ch_flag)
{
    switch(ch_option) {
       case AUX_FUNC::QUICK_TUNE:
            do_aux_function_quick_tune(ch_flag);
            break;

    default:
        RC_Channel::do_aux_function(ch_option, ch_flag);
        break;
    }
}

void RC_Channel_Tracker::do_aux_function_quick_tune(const AuxSwitchPos ch_flag)
{
    const char *axis_string = nullptr;
    AC_PID_OscillationDetector *detector = nullptr;
    switch(tracker.quick_tune_axis)
    {
        case AP_Vehicle::PID_AXIS::PITCH:
            axis_string = "Pitch";
            detector = &tracker.g.pidPitch2Srv.oscillationDetector;
            break;
        case AP_Vehicle::PID_AXIS::YAW:
            axis_string = "Yaw";
            detector = &tracker.g.pidYaw2Srv.oscillationDetector;
            break;
        default:
            break;
    }

    if (detector == nullptr) {
        // dont have this axis, look for the next
        if (tracker.advance_quick_tune_axis()) {
            do_aux_function_quick_tune(ch_flag);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Quick Tune: not enabled");
        }
        return;
    }

    tracker.quick_tune_start_stop(ch_flag,axis_string,detector);
}
