#include "Plane.h"

#include "RC_Channel.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Plane
#define RC_CHANNEL_SUBCLASS RC_Channel_Plane

#include <RC_Channel/RC_Channels_VarInfo.h>

// note that this callback is not presently used on Plane:
int8_t RC_Channels_Plane::flight_mode_channel_number() const
{
    return plane.g.flight_mode_channel.get();
}

bool RC_Channels_Plane::has_valid_input() const
{
    if (plane.failsafe.rc_failsafe) {
        return false;
    }
    if (plane.failsafe.throttle_counter != 0) {
        return false;
    }
    return true;
}

void RC_Channel_Plane::init_aux_function(const RC_Channel::aux_func_t ch_option,
                                         const RC_Channel::aux_switch_pos_t ch_flag)
{
    switch(ch_option) {
    // the following functions do not need to be initialised:
    case ARMDISARM:
    case INVERTED:
        break;

    case REVERSE_THROTTLE:
        plane.have_reverse_throttle_rc_option = true;
        // setup input throttle as a range. This is needed as init_aux_function is called
        // after set_control_channels()
        if (plane.channel_throttle) {
            plane.channel_throttle->set_range(100);
        }
        // note that we don't call do_aux_function() here as we don't
        // want to startup with reverse thrust
        break;

    default:
        // handle in parent class
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
}
}

// do_aux_function - implement the function invoked by auxillary switches
void RC_Channel_Plane::do_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
{
    switch(ch_option) {
    case ARMDISARM:
        // arm or disarm the vehicle
        switch (ch_flag) {
        case HIGH:
            plane.arm_motors(AP_Arming::ArmingMethod::AUXSWITCH, true);
            break;
        case MIDDLE:
            // nothing
            break;
        case LOW:
            plane.disarm_motors();
            break;
        }
        break;
    case INVERTED:
        plane.inverted_flight = (ch_flag == HIGH);
        break;

    case REVERSE_THROTTLE:
        plane.reversed_throttle = (ch_flag == HIGH);
        gcs().send_text(MAV_SEVERITY_INFO, "RevThrottle: %s", plane.reversed_throttle?"ENABLE":"DISABLE");
        break;
        
    default:
        RC_Channel::do_aux_function(ch_option, ch_flag);
        break;
    }
}
