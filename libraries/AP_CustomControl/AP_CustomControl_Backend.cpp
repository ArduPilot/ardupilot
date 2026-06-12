#include "AP_CustomControl/AP_CustomControl_Backend.h"

#if AP_CUSTOMCONTROL_ENABLED

void AP_CustomControl_Backend::update_rcin_channels() {
    channel_roll = &rc().get_roll_channel();
    channel_pitch = &rc().get_pitch_channel();
    channel_throttle = &rc().get_throttle_channel();
    channel_rudder = &rc().get_yaw_channel();
    channel_flap = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FLAP);
    channel_airbrake = rc().find_channel_for_option(RC_Channel::AUX_FUNC::AIRBRAKE);
}

#endif // AP_CUSTOMCONTROL_ENABLED
