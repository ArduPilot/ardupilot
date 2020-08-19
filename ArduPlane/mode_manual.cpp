#include "mode.h"
#include "Plane.h"

bool ModeManual::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    return true;
}

void ModeManual::_exit()
{
    if (plane.g.auto_trim > 0) {
        plane.trim_radio();
    }
}

void ModeManual::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
    plane.steering_control.steering = plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();
}

