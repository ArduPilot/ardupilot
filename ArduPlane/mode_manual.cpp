#include "mode.h"
#include "Plane.h"

void ModeManual::_exit()
{
    if (plane.g.auto_trim > 0) {
        plane.trim_radio();
    }
}

void ModeManual::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.roll_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    plane.steering_control.steering = plane.steering_control.rudder = plane.rudder_in_expo(false);
}

