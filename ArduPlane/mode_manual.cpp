#include "mode.h"
#include "Plane.h"

void ModeManual::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.roll_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    plane.steering_control.steering = plane.steering_control.rudder = plane.rudder_in_expo(false);

    plane.nav_roll_cd = ahrs.roll_sensor;
    plane.nav_pitch_cd = ahrs.pitch_sensor;
}

