#include "mode.h"
#include "Plane.h"

void ModeManual::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.roll_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    output_rudder_and_steering(plane.rudder_in_expo(false));

    const float throttle = plane.get_throttle_input(true);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

    plane.nav_roll_cd = ahrs.roll_sensor;
    plane.nav_pitch_cd = ahrs.pitch_sensor;
}

void ModeManual::run()
{
    reset_controllers();
}

// true if throttle min/max limits should be applied
bool ModeManual::use_throttle_limits() const
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() && quadplane.option_is_set(QuadPlane::Option::IDLE_GOV_MANUAL)) {
        return true;
    }
#endif
    return false;
}
