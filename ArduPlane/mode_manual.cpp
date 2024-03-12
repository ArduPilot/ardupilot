#include "mode.h"
#include "Plane.h"

void ModeManual::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.roll_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    output_rudder_and_steering(plane.rudder_in_expo(false));
    float throttle = plane.get_throttle_input(true);


#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() && quadplane.option_is_set(QuadPlane::OPTION::IDLE_GOV_MANUAL)) {
        // for quadplanes it can be useful to run the idle governor in MANUAL mode
        // as it prevents the VTOL motors from running
        int8_t min_throttle = plane.aparm.throttle_min.get();

        // apply idle governor
#if AP_ICENGINE_ENABLED
        plane.g2.ice_control.update_idle_governor(min_throttle);
#endif
        throttle = MAX(throttle, min_throttle);
    }
#endif
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

    plane.nav_roll_cd = ahrs.roll_sensor;
    plane.nav_pitch_cd = ahrs.pitch_sensor;
}

void ModeManual::run()
{
    reset_controllers();
}
