#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_Companion.h"

// perform any required initialisation of backend
void AC_PrecLand_Companion::init()
{
    // set healthy
    _state.healthy = true;
    _have_los_meas = false;
}

// retrieve updates from sensor
void AC_PrecLand_Companion::update()
{
    _have_los_meas = _have_los_meas && AP_HAL::millis()-_los_meas_time_ms <= 1000;
}

// return distance to target
float AC_PrecLand_Companion::distance_to_target() const
{
    return _distance_to_target;
}

void AC_PrecLand_Companion::handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
    _distance_to_target = packet.distance;

    // compute unit vector towards target
    _los_meas_body = Vector3f(-tanf(packet.angle_y), tanf(packet.angle_x), 1.0f);
    _los_meas_body /= _los_meas_body.length();

    _los_meas_time_ms = timestamp_ms;
    _have_los_meas = true;
}
