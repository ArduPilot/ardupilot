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

// provides a unit vector towards the target in body frame
//  returns same as have_los_meas()
bool AC_PrecLand_Companion::get_los_body(Vector3f& ret) {
    if (have_los_meas()) {
        ret = _los_meas_body;
        return true;
    }
    return false;
}

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_Companion::los_meas_time_ms() {
    return _los_meas_time_ms;
}

// return true if there is a valid los measurement available
bool AC_PrecLand_Companion::have_los_meas()
{
    return _have_los_meas;
}

// return distance to target
float AC_PrecLand_Companion::distance_to_target()
{
    return _distance_to_target;
}

void AC_PrecLand_Companion::handle_msg(const mavlink_message_t &msg)
{
    // parse mavlink message
    __mavlink_landing_target_t packet;
    mavlink_msg_landing_target_decode(&msg, &packet);

    _timestamp_us = packet.time_usec;
    _distance_to_target = packet.distance;

    // compute unit vector towards target
    _los_meas_body = Vector3f(-tanf(packet.angle_y), tanf(packet.angle_x), 1.0f);
    _los_meas_body /= _los_meas_body.length();

    _los_meas_time_ms = AP_HAL::millis();
    _have_los_meas = true;
}
