/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_Companion.h"

extern const AP_HAL::HAL& hal;

// Constructor
AC_PrecLand_Companion::AC_PrecLand_Companion(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state),
      _frame(MAV_FRAME_BODY_NED),
      _distance_to_target(0.0f),
      _timestamp_us(0),
      _new_estimate(false)
{
}

// init - perform initialisation of this backend
void AC_PrecLand_Companion::init()
{
    // set healthy
    _state.healthy = true;
    _new_estimate = false;
}

// update - give chance to driver to get updates from sensor
//  returns true if new data available
bool AC_PrecLand_Companion::update()
{
    // Mavlink commands are received asynchronous so all new data is processed by handle_msg()
    return _new_estimate;
}

MAV_FRAME AC_PrecLand_Companion::get_frame_of_reference()
{
    return _frame;
}

// get_angle_to_target - returns angles (in radians) to target
//  returns true if angles are available, false if not (i.e. no target)
//  x_angle_rad : roll direction, positive = target is to right (looking down)
//  y_angle_rad : pitch direction, postiive = target is forward (looking down)
bool AC_PrecLand_Companion::get_angle_to_target(float &x_angle_rad, float &y_angle_rad)
{
    if (_new_estimate){
        x_angle_rad = _angle_to_target.x;
        y_angle_rad = _angle_to_target.y;

        // reset and wait for new data
        _new_estimate = false;
        return true;
    }

    return false;
}

void AC_PrecLand_Companion::handle_msg(mavlink_message_t* msg)
{
    // parse mavlink message
    __mavlink_landing_target_t packet;
    mavlink_msg_landing_target_decode(msg, &packet);

    _timestamp_us = packet.time_usec;
    _frame = (MAV_FRAME) packet.frame;
    _angle_to_target.x = packet.angle_x;
    _angle_to_target.y = packet.angle_y;
    _distance_to_target = packet.distance;
    _state.healthy = true;
    _new_estimate = true;
}
