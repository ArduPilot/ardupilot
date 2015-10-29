/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_IRLock.h"

extern const AP_HAL::HAL& hal;

// this only builds for PX4 so far
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

// Constructor
AC_PrecLand_IRLock::AC_PrecLand_IRLock(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state),
      irlock()
{
}

// init - perform initialisation of this backend
void AC_PrecLand_IRLock::init()
{
    irlock.init();

    // set healthy
    _state.healthy = irlock.healthy();
}

// update - give chance to driver to get updates from sensor
bool AC_PrecLand_IRLock::update()
{
    // get new sensor data
    return (irlock.update());
}

// get_angle_to_target - returns body frame angles (in radians) to target
//  returns true if angles are available, false if not (i.e. no target)
//  x_angle_rad : body-frame roll direction, positive = target is to right (looking down)
//  y_angle_rad : body-frame pitch direction, postiive = target is forward (looking down)
bool AC_PrecLand_IRLock::get_angle_to_target(float &x_angle_rad, float &y_angle_rad)
{
    return irlock.get_angle_to_target(x_angle_rad, y_angle_rad);
}

#endif // PX4
