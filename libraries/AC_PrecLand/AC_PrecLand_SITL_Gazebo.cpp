#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_SITL_Gazebo.h"

extern const AP_HAL::HAL& hal;

#if AC_PRECLAND_SITL_GAZEBO_ENABLED

// Constructor
AC_PrecLand_SITL_Gazebo::AC_PrecLand_SITL_Gazebo(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state),
      irlock()
{
}

// init - perform initialisation of this backend
void AC_PrecLand_SITL_Gazebo::init()
{
    irlock.init(get_bus());
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand_SITL_Gazebo::update()
{
    // update health
    _state.healthy = irlock.healthy();

    // get new sensor data
    irlock.update();

    if (irlock.num_targets() > 0 && irlock.last_update_ms() != _los_meas_time_ms) {
        irlock.get_unit_vector_body(_los_meas_body);
        _have_los_meas = true;
        _los_meas_time_ms = irlock.last_update_ms();
    }
    _have_los_meas = _have_los_meas && AP_HAL::millis()-_los_meas_time_ms <= 1000;
}

#endif  // AC_PRECLAND_SITL_GAZEBO_ENABLED
