#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_SITL.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <stdio.h>

// Constructor
AC_PrecLand_SITL::AC_PrecLand_SITL(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state)
{
}

// init - perform initialisation of this backend
void AC_PrecLand_SITL::init()
{
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand_SITL::update()
{
    const uint32_t now = AP_HAL::millis();
    if (_los_meas_time_ms + 10 > now) { // 100Hz update
        return;
    }

    // get new sensor data; we always point home
    Vector3f home;
    if (! _frontend._ahrs.get_relative_position_NED_home(home)) {
        _state.healthy = false;
        return;
    }
    if (home.length() > 10.0f) { // we can see the target out to 10 metres
        return;
    }
    _state.healthy = true;

    const Matrix3f &body_to_ned = _frontend._ahrs.get_rotation_body_to_ned();

    _los_meas_body =  body_to_ned.mul_transpose(-home);
    _los_meas_body /= _los_meas_body.length();
    _los_meas_time_ms = now;
}

bool AC_PrecLand_SITL::have_los_meas() {
    return AP_HAL::millis() - _los_meas_time_ms < 1000;
}


// provides a unit vector towards the target in body frame
//  returns same as have_los_meas()
bool AC_PrecLand_SITL::get_los_body(Vector3f& ret) {
    if (AP_HAL::millis() - _los_meas_time_ms > 1000) {
        // no measurement for a full second; no vector available
        return false;
    }
    ret = _los_meas_body;
    return true;
}

#endif
