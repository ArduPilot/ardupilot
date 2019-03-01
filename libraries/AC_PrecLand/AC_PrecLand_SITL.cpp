#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand_SITL.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_AHRS/AP_AHRS.h"
// init - perform initialisation of this backend
void AC_PrecLand_SITL::init()
{
    _sitl = AP::sitl();
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand_SITL::update()
{
    _state.healthy = _sitl->precland_sim.healthy();

    if (_state.healthy && _sitl->precland_sim.last_update_ms() != _los_meas_time_ms) {
        const Vector3f position = _sitl->precland_sim.get_target_position();
        const Matrix3f &body_to_ned = AP::ahrs().get_rotation_body_to_ned();
        _los_meas_body =  body_to_ned.mul_transpose(-position);
        _los_meas_body /= _los_meas_body.length();
        _have_los_meas = true;
        _los_meas_time_ms = _sitl->precland_sim.last_update_ms();
    } else {
        _have_los_meas = false;
    }

    _have_los_meas = _have_los_meas && AP_HAL::millis() - _los_meas_time_ms <= 1000;
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
