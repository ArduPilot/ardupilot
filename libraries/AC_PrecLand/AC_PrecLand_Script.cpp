#include "AC_PrecLand_config.h"

#if AC_PRECLAND_SCRIPT_ENABLED

#include "AC_PrecLand_Script.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

// perform any required initialisation of backend
void AC_PrecLand_Script::init()
{
    // set healthy
    _state.healthy = true;
    _have_los_meas = false;
}

// retrieve updates from sensor
void AC_PrecLand_Script::update()
{
    _have_los_meas = AP_HAL::millis()-_los_meas_time_ms <= 1000;
}

// provides a unit vector towards the target in body frame
//  returns same as have_los_meas()
bool AC_PrecLand_Script::get_los_body(Vector3f& ret)
{
    if (have_los_meas()) {
        ret = _los_meas_body;
        return true;
    }
    return false;
}

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_Script::los_meas_time_ms()
{
    return _los_meas_time_ms;
}

// return true if there is a valid los measurement available
bool AC_PrecLand_Script::have_los_meas()
{
    return _have_los_meas;
}

// return distance to target
float AC_PrecLand_Script::distance_to_target()
{
    return _distance_to_target;
}

bool AC_PrecLand_Script::set_target_location(const Location &location)
{
    const Matrix3f body_to_ned = AP::ahrs().get_rotation_body_to_ned();

    Location body_location;
    if (!AP::ahrs().get_location(body_location)) { // If current location is unknown, return
        return false;
    }
    body_location.change_alt_frame(location.get_alt_frame()); // the alt frame does not matter if it is relative

    const Vector3f body_to_target = -(location.get_distance_NED(body_location));
    _los_meas_body = body_to_ned.mul_transpose(body_to_target);
    
    _distance_to_target = _los_meas_body.length();
    _los_meas_body /= _distance_to_target;

    _los_meas_time_ms = AP_HAL::millis();
    return true;
}

//bool AC_PrecLand_Script::set

#endif // AC_PRECLAND_SCRIPT_ENABLED
