#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_PrecLand/AC_PrecLand_Backend.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
 #include <AP_IRLock/AP_IRLock_SITL.h>
#else
 #include <AP_IRLock/AP_IRLock.h>
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_IRLock/AP_IRLock_SITL.h>
#endif

/*
 * AC_PrecLand_IRLock - implements precision landing using target vectors provided
 *                         by a companion computer (i.e. Odroid) communicating via MAVLink
 */

class AC_PrecLand_IRLock : public AC_PrecLand_Backend
{
public:

    // Constructor
    AC_PrecLand_IRLock(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state);

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates from sensor
    void update() override;

    // provides a unit vector towards the target in body frame
    //  returns same as have_los_meas()
    bool get_los_body(Vector3f& ret) override;

    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_time_ms() override;

    // return true if there is a valid los measurement available
    bool have_los_meas() override;

private:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_IRLock_SITL irlock;
#else
    AP_IRLock_I2C irlock;
#endif
    Vector3f            _los_meas_body;         // unit vector in body frame pointing towards target
    bool                _have_los_meas;         // true if there is a valid measurement from the camera
    uint32_t            _los_meas_time_ms;      // system time in milliseconds when los was measured
};
