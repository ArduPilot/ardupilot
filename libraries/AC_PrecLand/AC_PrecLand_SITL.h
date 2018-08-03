#pragma once

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_PrecLand/AC_PrecLand_Backend.h>
#include <SITL/SITL.h>

/*
 * AC_PrecLand_SITL - supplies vectors to a fake landing target
 */

class AC_PrecLand_SITL : public AC_PrecLand_Backend
{
public:

    // Constructor
    using AC_PrecLand_Backend::AC_PrecLand_Backend;

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates from sensor
    void update() override;

    // provides a unit vector towards the target in body frame
    //  returns same as have_los_meas()
    bool get_los_body(Vector3f& ret) override;

    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_time_ms() override { return _los_meas_time_ms; }

    // return true if there is a valid los measurement available
    bool have_los_meas() override;

private:
    SITL::SITL          *_sitl;                 // sitl instance pointer
    Vector3f            _los_meas_body;         // unit vector in body frame pointing towards target
    uint32_t            _los_meas_time_ms;      // system time in milliseconds when los was measured
    bool                _have_los_meas;         // true if there is a valid measurement from the camera
};

#endif
