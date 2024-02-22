#pragma once

#include "AC_PrecLand_config.h"

#if AC_PRECLAND_ENABLED

#include "AC_PrecLand.h"
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>


class AC_PrecLand_Backend
{
public:
    // Constructor
    AC_PrecLand_Backend(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state) :
        _frontend(frontend),
        _state(state) {}

    // destructor
    virtual ~AC_PrecLand_Backend() {}

    // perform any required initialisation of backend
    virtual void init() = 0;

    // retrieve updates from sensor
    virtual void update() = 0;

    // provides a unit vector towards the target in body frame
    //  returns same as have_los_meas()
    bool get_los_body(Vector3f& dir_body) {
        if (have_los_meas()) {
            dir_body = _los_meas_body;
            return true;
        }
        return false;
    };

    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_time_ms() { return _los_meas_time_ms; };

    // return true if there is a valid los measurement available
    bool have_los_meas() { return _have_los_meas; };

    // returns distance to target in meters (0 means distance is not known)
    float distance_to_target() { return _distance_to_target; };

    // parses a mavlink message from the companion computer
    virtual void handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) {};

    // get bus parameter
    int8_t get_bus(void) const { return _frontend._bus.get(); }
    
protected:
    const AC_PrecLand&  _frontend;          // reference to precision landing front end
    AC_PrecLand::precland_state &_state;    // reference to this instances state

    Vector3f            _los_meas_body;         // unit vector in body frame pointing towards target
    uint32_t            _los_meas_time_ms;      // system time in milliseconds when los was measured
    bool                _have_los_meas;         // true if there is a valid measurement from the sensor
    float               _distance_to_target;    // distance from the sensor to landing target in meters
};

#endif // AC_PRECLAND_ENABLED
