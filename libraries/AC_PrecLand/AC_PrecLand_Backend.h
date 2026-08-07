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
    bool get_los_meas(Vector3f& vec_unit, AC_PrecLand::VectorFrame& frame) const {
        if (!_los_meas.valid) {
            return false;
        }
        vec_unit = _los_meas.vec_unit;
        frame = _los_meas.frame;
        return true;
    };

    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_time_ms() const { return _los_meas.time_ms; };

    // returns distance to target in meters (0 means distance is not known)
    float distance_to_target() const { return _distance_to_target; };

    // parses a mavlink message from the companion computer
    virtual void handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) {};

    // get bus parameter
    int8_t get_bus(void) const { return _frontend._bus.get(); }
    
protected:
    const AC_PrecLand&  _frontend;          // reference to precision landing front end
    AC_PrecLand::precland_state &_state;    // reference to this instances state

    struct {
        Vector3f vec_unit;  // unit vector pointing towards target in earth or body frame (see frame)
        AC_PrecLand::VectorFrame frame;  // frame of vector pointing towards target
        uint32_t time_ms;   // system time in milliseconds when the vector was measured
        bool valid;         // true if there is a valid measurement from the sensor
    } _los_meas;
    float               _distance_to_target;    // distance from the sensor to landing target in meters
};

#endif // AC_PRECLAND_ENABLED
