#pragma once

#include <AP_Math/AP_Math.h>
#include "AC_PrecLand_Backend.h"

/*
 * AC_PrecLand_Companion - implements precision landing using target vectors provided
 *                         by a companion computer (i.e. Odroid) communicating via MAVLink
 *                         The companion computer must provide "Line-Of-Sight" measurements
 *                         in the form of LANDING_TARGET mavlink messages.
 */

class AC_PrecLand_Companion : public AC_PrecLand_Backend
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
    uint32_t los_meas_time_ms() override;

    // return true if there is a valid los measurement available
    bool have_los_meas() override;

    // returns distance to target in meters (0 means distance is not known)
    float distance_to_target() override;

    // parses a mavlink message from the companion computer
    void handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) override;

private:
    float               _distance_to_target;    // distance from the camera to target in meters

    Vector3f            _los_meas_body;         // unit vector in body frame pointing towards target
    bool                _have_los_meas;         // true if there is a valid measurement from the camera
    uint32_t            _los_meas_time_ms;      // system time in milliseconds when los was measured
};
