#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AC_PrecLand_Backend.h"

/*
 * AC_PrecLand_Companion - implements precision landing using target vectors provided
 *                         by a companion computer (i.e. Odroid) communicating via MAVLink
 */

class AC_PrecLand_Companion : public AC_PrecLand_Backend
{
public:
    // Constructor
    AC_PrecLand_Companion(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state);
    
    // perform any required initialisation of backend
    void init();

    // retrieve updates from sensor
    void update();

    // provides a unit vector towards the target in body frame
    //  returns same as have_los_meas()
    bool get_los_body(Vector3f& ret);
    
    // returns system time in milliseconds of last los measurement
    uint32_t los_meas_time_ms();
    
    // return true if there is a valid los measurement available
    bool have_los_meas();
    
    // returns distance to target in meters (0 means distance is not known)
    float distance_to_target() override;

    // parses a mavlink message from the companion computer
    void handle_msg(mavlink_message_t* msg);

private:
    uint64_t            _timestamp_us;          // timestamp from message
    float               _distance_to_target;    // distance from the camera to target in meters
    
    Vector3f            _los_meas_body;         // unit vector in body frame pointing towards target
    bool                _have_los_meas;         // true if there is a valid measurement from the camera
    uint32_t            _los_meas_time_ms;      // system time in milliseconds when los was measured
};
