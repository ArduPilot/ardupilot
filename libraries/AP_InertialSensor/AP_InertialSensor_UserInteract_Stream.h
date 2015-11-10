
#ifndef __AP_INERTIAL_SENSOR_USER_INTERACT_STREAM_H__
#define __AP_INERTIAL_SENSOR_USER_INTERACT_STREAM_H__

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_UserInteract.h"

/**
 * AP_InertialSensor_UserInteract, implemented in terms of a BetterStream.
 */
class AP_InertialSensor_UserInteractStream :
    public AP_InertialSensor_UserInteract {
public:
    AP_InertialSensor_UserInteractStream(AP_HAL::BetterStream *s) :
        _s(s) {}

    bool blocking_read();
    void printf(const char*, ...) FMT_PRINTF(2, 3);
private:
    AP_HAL::BetterStream *_s;
};

#endif // __AP_INERTIAL_SENSOR_USER_INTERACT_STREAM_H__

