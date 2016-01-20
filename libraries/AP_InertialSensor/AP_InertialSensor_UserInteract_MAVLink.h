#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>


#include "AP_InertialSensor_UserInteract.h"

class GCS_MAVLINK;

/**
 * AP_InertialSensor_UserInteract, implemented in terms of a MAVLink connection
 */
class AP_InertialSensor_UserInteract_MAVLink : public AP_InertialSensor_UserInteract {
public:
    AP_InertialSensor_UserInteract_MAVLink(GCS_MAVLINK *gcs) :
    _gcs(gcs) {}

    bool blocking_read();
    void printf(const char *, ...) FMT_PRINTF(2, 3);
private:
    GCS_MAVLINK *_gcs;
};
