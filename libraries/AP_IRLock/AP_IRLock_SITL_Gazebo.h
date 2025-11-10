/*
 * AP_IRLock_SITL.h
 *
 *  Created on: June 09, 2016
 *      Author: Ian Chen
 */
#pragma once

#include "AP_IRLock_config.h"

#if AP_IRLOCK_SITL_GAZEBO_ENABLED

#include <AP_HAL/utility/Socket_native.h>
#include "AP_IRLock.h"

class AP_IRLock_SITL_Gazebo : public AP_IRLock
{
public:
    AP_IRLock_SITL_Gazebo();

    // init - initialize sensor library
    void init(int8_t bus) override;

    // retrieve latest sensor data - returns true if new data is available
    bool update() override;

private:

    uint32_t _last_timestamp;
    SocketAPM_native sock;
};

#endif  // AP_IRLOCK_SITL_GAZEBO_ENABLED
