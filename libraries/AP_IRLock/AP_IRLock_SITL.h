/*
 * AP_IRLock_SITL.h
 *
 *  Created on: June 09, 2016
 *      Author: Ian Chen
 */
#pragma once

#include "AP_IRLock_config.h"

#if AP_IRLOCK_SITL_ENABLED

#include "AP_IRLock.h"
#include <SITL/SITL.h>

class AP_IRLock_SITL : public AP_IRLock
{
public:
    // init - initialize sensor library
    void init(int8_t bus) override;

    // retrieve latest sensor data - returns true if new data is available
    bool update() override;

private:
    SITL::SIM          *_sitl;                 // sitl instance pointer
    uint32_t _last_timestamp = 0;
};

#endif  // AP_IRLOCK_SITL_ENABLED
