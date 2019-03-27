/*
 * AP_IRLock_SITL.h
 *
 *  Created on: June 09, 2016
 *      Author: Ian Chen
 */
#pragma once

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "IRLock.h"
#include <SITL/SITL.h>

class AP_IRLock_SITL : public IRLock
{
public:
    // init - initialize sensor library
    void init(int8_t bus) override;

    // retrieve latest sensor data - returns true if new data is available
    bool update() override;

private:
    SITL::SITL          *_sitl;                 // sitl instance pointer
    uint32_t _last_timestamp = 0;
};
#endif // CONFIG_HAL_BOARD
