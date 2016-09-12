/*
 * AP_IRLock_PX4.h
 *
 *  Created on: Nov 10, 2014
 *      Author: MLandes
 */
#pragma once

#include "IRLock.h"

class AP_IRLock_PX4 : public IRLock
{
public:
    AP_IRLock_PX4();

    // init - initialize sensor library
    virtual void init();

    // retrieve latest sensor data - returns true if new data is available
    virtual bool update();

private:
    int _fd;
};
