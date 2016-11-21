/*
 * AP_IRLock_SITL.h
 *
 *  Created on: June 09, 2016
 *      Author: Ian Chen
 */
#pragma once

#include <AP_HAL/utility/Socket.h>

#include "IRLock.h"

class AP_IRLock_SITL : public IRLock
{
public:
    AP_IRLock_SITL();

    // init - initialize sensor library
    virtual void init();

    // retrieve latest sensor data - returns true if new data is available
    virtual bool update();

private:

    /*
      reply packet sent from simulator to ArduPilot
     */
    struct irlock_packet {
            uint64_t timestamp;
            uint16_t num_targets;
            float pos_x;
            float pos_y;
            float size_x;
            float size_y;
          };

    uint64_t _last_timestamp;
    SocketAPM sock;
};
