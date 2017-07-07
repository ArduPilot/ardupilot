/*
 * AP_IRLock_SITL.h
 *
 *  Created on: June 09, 2016
 *      Author: Ian Chen
 */
#pragma once

#include <AP_HAL/utility/Socket.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "IRLock.h"

class AP_IRLock_SITL : public IRLock
{
public:
    AP_IRLock_SITL();

    // init - initialize sensor library
    void init(int8_t bus) override;

    // retrieve latest sensor data - returns true if new data is available
    bool update() override;

private:

    /*
      reply packet sent from simulator to ArduPilot
     */
    struct irlock_packet {
        uint64_t timestamp;  // in miliseconds
        uint16_t num_targets;
        float pos_x;
        float pos_y;
        float size_x;
        float size_y;
    };

    uint32_t _last_timestamp;
    SocketAPM sock;
};
#endif // CONFIG_HAL_BOARD
