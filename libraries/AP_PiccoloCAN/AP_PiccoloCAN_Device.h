/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */

#pragma once

#include <stdint.h>

#include "piccolo_protocol/ESCPackets.h"
#include "piccolo_protocol/ServoPackets.h"

#ifndef HAL_PICCOLO_CAN_ENABLE
#define HAL_PICCOLO_CAN_ENABLE (HAL_NUM_CAN_IFACES && !HAL_MINIMIZE_FEATURES)
#endif

/**
 * Class definition for a "base" PiccoloCAN device.
 * Other PiccoloCAN devices (such as servos / ESCs) will inherit from this
 */

class AP_PiccoloCAN_Device
{
public:

    bool is_connected(uint64_t timeout_ms = 2000);

    bool newTelemetryAvailable = false;
    uint64_t last_rx_msg_timestamp = 0;    //! Time of most recently received message
};
