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
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>

#ifndef HAL_PICCOLO_CAN_ENABLE
#define HAL_PICCOLO_CAN_ENABLE HAL_NUM_CAN_IFACES
#endif

#if HAL_PICCOLO_CAN_ENABLE

// Piccolo message groups form part of the CAN ID of each frame
enum class PiccoloCAN_MessageGroup : uint8_t {
    SIMULATOR = 0x00,       // Simulator messages
    SENSOR = 0x04,          // External sensors
    ACTUATOR = 0x07,        // Actuators (e.g. ESC / servo)
    ECU_OUT = 0x08,         // Messages *from* an ECU
    ECU_IN = 0x09,          // Message *to* an ECU

    SYSTEM = 0x19,          // System messages (e.g. bootloader)
};

// Piccolo actuator types differentiate between actuator frames
enum class PiccoloCAN_ActuatorType : uint8_t {
    SERVO = 0x00,
    ESC = 0x20,
};


/*
 * Generic PiccoloCAN device class implementation
 */

class AP_PiccoloCAN_Device
{
public:
    virtual bool handle_can_frame(AP_HAL::CANFrame &frame) = 0;

    // Determine if this device is "enabled" (default implementation returns false)
    virtual bool is_enabled(void) const { return false; }

    // Determine if this device has been seen within a specified timeframe
    virtual bool is_connected(int64_t timeout_ms) const {
        uint64_t now = AP_HAL::micros64();

        return now < (last_msg_timestamp + (1000ULL * timeout_ms));
    }

    // Reset the received message timestamp
    void reset_rx_timestamp() {
        last_msg_timestamp = AP_HAL::micros64();
    }

    //! Timestamp of most recently received CAN message
    uint64_t last_msg_timestamp = 0;
};

#endif // HAL_PICCOLO_CAN_ENABLE
