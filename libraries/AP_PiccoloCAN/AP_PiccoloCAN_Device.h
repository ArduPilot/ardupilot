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

#include "AP_PiccoloCAN_config.h"


#if HAL_PICCOLO_CAN_ENABLE

//! Piccolo message groups form part of the CAN ID of each frame
enum class PiccoloCAN_MessageGroup : uint8_t {
    SIMULATOR = 0x00,       //!< Simulator messages
    SENSOR = 0x04,          //!< External sensors
    ACTUATOR = 0x07,        //!< Actuators (e.g. ESC / servo)
    ECU_OUT = 0x08,         //!< Messages *from* an ECU
    ECU_IN = 0x09,          //!< Message *to* an ECU

    BATTERY = 0x1A,         //!< Battery / generator messages
    SYSTEM = 0x19,          //!< System messages (e.g. bootloader)
};

//! Piccolo device type descriminators
enum class PiccoloCAN_DeviceType : uint8_t {
    SERVO = 0x00,
    ESC = 0x20,
    CORTEX_LEGACY=0x33,
    CORTEX_HYBRID=0x34,
    CORTEX_CHPS = 0x35,
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
    virtual bool is_connected(uint32_t timeout_us) const {
        uint64_t now = AP_HAL::micros64();

        return now < (last_msg_timestamp + timeout_us);
    }

    // Reset the received message timestamp
    void reset_rx_timestamp() {
        last_msg_timestamp = AP_HAL::micros64();
    }

    //! Timestamp of most recently received CAN message
    uint64_t last_msg_timestamp = 0;
};

#endif // HAL_PICCOLO_CAN_ENABLE
