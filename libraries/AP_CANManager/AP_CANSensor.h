/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  CANSensor class, for easy creation of CAN sensors using custom CAN protocols
 */
 
#pragma once

#include "AP_CANManager.h"

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

class CANSensor : public AP_CANDriver {
public:
    CANSensor(const char *driver_name, AP_CANManager::Driver_Type dtype, uint16_t stack_size=2048);
    
    /* Do not allow copies */
    CANSensor(const CANSensor &other) = delete;
    CANSensor &operator=(const CANSensor&) = delete;

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // handler for incoming frames
    virtual void handle_frame(AP_HAL::CANFrame &frame) = 0;

private:
    void loop();

    const char *const _driver_name;
    const uint16_t _stack_size;
    bool _initialized;
    uint8_t _driver_index;
    AP_CANDriver *_can_driver;
    HAL_EventHandle _event_handle;
    AP_HAL::CANIface* _can_iface;
};

#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

