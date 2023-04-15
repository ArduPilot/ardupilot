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
    CANSensor(const char *driver_name, uint16_t stack_size=2048);

    /* Do not allow copies */
    CLASS_NO_COPY(CANSensor);

    void init(uint8_t driver_index, bool enable_filters) override;
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    // handler for incoming frames
    virtual void handle_frame(AP_HAL::CANFrame &frame) = 0;

    // handler for outgoing frames
    bool write_frame(AP_HAL::CANFrame &out_frame, const uint64_t timeout_us);

#ifdef HAL_BUILD_AP_PERIPH
    static void set_periph(const uint8_t i, const AP_CANManager::Driver_Type protocol, AP_HAL::CANIface* iface) {
        if (i < ARRAY_SIZE(_periph)) {
            _periph[i].protocol = protocol;
            _periph[i].iface = iface;
        }
    }

    // return driver type index i
    static AP_CANManager::Driver_Type get_driver_type(const uint8_t i)
    {
        if (i < ARRAY_SIZE(_periph)) {
            return _periph[i].protocol;
        }
        return AP_CANManager::Driver_Type_None;
    }
#else
    static AP_CANManager::Driver_Type get_driver_type(const uint8_t i) { return AP::can().get_driver_type(i); }
#endif

protected:
    void register_driver(AP_CANManager::Driver_Type dtype);

private:
    void loop();

    const char *const _driver_name;
    const uint16_t _stack_size;
    bool _initialized;
    uint8_t _driver_index;
    AP_CANDriver *_can_driver;
    HAL_EventHandle _event_handle;
    AP_HAL::CANIface* _can_iface;

#ifdef HAL_BUILD_AP_PERIPH
    void register_driver_periph(const AP_CANManager::Driver_Type dtype);
    
    struct CANSensor_Periph {
        AP_HAL::CANIface* iface;
        AP_CANManager::Driver_Type protocol;
    } static _periph[HAL_NUM_CAN_IFACES];
#endif
};

#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

