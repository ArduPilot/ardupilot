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

#include "AP_CAN.h"
#include "AP_CANDriver.h"
#ifndef HAL_BUILD_AP_PERIPH
#include "AP_CANManager.h"
#endif

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
    static void set_periph(const uint8_t i, const AP_CAN::Protocol protocol, AP_HAL::CANIface* iface) {
        if (i < ARRAY_SIZE(_periph)) {
            _periph[i].protocol = protocol;
            _periph[i].iface = iface;
        }
    }

    // return driver type index i
    static AP_CAN::Protocol get_driver_type(const uint8_t i)
    {
        if (i < ARRAY_SIZE(_periph)) {
            return _periph[i].protocol;
        }
        return AP_CAN::Protocol::None;
    }
#else
    static AP_CAN::Protocol get_driver_type(const uint8_t i) { return AP::can().get_driver_type(i); }
#endif

protected:
    void register_driver(AP_CAN::Protocol dtype);

private:
    void loop();

    const char *const _driver_name;
    const uint16_t _stack_size;
    bool _initialized;
    uint8_t _driver_index;

    // this is true when we are setup as an auxillary driver using CAN_Dn_PROTOCOL2
    bool is_aux_11bit_driver;

    AP_CANDriver *_can_driver;
    HAL_BinarySemaphore sem_handle;
    AP_HAL::CANIface* _can_iface;

#ifdef HAL_BUILD_AP_PERIPH
    void register_driver_periph(const AP_CAN::Protocol dtype);
    
    struct CANSensor_Periph {
        AP_HAL::CANIface* iface;
        AP_CAN::Protocol protocol;
    } static _periph[HAL_NUM_CAN_IFACES];
#endif
};

// a class to allow for multiple CAN backends with one
// CANSensor driver. This can be shared among different libraries like rangefinder and proximity
class MultiCAN : public CANSensor {
public:
    // callback functor def for forwarding frames
    FUNCTOR_TYPEDEF(ForwardCanFrame, bool, AP_HAL::CANFrame &);

    MultiCAN(ForwardCanFrame cf, AP_CAN::Protocol can_type, const char *driver_name);

    // handle a received frame from the CAN bus
    void handle_frame(AP_HAL::CANFrame &frame) override;

private:
    // class to allow for multiple callbacks implemented as a linked list
    class MultiCANLinkedList {
    public:
        struct CANSensor_Multi {
            ForwardCanFrame _callback;
            CANSensor_Multi* next = nullptr;
        };

        // register a callback for a CAN frame by adding it to the linked list
        void register_callback(ForwardCanFrame callback);

        // distribute the CAN frame to the registered callbacks
        void handle_frame(AP_HAL::CANFrame &frame);
        HAL_Semaphore sem;

    private:
        CANSensor_Multi* head = nullptr;
    };

    // Pointer to static instance of the linked list for persistence across instances
    static MultiCANLinkedList* callbacks;
};

#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

