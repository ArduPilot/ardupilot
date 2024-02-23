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
  generic CAN sensor class, for easy creation of CAN sensors using proprietary protocols
 */
#include <AP_HAL/AP_HAL.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

#include <AP_Scheduler/AP_Scheduler.h>
#include "AP_CANSensor.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, _driver_name, fmt, ##args); } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

CANSensor::CANSensor(const char *driver_name, uint16_t stack_size) :
    _driver_name(driver_name),
    _stack_size(stack_size)
{}


void CANSensor::register_driver(AP_CAN::Protocol dtype)
{
#if HAL_CANMANAGER_ENABLED
    if (!AP::can().register_driver(dtype, this)) {
        if (AP::can().register_11bit_driver(dtype, this, _driver_index)) {
            is_aux_11bit_driver = true;
            _can_driver = AP::can().get_driver(_driver_index);
            _initialized = true;
        } else {
            debug_can(AP_CANManager::LOG_ERROR, "Failed to register CANSensor %s", _driver_name);
        }
    } else {
        debug_can(AP_CANManager::LOG_INFO, "%s: constructed", _driver_name);
    }
#elif defined(HAL_BUILD_AP_PERIPH)
    register_driver_periph(dtype);
#endif
}


#ifdef HAL_BUILD_AP_PERIPH
CANSensor::CANSensor_Periph CANSensor::_periph[HAL_NUM_CAN_IFACES];

void CANSensor::register_driver_periph(const AP_CAN::Protocol dtype)
{
    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (_periph[i].protocol != dtype) {
            continue;
        }

        if (!add_interface(_periph[i].iface)) {
            continue;
        }

        init(0, false); // TODO: allow multiple drivers
        return;
    }
}
#endif

void CANSensor::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_INFO, "starting init");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "already initialized");
        return;
    }

#ifndef HAL_BUILD_AP_PERIPH
    // get CAN manager instance
    _can_driver = AP::can().get_driver(driver_index);

    if (_can_driver == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "no CAN driver");
        return;
    }
#endif

    // start thread for receiving and sending CAN frames
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&CANSensor::loop, void), _driver_name, _stack_size, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        debug_can(AP_CANManager::LOG_ERROR, "couldn't create thread");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_INFO, "init done");
}

bool CANSensor::add_interface(AP_HAL::CANIface* can_iface)
{
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "Multiple Interface not supported");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "CAN driver not found");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "Driver not initialized");
        return false;
    }

    if (!_can_iface->set_event_handle(&sem_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "Cannot add event handle");
        return false;
    }
    return true;
}

bool CANSensor::write_frame(AP_HAL::CANFrame &out_frame, const uint64_t timeout_us)
{
    if (!_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "Driver not initialized for write_frame");
        return false;
    }

    if (is_aux_11bit_driver && _can_driver != nullptr) {
        return _can_driver->write_aux_frame(out_frame, timeout_us);
    }

    bool read_select = false;
    bool write_select = true;
    bool ret = _can_iface->select(read_select, write_select, &out_frame, AP_HAL::micros64() + timeout_us);
    if (!ret || !write_select) {
        return false;
    }

    uint64_t deadline = AP_HAL::micros64() + 2000000;
    return (_can_iface->send(out_frame, deadline, AP_HAL::CANIface::AbortOnError) == 1);
}

void CANSensor::loop()
{
    while (!hal.scheduler->is_system_initialized()) {
        // don't process packets till startup complete
        hal.scheduler->delay(1);
    }

#ifdef HAL_BUILD_AP_PERIPH
    const uint32_t LOOP_INTERVAL_US = 1000;
#else
    const uint32_t LOOP_INTERVAL_US = AP::scheduler().get_loop_period_us();
#endif

    while (true) {
        uint64_t timeout = AP_HAL::micros64() + LOOP_INTERVAL_US;

        // wait to receive frame
        bool read_select = true;
        bool write_select = false;
        bool ret = _can_iface->select(read_select, write_select, nullptr, timeout);
        if (ret && read_select) {
            uint64_t time;
            AP_HAL::CANIface::CanIOFlags flags {};

            AP_HAL::CANFrame frame;
            int16_t res = _can_iface->receive(frame, time, flags);

            if (res == 1) {
                handle_frame(frame);
            }
        }
    }
}

MultiCAN::MultiCANLinkedList* MultiCAN::callbacks;

MultiCAN::MultiCAN(ForwardCanFrame cf, AP_CAN::Protocol can_type, const char *driver_name) :
        CANSensor(driver_name)
{
    if (callbacks == nullptr) {
        callbacks = new MultiCANLinkedList();
    }
    if (callbacks == nullptr) {
        AP_BoardConfig::allocation_error("Failed to create multican callback");
    }

    // Register new driver
    register_driver(can_type);
    callbacks->register_callback(cf);
}

// handle a received frame from the CAN bus
void MultiCAN::handle_frame(AP_HAL::CANFrame &frame)
{
    if (callbacks != nullptr) {
        callbacks->handle_frame(frame);
    }

}

// register a callback for a CAN frame by adding it to the linked list
void MultiCAN::MultiCANLinkedList::register_callback(ForwardCanFrame callback)
{
    CANSensor_Multi* newNode = new CANSensor_Multi();
    if (newNode == nullptr) {
        AP_BoardConfig::allocation_error("Failed to create multican node");
    }
    WITH_SEMAPHORE(sem);
    {
        newNode->_callback = callback;
        newNode->next = head;
        head = newNode;
    }
}

// distribute the CAN frame to the registered callbacks
void MultiCAN::MultiCANLinkedList::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(sem);
    for (CANSensor_Multi* current = head; current != nullptr; current = current->next) {
        if (current->_callback(frame)) {
            return;
        }
    }
}

#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

