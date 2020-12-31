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
  generic CAN sensor class, for easy creation of CAN sensors using prioprietary protocols
 */
#include <AP_HAL/AP_HAL.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

#include <AP_Scheduler/AP_Scheduler.h>
#include "AP_CANSensor.h"

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, _driver_name, fmt, ##args); } while (0)

CANSensor::CANSensor(const char *driver_name, AP_CANManager::Driver_Type dtype, uint16_t stack_size) :
    _driver_name(driver_name),
    _stack_size(stack_size)
{
    if (!AP::can().register_driver(dtype, this)) {
        debug_can(AP_CANManager::LOG_ERROR, "Failed to register CANSensor %s", driver_name);
    } else {
        debug_can(AP_CANManager::LOG_INFO, "%s: constructed", driver_name);
    }
}

void CANSensor::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_INFO, "starting init");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "already initialized");
        return;
    }

    // get CAN manager instance
    _can_driver = AP::can().get_driver(driver_index);

    if (_can_driver == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "no CAN driver");
        return;
    }

    // start thread for receiving and sending CAN frames
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&CANSensor::loop, void), _driver_name, _stack_size, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        debug_can(AP_CANManager::LOG_ERROR, "couldn't create thread");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_INFO, "init done");

    return;
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

    if (!_can_iface->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "Cannot add event handle");
        return false;
    }
    return true;
}

void CANSensor::loop()
{
    while (!hal.scheduler->is_system_initialized()) {
        // don't process packets till startup complete
        hal.scheduler->delay(1);
    }
    const uint32_t LOOP_INTERVAL_US = AP::scheduler().get_loop_period_us();
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

#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

