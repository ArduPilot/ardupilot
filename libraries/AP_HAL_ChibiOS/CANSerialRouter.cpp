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
 * Siddharth Bharat Purohit
 */

#include "CANSerialRouter.h"

#if HAL_WITH_UAVCAN && !HAL_MINIMIZE_FEATURES
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
SLCANRouter* SLCANRouter::_singleton = nullptr;

extern const AP_HAL::HAL& hal;

SLCANRouter &slcan_router()
{
    return *SLCANRouter::instance();
}

void SLCANRouter::init(ChibiOS_CAN::CanIface* can_if, ChibiOS_CAN::BusEvent* update_event)
{
    _can_if = can_if;
    _update_event = update_event;
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&SLCANRouter::timer, void));
}

void SLCANRouter::run()
{
    _port = AP_SerialManager::get_instance()->get_serial_by_id(AP::can().get_slcan_serial());
    if (_slcan_if.init(921600, SLCAN::CAN::OperatingMode::NormalMode, _port) < 0) {
        return;
    }
    _slcan_rt_timeout = AP::can().get_slcan_timeout();
    if (!_thread_started) {
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&SLCANRouter::can2slcan_router_trampoline, void), "C2SRouter", 512, AP_HAL::Scheduler::PRIORITY_CAN, 0);
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&SLCANRouter::slcan2can_router_trampoline, void), "S2CRouter", 512, AP_HAL::Scheduler::PRIORITY_CAN, 0);
        _thread_started = true;
        _thread_suspended = false;
    }
    else if (_thread_suspended) {   //wake up threads
        chSysLock();
        if (_c2s_thd_ref && _s2c_thd_ref) {
            chThdResumeS(&_c2s_thd_ref, MSG_OK);
            chThdResumeS(&_s2c_thd_ref, MSG_OK);
            _thread_suspended = false;
        }
        chSysUnlock();
    }
}

void SLCANRouter::timer()
{
    if ((!_thread_started || _thread_suspended) && (AP::can().get_slcan_serial() != -1)) {
        run();
        AP::can().reset_slcan_serial();
        _last_active_time = AP_HAL::millis();
    }
    if (!_slcan_if.closed()) {
        _last_active_time = AP_HAL::millis();
    }
    if (_thread_suspended) {
        return;
    }
    if (AP_HAL::millis() - _last_active_time > (_slcan_rt_timeout * 1000) && _slcan_rt_timeout != 0) {
        chSysLock();
        _port->lock_port(0, 0);
        _port->flush();
        _thread_suspended = true;
        chSysUnlock();
    }
}

void SLCANRouter::route_frame_to_slcan(ChibiOS_CAN::CanIface* can_if, const uavcan::CanFrame& frame, uint64_t timestamp_usec)
{
    if (_can_if != can_if) {
        return;
    }
    CanRouteItem it;
    it.frame = frame;
    it.utc_usec = timestamp_usec;
    if (_slcan_tx_queue.space() == 0) {
        _slcan_tx_queue.pop();
    }
    _slcan_tx_queue.push(it);
}

void SLCANRouter::route_frame_to_can(const uavcan::CanFrame& frame, uint64_t timestamp_usec)
{
    CanRouteItem it;
    it.frame = frame;
    it.utc_usec = timestamp_usec;
    if (_can_tx_queue.space() == 0) {
        _can_tx_queue.pop();
    }
    _can_tx_queue.push(it);
}

void SLCANRouter::slcan2can_router_trampoline(void)
{
    CanRouteItem it;
    while (true) {
        chSysLock();
        _s2c_thd_ref = nullptr;
        if (_thread_suspended) {
            chThdSuspendS(&_s2c_thd_ref);
            _port->flush();
        }
        chSysUnlock();
        _slcan_if.reader();
        if (_can_tx_queue.available() && _can_if) {
            _can_tx_queue.peek(it);
            if (_can_if->send(it.frame, uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + 1000), 0)) {
                _can_tx_queue.pop();
            }
        }
    }
}

void SLCANRouter::can2slcan_router_trampoline(void)
{
    CanRouteItem it;
    while (true) {
        chSysLock();
        _c2s_thd_ref = nullptr;
        if (_thread_suspended) {
            chThdSuspendS(&_c2s_thd_ref);
        }
        chSysUnlock();
        _update_event->wait(uavcan::MonotonicDuration::fromUSec(1000));
        if (_slcan_tx_queue.available()) {
            _slcan_tx_queue.peek(it);
            if (_slcan_if.send(it.frame, uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + 1000), 0)) {
                _slcan_tx_queue.pop();
            }
        }
    }
}

#endif
