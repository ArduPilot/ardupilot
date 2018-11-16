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

#if HAL_WITH_UAVCAN
SLCANRouter* SLCANRouter::_singleton = nullptr;

extern const AP_HAL::HAL& hal;

SLCANRouter &slcan_router()
{
    return *SLCANRouter::instance();
}

void SLCANRouter::init(ChibiOS_CAN::CanIface* can_if, ChibiOS_CAN::BusEvent* update_event)
{
    can_if_ = can_if;
    if (!slcan_if_.is_initialized()) {
        if (slcan_if_.init(921600, SLCAN::CAN::OperatingMode::NormalMode) < 0) {
            return;
        }
    }
    update_event_ = update_event;
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&SLCANRouter::can2slcan_router_trampoline, void), "C2SRouter", 512, AP_HAL::Scheduler::PRIORITY_CAN, 0);
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&SLCANRouter::slcan2can_router_trampoline, void), "S2CRouter", 512, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}

void SLCANRouter::route_frame_to_slcan(ChibiOS_CAN::CanIface* can_if, const uavcan::CanFrame& frame, uint64_t timestamp_usec)
{
    if (can_if_ != can_if) {
        return;
    }
    CanRouteItem it;
    it.frame = frame;
    it.utc_usec = timestamp_usec;
    if (slcan_tx_queue_.space() == 0) {
        slcan_tx_queue_.pop();
    }
    slcan_tx_queue_.push(it);
}

void SLCANRouter::route_frame_to_can(const uavcan::CanFrame& frame, uint64_t timestamp_usec)
{
    CanRouteItem it;
    it.frame = frame;
    it.utc_usec = timestamp_usec;
    if (can_tx_queue_.space() == 0) {
        can_tx_queue_.pop();
    }
    can_tx_queue_.push(it);
}

void SLCANRouter::slcan2can_router_trampoline(void)
{
    CanRouteItem it;
    while(true) {
        chSysLock();
        _s2c_thd_ref = nullptr;        
        if (_thread_suspended) {
            _port->lock_port(0, 0);
            chThdSuspendS(&_s2c_thd_ref);
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
    while(true) {
        update_event_->wait(uavcan::MonotonicDuration::fromUSec(1000));
        if (slcan_tx_queue_.available()) {
            slcan_tx_queue_.peek(it);
            if (slcan_if_.send(it.frame, uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + 1000), 0)) {
                slcan_tx_queue_.pop();
            }
        }
    }
}

#endif
