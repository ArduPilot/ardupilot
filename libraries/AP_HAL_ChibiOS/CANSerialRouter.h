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

#pragma once

#include "AP_HAL_ChibiOS.h"

#if HAL_WITH_UAVCAN 
#include "CAN.h"
#include <AP_UAVCAN/AP_UAVCAN_SLCAN.h>

#include "CANThread.h"
#include "CANClock.h"
#include "CANIface.h"
#define SLCAN_ROUTER_QUEUE_SIZE 64

struct CanRouteItem {
    uint64_t utc_usec;
    uavcan::CanFrame frame;
    CanRouteItem() :
        utc_usec(0)
    {
    }
};

class SLCANRouter
{
    ChibiOS_CAN::CanIface* can_if_;
    SLCAN::CAN slcan_if_;
    ObjectBuffer<CanRouteItem> can_tx_queue_;
    ObjectBuffer<CanRouteItem> slcan_tx_queue_;
    static SLCANRouter* _singleton;
    ChibiOS_CAN::BusEvent* update_event_;
public:
    SLCANRouter() : slcan_if_(SLCAN_DRIVER_INDEX, SLCAN_RX_QUEUE_SIZE), can_tx_queue_(SLCAN_ROUTER_QUEUE_SIZE), slcan_tx_queue_(SLCAN_ROUTER_QUEUE_SIZE) 
    {
        if (_singleton  == nullptr) {
            _singleton = this;
        }
    }
    void init(ChibiOS_CAN::CanIface* can_if, ChibiOS_CAN::BusEvent* update_event);
    void route_frame_to_slcan(ChibiOS_CAN::CanIface* can_if, const uavcan::CanFrame& frame, uint64_t timestamp_usec);
    void route_frame_to_can(const uavcan::CanFrame& frame, uint64_t timestamp_usec);
    void slcan2can_router_trampoline(void);
    void can2slcan_router_trampoline(void);
    static SLCANRouter* instance() { 
        if (_singleton == nullptr) {
            _singleton = new SLCANRouter;
        }
        return _singleton; 
    }
};

SLCANRouter &slcan_router();

#endif
