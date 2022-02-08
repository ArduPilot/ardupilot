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
 * Code by Siddharth Bharat Purohit
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
#include "AP_HAL/utility/RingBuffer.h"
#include <AP_Param/AP_Param.h>
#include "AP_CANBridge.h"

#define SLCAN_BUFFER_SIZE 200
#define SLCAN_RX_QUEUE_SIZE 64

#ifndef HAL_CAN_RX_QUEUE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

static_assert(HAL_CAN_RX_QUEUE_SIZE <= 254, "Invalid CAN Rx queue size");

namespace MAVCAN
{

class CANIface: public CANBridge::CANIface
{

    int16_t reportFrame(const AP_HAL::CANFrame& frame, uint64_t timestamp_usec);

    bool initialized_;

    ObjectBuffer<AP_HAL::CANIface::CanRxItem> rx_queue_; // Parsed Rx Frame queue

    uint8_t _fwd_system_id;
    uint8_t _fwd_component_id;
public:
    CANIface()    {}

    // Overriden methods
    bool select(bool &read, bool &write,
                const AP_HAL::CANFrame* const pending_tx,
                uint64_t blocking_deadline) override;
    int16_t send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                 AP_HAL::CANIface::CanIOFlags flags) override;

    int16_t receive(AP_HAL::CANFrame& out_frame, uint64_t& rx_time,
                    AP_HAL::CANIface::CanIOFlags& out_flags) override;
};

}

#endif
