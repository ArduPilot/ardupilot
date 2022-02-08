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

void MAVCAN::CANIface::reportFrame(const AP_HAL::CANFrame& frame, uint64_t timestamp_usec)
{
    WITH_SEMAPHORE(port_sem);
    if (_mavchan == -1) {
        return;
    }
    if (HAVE_PAYLOAD_SPACE(chan, CAN_FRAME)) {
        mavlink_msg_can_frame_send(_mavchan, _fwd_system_id, _fwd_component_id,
                                   _iface_num, frame.dlc, frame.id, const_cast<uint8_t*>(frame.data));
    }
}

void MAVCAN::CANIface::handle_can_frame(const mavlink_message_t &msg) const
{
    WITH_SEMAPHORE(sem);
    mavlink_can_frame_t p;
    mavlink_msg_can_frame_decode(&msg, &p);
    if (_iface_num >= HAL_NUM_CAN_IFACES || _can_iface == nullptr) {
        return;
    }
    const uint16_t timeout_us = 2000;
    AP_HAL::CANFrame frame{p.id, p.data, p.len};
    _rx_queue.push(frame, AP_HAL::native_micros64() + timeout_us);
}

void MAVCAN::CANIface::enable_can_forward(uint8_t chan, uint8_t target_system, uint8_t target_component)
{
    WITH_SEMAPHORE(sem);
    if (!_rx_queue.set_size(HAL_CAN_RX_QUEUE_SIZE)) {
        return;
    }
    _mavchan = chan;
    _fwd_system_id = target_system;
    _fwd_component_id = target_component;
}

void MAVCAN::CANIface::disabled_can_forward()
{
    WITH_SEMAPHORE(port_sem);
    _mavchan = -1;
}

bool MAVCAN::CANIface::select(bool &read, bool &write, const AP_HAL::CANFrame* const pending_tx,
                             uint64_t blocking_deadline)
{
    bool ret = false;
    // When in passthrough mode select is handled through can iface
    if (_can_iface) {
        ret = _can_iface->select(read, write, pending_tx, blocking_deadline);
    }

    WITH_SEMAPHORE(sem);

    if (_mavchan == -1) {
        return ret;
    }

    // if under passthrough, we only do send when can_iface also allows it
    if (rx_queue_.available()) {
        // allow for receiving messages over slcan
        read = true;
        ret = true;
    }

    return ret;
}


// send method to transmit the frame through SLCAN interface
int16_t MAVCAN::CANIface::send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline, AP_HAL::CANIface::CanIOFlags flags)
{
    int16_t ret = 0;
    ret = CANBridge::CANIface::send(frame, tx_deadline, flags);

    if (_mavchan == -1) {
        return ret;
    }

    if (frame.isErrorFrame() || frame.dlc > 8) {
        return ret;
    }
    reportFrame(frame, AP_HAL::native_micros64());
    return ret;
}

// receive method to read the frame recorded in the buffer
int16_t MAVCAN::CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& rx_time,
                                 AP_HAL::CANIface::CanIOFlags& out_flags)
{
    // When in passthrough mode select is handled through can iface
    if (_can_iface) {
        int16_t ret = _can_iface->receive(out_frame, rx_time, out_flags);
        if (ret > 0) {
            // we also pass this frame through to mavlink iface,
            // and immediately return
            reportFrame(out_frame, AP_HAL::native_micros64());
            return ret;
        } else if (ret < 0) {
            return ret;
        }
    }

    // We found nothing in HAL's CANIface recieve, so look in SLCANIface
    if (_mavchan == -1) {
        return 0;
    }
    WITH_SEMAPHORE(port_sem);

    return CANBridge::CANIface::receive(out_frame, rx_time, out_flags);
}
#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
