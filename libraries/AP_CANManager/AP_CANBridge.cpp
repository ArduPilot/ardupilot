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
 * Author: Siddharth Bharat Purohit
 * Referenced from implementation by Pavel Kirienko <pavel.kirienko@zubax.com>
 * for Zubax Babel
 */

#include "AP_CANBridgeIface.h"

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

#include "AP_CANManager.h"

#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
#include <AP_Vehicle/AP_Vehicle.h>

#define LOG_TAG "Bridge"

extern const AP_HAL::HAL& hal;

bool CANBridge::CANIface::push_Frame(AP_HAL::CANFrame &frame)
{
    AP_HAL::CANIface::CanRxItem frm;
    frm.frame = frame;
    frm.flags = 0;
    frm.timestamp_us = AP_HAL::native_micros64();
    return rx_queue_.push(frm);
}

bool CANBridge::CANIface::set_event_handle(AP_HAL::EventHandle* evt_handle)
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        return _can_iface->set_event_handle(evt_handle);
    }
    return false;
}

uint16_t CANBridge::CANIface::getNumFilters() const
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        return _can_iface->getNumFilters();
    }
    return 0;
}

uint32_t CANBridge::CANIface::getErrorCount() const
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        return _can_iface->getErrorCount();
    }
    return 0;
}

void CANBridge::CANIface::get_stats(ExpandingString &str)
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        _can_iface->get_stats(str);
    }
}

bool CANBridge::CANIface::is_busoff() const
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        return _can_iface->is_busoff();
    }
    return false;
}

bool CANBridge::CANIface::configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs)
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        return _can_iface->configureFilters(filter_configs, num_configs);
    }
    return true;
}

void CANBridge::CANIface::flush_tx()
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        _can_iface->flush_tx();
    }
}

void CANBridge::CANIface::clear_rx()
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        _can_iface->clear_rx();
    }
    rx_queue_.clear();
}

bool CANBridge::CANIface::is_initialized() const
{
    // When in passthrough mode methods is handled through can iface
    if (_can_iface) {
        return _can_iface->is_initialized();
    }
    return false;
}

bool CANBridge::CANIface::select(bool &read, bool &write, const AP_HAL::CANFrame* const pending_tx,
                             uint64_t blocking_deadline)
{
    // When in passthrough mode select is handled through can iface
    if (_can_iface) {
        return _can_iface->select(read, write, pending_tx, blocking_deadline);
    }

    return false;
}


// send method to transmit the frame through CANBridge interface
int16_t CANBridge::CANIface::send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline, AP_HAL::CANIface::CanIOFlags flags)
{
    // When in passthrough mode select is handled through can iface
    if (_can_iface) {
        return _can_iface->send(frame, tx_deadline, flags);
    }

    return 0;
}

// receive method to read the frame recorded in the buffer
int16_t CANBridge::CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& rx_time,
                                 AP_HAL::CANIface::CanIOFlags& out_flags)
{
    if (rx_queue_.available()) {
        // if we already have something in buffer transmit it
        CanRxItem frm;
        if (!rx_queue_.peek(frm)) {
            return 0;
        }
        out_frame = frm.frame;
        rx_time = frm.timestamp_us;
        out_flags = frm.flags;
        _last_had_activity = AP_HAL::millis();
        // Also send this frame over can_iface when in passthrough mode,
        // We just push this frame without caring for priority etc
        if (_can_iface) {
            bool read = false;
            bool write = true;
            _can_iface->select(read, write, &out_frame, 0); // select without blocking
            if (write && _can_iface->send(out_frame, AP_HAL::native_micros64() + 100000, out_flags) == 1) {
                    rx_queue_.pop();
                    num_tries = 0;
            } else if (num_tries > 8) {
                rx_queue_.pop();
                num_tries = 0;
            } else {
                num_tries++;
            }
        } else {
            // we just throw away frames if we don't
            // have any can iface to pass through to
            rx_queue_.pop();
        }
        return 1;
    }
    return 0;
}

#endif
