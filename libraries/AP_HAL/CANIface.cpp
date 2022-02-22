/*
 * Copyright (C) 2020 Siddharth B Purohit
 *
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
 */

#include "CANIface.h"
#include "system.h"

bool AP_HAL::CANFrame::priorityHigherThan(const CANFrame& rhs) const
{
    const uint32_t clean_id     = id     & MaskExtID;
    const uint32_t rhs_clean_id = rhs.id & MaskExtID;

    /*
     * STD vs EXT - if 11 most significant bits are the same, EXT loses.
     */
    const bool ext     = id     & FlagEFF;
    const bool rhs_ext = rhs.id & FlagEFF;
    if (ext != rhs_ext) {
        const uint32_t arb11     = ext     ? (clean_id >> 18)     : clean_id;
        const uint32_t rhs_arb11 = rhs_ext ? (rhs_clean_id >> 18) : rhs_clean_id;
        if (arb11 != rhs_arb11) {
            return arb11 < rhs_arb11;
        } else {
            return rhs_ext;
        }
    }

    /*
     * RTR vs Data frame - if frame identifiers and frame types are the same, RTR loses.
     */
    const bool rtr     = id     & FlagRTR;
    const bool rhs_rtr = rhs.id & FlagRTR;
    if (clean_id == rhs_clean_id && rtr != rhs_rtr) {
        return rhs_rtr;
    }

    /*
     * Plain ID arbitration - greater value loses.
     */
    return clean_id < rhs_clean_id;
}

/*
  parent class receive handling for MAVCAN
 */
int16_t AP_HAL::CANIface::receive(CANFrame& out_frame, uint64_t& out_ts_monotonic, CanIOFlags& out_flags)
{
    auto cb = frame_callback;
    if (cb && (out_flags & IsMAVCAN)==0) {
        cb(get_iface_num(), out_frame);
    }
    return 1;
}

/*
  parent class send handling for MAVCAN
 */
int16_t AP_HAL::CANIface::send(const CANFrame& frame, uint64_t tx_deadline, CanIOFlags flags)
{
    auto cb = frame_callback;
    if (cb) {
        if ((flags & IsMAVCAN) == 0) {
            cb(get_iface_num(), frame);
        } else {
            CanRxItem rx_item;
            rx_item.frame = frame;
            rx_item.timestamp_us = AP_HAL::native_micros64();
            rx_item.flags = AP_HAL::CANIface::IsMAVCAN;
            add_to_rx_queue(rx_item);
        }
    }
    return 1;
}

/*
  register a callback for for sending CAN_FRAME messages
 */
bool AP_HAL::CANIface::register_frame_callback(FrameCb cb)
{
    frame_callback = cb;
    return true;
}
