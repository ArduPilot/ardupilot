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

AP_HAL::CANFrame::CANFrame(uint32_t can_id, const uint8_t* can_data, uint8_t data_len, bool canfd_frame) :
        id(can_id),
        canfd(canfd_frame)
{
    if ((can_data == nullptr) || (data_len == 0) || (data_len > MaxDataLen)) {
        return;
    }
    memcpy(this->data, can_data, data_len);
    if (data_len <= 8) {
        dlc = data_len;
    } else if (!canfd) {
        dlc = 8;
    } else {
        /*
        Data Length Code      9  10  11  12  13  14  15
        Number of data bytes 12  16  20  24  32  48  64
        */
        if (data_len <= 12) {
            dlc = 9;
        } else if (data_len <= 16) {
            dlc = 10;
        } else if (data_len <= 20) {
            dlc = 11;
        } else if  (data_len <= 24) {
            dlc = 12;
        } else if (data_len <= 32) {
            dlc = 13;
        } else if (data_len <= 48) {
            dlc = 14;
        } else if (data_len <= 64) {
            dlc = 15;
        }
    }
}

uint8_t AP_HAL::CANFrame::dataLengthToDlc(uint8_t data_length)
{
    if (data_length <= 8) {
        return data_length;
    } else if (data_length <= 12) {
        return 9;
    } else if (data_length <= 16) {
        return 10;
    } else if (data_length <= 20) {
        return 11;
    } else if (data_length <= 24) {
        return 12;
    } else if (data_length <= 32) {
        return 13;
    } else if (data_length <= 48) {
        return 14;
    }
    return 15;
}


uint8_t AP_HAL::CANFrame::dlcToDataLength(uint8_t dlc)
{
    /*
    Data Length Code      9  10  11  12  13  14  15
    Number of data bytes 12  16  20  24  32  48  64
    */
    if (dlc <= 8) {
        return dlc;
    } else if (dlc == 9) {
        return 12;
    } else if (dlc == 10) {
        return 16;
    } else if (dlc == 11) {
        return 20;
    } else if (dlc == 12) {
        return 24;
    } else if (dlc == 13) {
        return 32;
    } else if (dlc == 14) {
        return 48;
    }
    return 64;
}
