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
 * Author: Dmitry Ponomarev
 */

#include <AP_UAVCAN_V1/AP_UAVCAN_V1_IfaceMgr.h>
#include <string.h>


extern const AP_HAL::HAL& hal;


void UavcanFirstTransportIface::attach_can_iface(AP_HAL::CANIface* new_can_iface)
{
    _can_iface = new_can_iface;
}

bool UavcanFirstTransportIface::receive(CanardFrame* canard_frame)
{
    if (_can_iface == nullptr) {
        return false;
    }

    bool read_select = true;
    bool write_select = false;
    uint64_t timeout = AP_HAL::native_micros64() + 10000;
    int ret = _can_iface->select(read_select, write_select, nullptr, timeout);
    if (!ret || !read_select) {
        return false; // no data is available to read
    }

    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags {};
    static AP_HAL::CANFrame can_frame;
    if (_can_iface->receive(can_frame, time, flags) == 1) {
        canard_frame->extended_can_id = can_frame.id;
        canard_frame->payload_size = can_frame.dlc;
        canard_frame->payload = can_frame.data;

        ///< all auxilliary info should be cleared
        canard_frame->extended_can_id &= ((UINT32_C(1) << 29U) - 1U);

        return true;
    }
    return false;
}


bool UavcanFirstTransportIface::send(const CanardTxQueueItem* transfer)
{
    if (_can_iface == nullptr || transfer->frame.payload_size == 0) {
        return false;
    }

    size_t num_of_frames = transfer->frame.payload_size / 8;
    size_t length_of_last_frame = transfer->frame.payload_size % 8;
    if (length_of_last_frame == 0) {
        length_of_last_frame = 8;
    } else {
        num_of_frames++;
    }

    AP_HAL::CANFrame can_frame;
    can_frame.id = transfer->frame.extended_can_id | AP_HAL::CANFrame::FlagEFF;

    bool result = false;
    for (size_t frame_idx = 0; frame_idx < num_of_frames; frame_idx++) {
        uint8_t payload_size = (frame_idx + 1 < num_of_frames) ? 8 : length_of_last_frame;

        can_frame.dlc = transfer->frame.payload_size;
        memcpy(can_frame.data, (void*)(((uint8_t*)transfer->frame.payload) + frame_idx * 8), payload_size);

        uint64_t timeout = AP_HAL::native_micros64() + 10000;
        bool read_select = false;
        bool write_select = true;

        bool ret;
        ret = _can_iface->select(read_select, write_select, &can_frame, timeout);
        if (!ret || !write_select) {
            result = false;
            break;  ///< skip the whole transfer
        }

        result = (_can_iface->send(can_frame, timeout, AP_HAL::CANIface::AbortOnError) == 1);
        if (!result) {
            break;  ///< skip the whole transfer
        }

        ///< we need to have a delay between each push (50 us is not enough)
        hal.scheduler->delay_microseconds(250);
    }

    return result;
}
