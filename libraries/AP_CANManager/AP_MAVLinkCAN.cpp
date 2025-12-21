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
 */

#include "AP_MAVLinkCAN.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Common/sorting.h>

#if AP_MAVLINKCAN_ENABLED

extern const AP_HAL::HAL& hal;

static AP_MAVLinkCAN *singleton;

AP_MAVLinkCAN *AP_MAVLinkCAN::ensure_singleton()
{
    if (singleton == nullptr) {
        singleton = NEW_NOTHROW AP_MAVLinkCAN();
    }
    return singleton;
}

bool AP_MAVLinkCAN::handle_can_forward(mavlink_channel_t chan, const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    auto *s = ensure_singleton();
    if (s == nullptr) {
        return false;
    }
    return singleton->_handle_can_forward(chan, packet, msg);
}

void AP_MAVLinkCAN::handle_can_frame(const mavlink_message_t &msg)
{
    auto *s = ensure_singleton();
    if (s == nullptr) {
        return;
    }
    singleton->_handle_can_frame(msg);
}

void AP_MAVLinkCAN::handle_can_filter_modify(const mavlink_message_t &msg)
{
    auto *s = ensure_singleton();
    if (s == nullptr) {
        return;
    }
    singleton->_handle_can_filter_modify(msg);
}


/*
  handle MAV_CMD_CAN_FORWARD mavlink long command
 */
bool AP_MAVLinkCAN::_handle_can_forward(mavlink_channel_t chan, const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    WITH_SEMAPHORE(can_forward.sem);
    const int8_t bus = int8_t(packet.param1)-1;

    if (bus == -1) {
        /*
          a request to stop forwarding
         */
        if (can_forward.callback_id != 0) {
            hal.can[can_forward.callback_bus]->unregister_frame_callback(can_forward.callback_id);
            can_forward.callback_id = 0;
        }
        return true;
    }

    if (bus >= HAL_NUM_CAN_IFACES || hal.can[bus] == nullptr) {
        return false;
    }

    if (can_forward.callback_id != 0 && can_forward.callback_bus != bus) {
        /*
          the client is changing which bus they are monitoring, unregister from the previous bus
         */
        hal.can[can_forward.callback_bus]->unregister_frame_callback(can_forward.callback_id);
        can_forward.callback_id = 0;
    }

    if (can_forward.callback_id == 0 &&
        !hal.can[bus]->register_frame_callback(
            FUNCTOR_BIND_MEMBER(&AP_MAVLinkCAN::can_frame_callback, void, uint8_t, const AP_HAL::CANFrame &, AP_HAL::CANIface::CanIOFlags), can_forward.callback_id)) {
        // failed to register the callback
        return false;
    }

    can_forward.callback_bus = bus;
    can_forward.last_callback_enable_ms = AP_HAL::millis();
    can_forward.chan = chan;
    can_forward.system_id = msg.sysid;
    can_forward.component_id = msg.compid;

    return true;
}

/*
  handle a CAN_FRAME packet
 */
void AP_MAVLinkCAN::_handle_can_frame(const mavlink_message_t &msg)
{
    if (frame_buffer == nullptr) {
        // allocate frame buffer
        // 20 is good for firmware upload
        uint8_t buffer_size = 20;
        WITH_SEMAPHORE(frame_buffer_sem);
        while (frame_buffer == nullptr && buffer_size > 0) {
            // we'd like 20 frames, but will live with less
            frame_buffer = NEW_NOTHROW ObjectBuffer<BufferFrame>(buffer_size);
            if (frame_buffer != nullptr && frame_buffer->get_size() != 0) {
                // register a callback for when frames can't be sent immediately
                hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_MAVLinkCAN::process_frame_buffer, void));
                break;
            }
            delete frame_buffer;
            frame_buffer = nullptr;
            buffer_size /= 2;
        }
        if (frame_buffer == nullptr) {
            // discard the frames
            return;
        }
    }

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_CAN_FRAME: {
        mavlink_can_frame_t p;
        mavlink_msg_can_frame_decode(&msg, &p);
        if (p.bus >= HAL_NUM_CAN_IFACES || hal.can[p.bus] == nullptr) {
            return;
        }
        struct BufferFrame frame {
            bus : p.bus,
            frame : AP_HAL::CANFrame(p.id, p.data, p.len)
        };
        {
            WITH_SEMAPHORE(frame_buffer_sem);
            frame_buffer->push(frame);
        }
        break;
    }
#if HAL_CANFD_SUPPORTED
    case MAVLINK_MSG_ID_CANFD_FRAME: {
        mavlink_canfd_frame_t p;
        mavlink_msg_canfd_frame_decode(&msg, &p);
        if (p.bus >= HAL_NUM_CAN_IFACES || hal.can[p.bus] == nullptr) {
            return;
        }
        struct BufferFrame frame {
            bus : p.bus,
            frame : AP_HAL::CANFrame(p.id, p.data, p.len, true)
        };
        {
            WITH_SEMAPHORE(frame_buffer_sem);
            frame_buffer->push(frame);
        }
        break;
    }
#endif
    }
    process_frame_buffer();
}

/*
  process the frame buffer
 */
void AP_MAVLinkCAN::process_frame_buffer()
{
    while (frame_buffer) {
        WITH_SEMAPHORE(frame_buffer_sem);
        struct BufferFrame frame;
        const uint16_t timeout_us = 2000;
        if (!frame_buffer->peek(frame)) {
            // no frames in the queue
            break;
        }
        const int16_t retcode = hal.can[frame.bus]->send(frame.frame,
                                                            AP_HAL::micros64() + timeout_us,
                                                            AP_HAL::CANIface::IsForwardedFrame);
        if (retcode == 0) {
            // no space in the CAN output slots, try again later
            break;
        }
        // retcode == 1 means sent, -1 means a frame that can't be
        // sent. Either way we should remove from the queue
        frame_buffer->pop();
    }
}

/*
  handle a CAN_FILTER_MODIFY packet
 */
void AP_MAVLinkCAN::_handle_can_filter_modify(const mavlink_message_t &msg)
{
    mavlink_can_filter_modify_t p;
    mavlink_msg_can_filter_modify_decode(&msg, &p);
    const int8_t bus = int8_t(p.bus)-1;
    if (bus >= HAL_NUM_CAN_IFACES || hal.can[bus] == nullptr) {
        return;
    }
    if (p.num_ids > ARRAY_SIZE(p.ids)) {
        return;
    }
    uint16_t *new_ids = nullptr;
    uint16_t num_new_ids = 0;
    WITH_SEMAPHORE(can_forward.sem);

    // sort the list, so we can bisection search and the array
    // operations below are efficient
    insertion_sort_uint16(p.ids, p.num_ids);
    
    switch (p.operation) {
    case CAN_FILTER_REPLACE: {
        if (p.num_ids == 0) {
            can_forward.num_filter_ids = 0;
            delete[] can_forward.filter_ids;
            can_forward.filter_ids = nullptr;
            return;
        }
        if (p.num_ids == can_forward.num_filter_ids &&
            memcmp(p.ids, can_forward.filter_ids, p.num_ids*sizeof(uint16_t)) == 0) {
            // common case of replacing with identical list
            return;
        }
        new_ids = NEW_NOTHROW uint16_t[p.num_ids];
        if (new_ids != nullptr) {
            num_new_ids = p.num_ids;
            memcpy((void*)new_ids, (const void *)p.ids, p.num_ids*sizeof(uint16_t));
        }
        break;
    }
    case CAN_FILTER_ADD: {
        if (common_list_uint16(can_forward.filter_ids, can_forward.num_filter_ids,
                               p.ids, p.num_ids) == p.num_ids) {
            // nothing changing
            return;
        }
        new_ids = NEW_NOTHROW uint16_t[can_forward.num_filter_ids+p.num_ids];
        if (new_ids == nullptr) {
            return;
        }
        if (can_forward.num_filter_ids != 0) {
            memcpy(new_ids, can_forward.filter_ids, can_forward.num_filter_ids*sizeof(uint16_t));
        }
        memcpy(&new_ids[can_forward.num_filter_ids], p.ids, p.num_ids*sizeof(uint16_t));
        insertion_sort_uint16(new_ids, can_forward.num_filter_ids+p.num_ids);
        num_new_ids = remove_duplicates_uint16(new_ids, can_forward.num_filter_ids+p.num_ids);
        break;
    }
    case CAN_FILTER_REMOVE: {
        if (common_list_uint16(can_forward.filter_ids, can_forward.num_filter_ids,
                               p.ids, p.num_ids) == 0) {
            // nothing changing
            return;
        }
        can_forward.num_filter_ids = remove_list_uint16(can_forward.filter_ids, can_forward.num_filter_ids,
                                                        p.ids, p.num_ids);
        if (can_forward.num_filter_ids == 0) {
            delete[] can_forward.filter_ids;
            can_forward.filter_ids = nullptr;
        }
        break;
    }
    }
    if (new_ids != nullptr) {
        // handle common case of no change
        if (num_new_ids == can_forward.num_filter_ids &&
            memcmp(new_ids, can_forward.filter_ids, num_new_ids*sizeof(uint16_t)) == 0) {
            delete[] new_ids;
        } else {
            // put the new list in place
            delete[] can_forward.filter_ids;
            can_forward.filter_ids = new_ids;
            can_forward.num_filter_ids = num_new_ids;
        }
    }
}

/*
  handler for CAN frames from the registered callback, sending frames
  out as CAN_FRAME or CANFD_FRAME messages
 */
void AP_MAVLinkCAN::can_frame_callback(uint8_t bus, const AP_HAL::CANFrame &frame, AP_HAL::CANIface::CanIOFlags flags)
{
    mavlink_channel_t chan;
    uint8_t system_id;
    uint8_t component_id;
    {
        WITH_SEMAPHORE(can_forward.sem);
        if (bus != can_forward.callback_bus) {
            // we are not registered for forwarding this bus, discard frame
            return;
        }
        if (can_forward.frame_counter++ == 100) {
            // check every 100 frames for disabling CAN_FRAME send
            // we stop sending after 5s if the client stops
            // sending MAV_CMD_CAN_FORWARD requests
            if (can_forward.callback_id != 0 &&
                AP_HAL::millis() - can_forward.last_callback_enable_ms > 5000) {
                hal.can[bus]->unregister_frame_callback(can_forward.callback_id);
                can_forward.callback_id = 0;
                return;
            }
            can_forward.frame_counter = 0;
        }
        if (can_forward.filter_ids != nullptr) {
            // work out ID of this frame
            uint16_t id = 0;
            if ((frame.id&0xff) != 0) {
                // not anonymous
                if (frame.id & 0x80) {
                    // service message
                    id = uint8_t(frame.id>>16);
                } else {
                    // message frame
                    id = uint16_t(frame.id>>8);
                }
            }
            if (!bisect_search_uint16(can_forward.filter_ids, can_forward.num_filter_ids, id)) {
                return;
            }
        }
        // remeber destination while we hold the mutex
        chan = can_forward.chan;
        system_id = can_forward.system_id;
        component_id = can_forward.component_id;
    }

    // the rest is run without the can_forward.sem
    WITH_SEMAPHORE(comm_chan_lock(chan));
    const uint8_t data_len = AP_HAL::CANFrame::dlcToDataLength(frame.dlc);
#if HAL_CANFD_SUPPORTED
    if (frame.isCanFDFrame()) {
        if (HAVE_PAYLOAD_SPACE(chan, CANFD_FRAME)) {
            mavlink_msg_canfd_frame_send(chan, system_id, component_id,
                                         bus, data_len, frame.id, const_cast<uint8_t*>(frame.data));
        }
    } else
#endif
    {
        if (HAVE_PAYLOAD_SPACE(chan, CAN_FRAME)) {
            mavlink_msg_can_frame_send(chan, system_id, component_id,
                                       bus, data_len, frame.id, const_cast<uint8_t*>(frame.data));
        }
    }
}

#endif  // AP_MAVLINKCAN_ENABLED
