/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Scripting_MAVLinkRx.h"

#if AP_SCRIPTING_ENABLED && HAL_GCS_ENABLED

AP_Scripting_MAVLinkRx::~AP_Scripting_MAVLinkRx()
{
    delete queue;
    delete[] msgids;
}

bool AP_Scripting_MAVLinkRx::init(uint32_t _queue_len, uint32_t _num_msgids)
{
    msgids = NEW_NOTHROW uint32_t[_num_msgids];
    if (_num_msgids > 0 && msgids == nullptr) {
        return false;
    }
    num_msgids = _num_msgids;
    queue_len = _queue_len;
    return true;
}

bool AP_Scripting_MAVLinkRx::grow_queue(uint8_t payload_len)
{
    // ByteBuffer holds one byte less than its size
    auto *new_queue = NEW_NOTHROW ByteBuffer(queue_len * (sizeof(Record) + payload_len) + 1);
    if (new_queue == nullptr || new_queue->get_size() == 0) {
        delete new_queue;
        return false;
    }
    if (queue != nullptr) {
        uint8_t buf[64];
        uint32_t n;
        while ((n = queue->read(buf, sizeof(buf))) > 0) {
            new_queue->write(buf, n);
        }
        delete queue;
    }
    queue = new_queue;
    max_payload_len = payload_len;
    return true;
}

AP_Scripting_MAVLinkRx::RegisterResult AP_Scripting_MAVLinkRx::register_msgid(uint32_t msgid)
{
    for (uint16_t i = 0; i < used_msgids; i++) {
        if (msgids[i] == msgid) {
            return RegisterResult::ALREADY_REGISTERED;
        }
    }
    if (used_msgids >= num_msgids) {
        return RegisterResult::NO_SPACE;
    }
    // messages we have no definition for could have any length
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
    const uint8_t payload_len = entry != nullptr ? entry->max_msg_len : MAVLINK_MAX_PAYLOAD_LEN;
    if ((queue == nullptr || payload_len > max_payload_len) && !grow_queue(payload_len)) {
        return RegisterResult::OUT_OF_MEMORY;
    }
    msgids[used_msgids++] = msgid;
    return RegisterResult::ADDED;
}

void AP_Scripting_MAVLinkRx::handle_message(const mavlink_message_t &msg, mavlink_channel_t chan, uint32_t timestamp_ms)
{
    bool wanted = false;
    for (uint16_t i = 0; i < used_msgids; i++) {
        if (msgids[i] == msg.msgid) {
            wanted = true;
            break;
        }
    }
    if (!wanted || queue->space() < sizeof(Record) + msg.len) {
        return;
    }

    Record record;
    record.timestamp_ms = timestamp_ms;
    record.chan = chan;
    memcpy(record.header, &msg, sizeof(record.header));
    memcpy(record.tail, &msg.ck, sizeof(record.tail));
    queue->write((const uint8_t *)&record, sizeof(record));
    queue->write((const uint8_t *)_MAV_PAYLOAD(&msg), msg.len);
}

bool AP_Scripting_MAVLinkRx::pop(mavlink_message_t &msg, mavlink_channel_t &chan, uint32_t &timestamp_ms)
{
    Record record;
    if (queue == nullptr || queue->read((uint8_t *)&record, sizeof(record)) != sizeof(record)) {
        return false;
    }
    // unreceived payload bytes are zero, as after parsing
    memset(&msg, 0, sizeof(msg));
    memcpy(&msg, record.header, sizeof(record.header));
    memcpy(&msg.ck, record.tail, sizeof(record.tail));
    queue->read((uint8_t *)_MAV_PAYLOAD_NON_CONST(&msg), msg.len);
    chan = mavlink_channel_t(record.chan);
    timestamp_ms = record.timestamp_ms;
    return true;
}

#endif  // AP_SCRIPTING_ENABLED && HAL_GCS_ENABLED
