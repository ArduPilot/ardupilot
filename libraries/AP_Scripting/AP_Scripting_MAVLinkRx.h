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
/*
  MAVLink receive registrations and queue belonging to one script
 */
#pragma once

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include <GCS_MAVLink/GCS_config.h>

#if HAL_GCS_ENABLED

#include <stddef.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Scripting_MAVLinkRx {
public:
    AP_Scripting_MAVLinkRx(int _env_ref, AP_Scripting_MAVLinkRx *_next) :
        env_ref(_env_ref),
        next(_next) {}

    ~AP_Scripting_MAVLinkRx();

    CLASS_NO_COPY(AP_Scripting_MAVLinkRx);

    // allocate room for num_msgids registrations, returns false on
    // allocation failure.  The queue holds queue_len of the largest
    // message registered
    bool init(uint32_t queue_len, uint32_t num_msgids);

    enum class RegisterResult {
        ADDED,
        ALREADY_REGISTERED,
        NO_SPACE,
        OUT_OF_MEMORY,
    };
    RegisterResult register_msgid(uint32_t msgid);

    // queue msg if it has been registered for and there is room
    void handle_message(const mavlink_message_t &msg, mavlink_channel_t chan, uint32_t timestamp_ms);

    // fetch the oldest queued message, false if the queue is empty
    bool pop(mavlink_message_t &msg, mavlink_channel_t &chan, uint32_t &timestamp_ms);

    // environment reference of the owning script
    const int env_ref;

    AP_Scripting_MAVLinkRx *next;

private:
    // queued messages hold only the received payload bytes,
    // which follow this structure in the queue
    struct PACKED Record {
        uint32_t timestamp_ms;
        uint8_t chan;
        // mavlink_message_t up to the payload
        uint8_t header[offsetof(mavlink_message_t, payload64)];
        // mavlink_message_t from the checksum bytes to the end
        uint8_t tail[sizeof(mavlink_message_t) - offsetof(mavlink_message_t, ck)];
    };

    // grow the queue to hold queue_len messages of payload_len,
    // keeping its contents
    bool grow_queue(uint8_t payload_len);

    ByteBuffer *queue;
    uint32_t *msgids;
    uint16_t num_msgids;
    uint16_t used_msgids;
    uint16_t queue_len;
    // largest payload the queue is sized for
    uint8_t max_payload_len;
};

#endif  // HAL_GCS_ENABLED

#endif  // AP_SCRIPTING_ENABLED
