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
  Scripting MAVLink class, for easy scripting MAVLink support
*/

#pragma once

#include "AP_Scripting/AP_Scripting.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/AP_HAL.h>

struct mavlink_msg {
    mavlink_message_t msg;
    mavlink_channel_t chan;
    uint32_t timestamp_ms;
};

class ScriptingMAVLinkBuffer {
friend class AP_Scripting;

public: ScriptingMAVLinkBuffer(uint32_t buffer_size, uint32_t num_msgs):
        buffer(buffer_size), accept_msg_ids(NEW_NOTHROW uint32_t[num_msgs]), num_accepted_msgs(num_msgs)
    {};

    ~ScriptingMAVLinkBuffer() {
        delete[] accept_msg_ids;
    }

    bool init_ok() const { return accept_msg_ids != nullptr; }

    // read a messages from the buffer
    bool read_msg(mavlink_msg &msg);

    // recursively add messages to buffer
    void handle_msg(mavlink_msg &msg);

    // recursively add new buffer
    void add_buffer(ScriptingMAVLinkBuffer* new_buff);

    // Add a filter to this buffer
    bool add_accepted_id(uint32_t id);

private:

    ObjectBuffer<mavlink_msg> buffer;

    uint32_t* accept_msg_ids;
    uint32_t num_accepted_msgs;
    uint32_t next_accepted_pos = 0;

    ScriptingMAVLinkBuffer *next = nullptr;

    HAL_Semaphore sem;

};

