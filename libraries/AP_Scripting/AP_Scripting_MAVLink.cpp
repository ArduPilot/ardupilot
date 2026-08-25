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
  Scripting MAVLinkBuffer class, for easy scripting MAVLink support
 */

#include "AP_Scripting/AP_Scripting_MAVLink.h"
#include "AP_HAL/Semaphores.h"
#include <cstdint>

// read a message from the buffer
bool ScriptingMAVLinkBuffer::read_msg(mavlink_msg &msg) {
    return buffer.pop(msg);
}

// handler for incoming messages, add to buffers
void ScriptingMAVLinkBuffer::handle_msg(mavlink_msg &msg) {
    WITH_SEMAPHORE(sem);

    uint32_t id = msg.msg.msgid;
    bool valid = false;
    for(uint32_t i = 0; i < next_accepted_pos; i++) {
        if(id == accept_msg_ids[i]) {
            valid = true;
            break;
        }
    }

    if(valid) buffer.push(msg);

    if (next != nullptr) {
        next->handle_msg(msg);
    }
}

// add a new buffer to this list
void ScriptingMAVLinkBuffer::add_buffer(ScriptingMAVLinkBuffer* new_buff) {
    WITH_SEMAPHORE(sem);
    if (next == nullptr) {
        next = new_buff;
        return;
    }

    next->add_buffer(new_buff);
}

bool ScriptingMAVLinkBuffer::add_accepted_id(uint32_t id) {
    WITH_SEMAPHORE(sem);

    for(uint32_t i = 0; i < next_accepted_pos; i++) {
        if(accept_msg_ids[i] == id) return true;
    }

    if(next_accepted_pos < num_accepted_msgs) {
        accept_msg_ids[next_accepted_pos] = id;
        next_accepted_pos++;
        return true;
    }

    return false;
}
